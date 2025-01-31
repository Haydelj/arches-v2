#include "unit-ray-stream-buffer.hpp"

namespace Arches { namespace Units { namespace STRaTART {

UnitRayStreamBuffer::UnitRayStreamBuffer(const Configuration& config) : _rtc_max_rays(config.rtc_max_rays),
_request_network(config.num_tm, config.num_banks), _return_network(config.num_banks, config.num_tm),
_banks(config.num_banks, config.latency), _tm_states(config.num_tm), arb(config.num_banks), _cheat_treelets(config.cheat_treelets)
{
	_buffer_address_mask = generate_nbit_mask(log2i(config.size));
}

void UnitRayStreamBuffer::clock_rise()
{
	_request_network.clock();
	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		Bank& bank = _banks[bank_index];
		bank.request_pipline.clock();
		if(!_request_network.is_read_valid(bank_index) || !bank.request_pipline.is_write_valid()) continue;

		MemoryRequest req = _request_network.read(bank_index);
		bank.request_pipline.write(req);
	}
}

void UnitRayStreamBuffer::clock_fall()
{
	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		Bank& bank = _banks[bank_index];
		if(!bank.request_pipline.is_read_valid()) continue;

		const MemoryRequest req = bank.request_pipline.read();
		if(req.size == sizeof(STRaTARTKernel::HitReturn)) // process load hit
		{
			_assert(req.type == MemoryRequest::Type::LOAD);
			bank.hit_load_queue.push(req);
		}
		else
		{
			if(req.type == MemoryRequest::Type::LOAD)
			{
				_assert(req.size == sizeof(STRaTARTKernel::RayData));
				uint tm_index = req.port;
				_tm_states[tm_index].ray_load_queue.push(req);
			}
			else if(req.type == MemoryRequest::Type::STORE)
			{
				STRaTARTKernel::RayData ray_data;
				std::memcpy(&ray_data, req.data, sizeof(STRaTARTKernel::RayData));
				if(ray_data.restart_trail.is_done())
				{
					_completed_rays.push(ray_data);
				}
				else
				{
					uint treelet_index = ray_data.treelet_id;
					_treelet_states[treelet_index].rays.push(ray_data);
				}
				log.bytes_written += sizeof(STRaTARTKernel::RayData);
			}
		}
	}

	for(auto& a : _treelet_states)
	{
		if(a.second.rays.empty() && a.second.num_tms == 0)
			a.second.first_ray = true;
	}

	for(uint i = 0; i < _tm_states.size(); ++i)
	{
		TMState& tm_state = _tm_states[i];
		uint& treelet_index = tm_state.current_treelet;
		if(_treelet_states[treelet_index].rays.empty() && tm_state.ray_load_queue.size() >= 32)
		{
			uint i = 0;
			uint best_treelet = ~0u;
			float best_rate = 0.0;
			for(auto& a : _treelet_states)
			{
				if(a.second.num_tms > 0 && a.second.rays.size() / a.second.num_tms < 256) continue;

				float rate = (float)a.second.rays.size() / (a.second.num_tms + 1);
				if(rate > best_rate)
				{
					best_rate = rate;
					best_treelet = a.first;
					break;
				}
			}

			if(best_treelet != ~0u)
			{
				if(treelet_index != ~0u)
					_treelet_states[treelet_index].num_tms--;

				_treelet_states[best_treelet].num_tms++;
				treelet_index = best_treelet;
				//for(uint i = 0; i < _tm_states.size(); ++i)
				//{
				//	printf("%04d(%03d), ", _tm_states[i].current_treelet, _tm_states[i].ray_load_queue.size());
				//	if(i % 16 == 15) printf("\n");
				//}
				//printf("\033[%dA\n", _tm_states.size() / 16 + 1);
			}
		}
	}


	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		Bank& bank = _banks[bank_index];
		if(!bank.hit_load_queue.empty() && _return_network.is_write_valid(bank_index))
			arb.add(bank_index);
	}

	while(arb.num_pending() > 0 && !_completed_rays.empty())
	{
		uint bank_index = arb.get_index();
		arb.remove(bank_index);
		Bank& bank = _banks[bank_index];

		STRaTARTKernel::HitReturn hit_ret;
		hit_ret.hit = _completed_rays.front().hit;
		hit_ret.index = _completed_rays.front().global_ray_id;
		_completed_rays.pop();

		MemoryReturn ret(bank.hit_load_queue.top(), &hit_ret);
		_return_network.write(ret, bank_index);
		bank.hit_load_queue.pop();
		log.bytes_read += sizeof(STRaTARTKernel::HitReturn);

		log.hits++;
	}


	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		Bank& bank = _banks[bank_index];
		if(_return_network.is_write_valid(bank_index))
		{
			uint tms_per_bank = _tm_states.size() / _banks.size();
			for(uint i = 0; i < tms_per_bank; ++i)
			{
				TMState& tm_state = _tm_states[tms_per_bank * bank_index + i];
				if(!tm_state.ray_load_queue.empty()
					&& tm_state.current_treelet != ~0u
					&& !_treelet_states[tm_state.current_treelet].rays.empty())
				{
					uint8_t pf_mask = 0x0;
					if(_treelet_states[tm_state.current_treelet].first_ray)
					{
						for(uint i = 0; i < 8; ++i)
						{
							float a = _cheat_treelets[tm_state.current_treelet].header.page_sah[i];
							uint pf = std::powf(1.0f - a, _treelet_states[tm_state.current_treelet].rays.size()) < 0.75;
							pf_mask |= pf << i;
						}
						_treelet_states[tm_state.current_treelet].first_ray = false;
					}

					const MemoryRequest& req = tm_state.ray_load_queue.front();
					STRaTARTKernel::RayData ray_data = _treelet_states[tm_state.current_treelet].rays.front();
					ray_data.pf_mask = pf_mask;

					MemoryReturn ret(req, &ray_data);
					_return_network.write(ret, bank_index);

					_treelet_states[tm_state.current_treelet].rays.pop();
					tm_state.ray_load_queue.pop();

					log.bytes_read += ret.size;
					break;
				}
			}
		}
	}
	_return_network.clock();
}

}}}