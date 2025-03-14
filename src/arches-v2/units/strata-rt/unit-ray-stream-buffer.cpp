#include "unit-ray-stream-buffer.hpp"

namespace Arches { namespace Units { namespace STRaTART {

UnitRayStreamBuffer::UnitRayStreamBuffer(const Configuration& config) : _rtc_max_rays(config.rtc_max_rays),
_request_network(config.num_tm, config.num_banks), _return_network(config.num_banks, config.num_tm),
_banks(config.num_banks, config.latency), _tm_states(config.num_tm), arb(config.num_banks), _cheat_treelets(config.cheat_treelets), arb_0(config.num_banks), arb_1(config.num_banks)
{
	_buffer_address_mask = generate_nbit_mask(log2i(config.size));
	_hit_load_queue.resize(2);
	for (uint i = 0; i < 2; i++)
	{
		_hit_load_queue[i].resize(config.num_banks);
	}
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

void UnitRayStreamBuffer::_return_hit(std::queue<HitRequest>& queue, uint32_t bank_index)
{
	HitRequest hit_req = queue.front();
	const STRaTARTKernel::RayData& ray_data = _complete_ray_buffers.back();
	STRaTARTKernel::HitReturn hit_return;
	rtm::Hit hit = ray_data.hit;
	hit_return.hit = hit;
	hit_return.index = ray_data.global_ray_id;
	MemoryReturn ret;
	ret.size = sizeof(STRaTARTKernel::HitReturn);
	ret.port = hit_req.port;
	ret.dst = hit_req.dst;
	ret.paddr = hit_req.paddr;
	std::memcpy(ret.data, &hit_return, sizeof(STRaTARTKernel::HitReturn));
	log.hits++;
	log.bytes_read += sizeof(STRaTARTKernel::HitReturn);
	_complete_ray_buffers.pop_back();
	_return_network.write(ret, bank_index);
	queue.pop();
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
			//bank.hit_load_queue.push(req);
			HitRequest hit_req;
			hit_req.paddr = req.paddr;
			hit_req.port = req.port;
			hit_req.dst = req.dst;
			uint16_t priority = req.paddr >> 4;
			_hit_load_queue[priority][bank_index].push(hit_req);
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
					_complete_ray_buffers.push_back(ray_data);
				}
				else
				{
					while (ray_data.level < _cheat_treelets[ray_data.treelet_id].header.root_node_level)
					{
						ray_data.treelet_id = _cheat_treelets[ray_data.treelet_id].header.parent_treelet_index;
					}
					ray_data.level = _cheat_treelets[ray_data.treelet_id].header.root_node_level;
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


	/*for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
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
	}*/

	std::vector<uint32_t> idle_banks(_banks.size(), 0u);
	// return higher priority hits first
	for (uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if (!_hit_load_queue[0][bank_index].empty() && _return_network.is_write_valid(bank_index))
			arb_0.add(bank_index);
	}
	while (arb_0.num_pending() > 0 && !_complete_ray_buffers.empty())
	{
		uint bank_index = arb_0.get_index();
		arb_0.remove(bank_index);
		if (_return_network.is_write_valid(bank_index))
			_return_hit(_hit_load_queue[0][bank_index], bank_index);
		idle_banks[bank_index] = ~0u;
	}
	/*for (uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if (!_hit_load_queue[0][bank_index].empty() && _return_network.is_write_valid(bank_index) && !_complete_ray_buffers.empty())
		{
			_return_hit(_hit_load_queue[0][bank_index], bank_index);
			idle_banks[bank_index] = ~0u;
		}
	}*/

	// return lower priority hits
	for (uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if ((idle_banks[bank_index] != ~0u) && !_hit_load_queue[1][bank_index].empty() && _return_network.is_write_valid(bank_index))
			arb_1.add(bank_index);
	}
	while (arb_1.num_pending() > 0 && !_complete_ray_buffers.empty())
	{
		uint bank_index = arb_1.get_index();
		arb_1.remove(bank_index);
		if(_return_network.is_write_valid(bank_index))
			_return_hit(_hit_load_queue[1][bank_index], bank_index);
		idle_banks[bank_index] = ~0u;
	}
	/*for (uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if ((idle_banks[bank_index] != ~0u) && !_hit_load_queue[1][bank_index].empty() && _return_network.is_write_valid(bank_index) && !_complete_ray_buffers.empty())
		{
			_return_hit(_hit_load_queue[1][bank_index], bank_index);
			idle_banks[bank_index] = ~0u;
		}
	}*/


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