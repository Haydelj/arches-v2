#include "unit-ray-steam-buffer.hpp"

namespace Arches { namespace Units { namespace STRaTA {

void UnitRayStreamBuffer::clock_rise()
{
	_request_network.clock();

	for(uint i = 0; i < _banks.size(); ++i)
		_proccess_request(i);
}

void UnitRayStreamBuffer::clock_fall()
{
	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		Bank& bank = _banks[bank_index];
		bank.data_pipline.clock();

		if(!bank.data_pipline.is_read_valid()) continue;

		const MemoryRequest& req = bank.data_pipline.peek();
		if(req.size == sizeof(STRaTAHitReturn))		// process load hit
		{
			_assert(req.type == MemoryRequest::Type::LOAD);
			HitRequest hit_req;
			hit_req.paddr = req.paddr;
			hit_req.port = req.port;
			hit_req.dst = req.dst;
			uint16_t priority = req.paddr >> 31;
			_hit_load_queue[priority][bank_index].push(hit_req);
			bank.data_pipline.read();
		}
		else
		{
			_assert(req.size == sizeof(RayData));
			if(req.type == MemoryRequest::Type::LOAD)
			{
				_raydata_request_queue[bank_index][req.port].push(req.dst);
				bank.data_pipline.read();
				log.ray_request_push_count++;
			}
			else if(req.type == MemoryRequest::Type::STORE)
			{
				RayData ray_data;
				std::memcpy(&ray_data, req.data, req.size);
				if(ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::OVER)
				{
					_complete_ray_buffers.push_back(ray_data);
					_ray_buffers_size += sizeof(RayData);
				}
				else
				{
					if(_ray_buffers.count(ray_data.raystate.treelet_id) == 0)
					{
						_ray_buffers[ray_data.raystate.treelet_id].push_back(ray_data);
						_ray_buffers_size += sizeof(RayData);
						_idle_ray_buffer.insert(ray_data.raystate.treelet_id);
					}
					else
					{
						_ray_buffers[ray_data.raystate.treelet_id].push_back(ray_data);
						_ray_buffers_size += sizeof(RayData);
					}
				}
				log.stores++;
				log.bytes_written += req.size;
				bank.data_pipline.read();
			}
		}
	}
	_issue_returns();
	_return_network.clock();
}

void UnitRayStreamBuffer::_proccess_request(uint bank_index)
{
	Bank& bank = _banks[bank_index];
	_assert(_ray_buffers_size <= _max_size);

	if(!_request_network.is_read_valid(bank_index) || !bank.data_pipline.is_write_valid())
		return;

	bank.data_pipline.write(_request_network.read(bank_index));
}

MemoryReturn UnitRayStreamBuffer::allocate_ray_buffer(uint tm_index, uint dst)
{
	MemoryReturn ret;
	uint32_t treelet_id = *_idle_ray_buffer.begin();
	_tm_buffer_table[tm_index] = treelet_id;
	_assert(_ray_buffers[treelet_id].size() > 0);
	std::memcpy(ret.data, &_ray_buffers[treelet_id].back(), sizeof(RayData));
	ret.size = sizeof(RayData);
	ret.port = tm_index;
	ret.dst = dst;
	_ray_buffers[treelet_id].pop_back();
	_idle_ray_buffer.erase(treelet_id);
	return ret;
}

void UnitRayStreamBuffer::_return_hit(std::queue<HitRequest>& queue, uint32_t bank_index)
{
	HitRequest hit_req = queue.front();
	const RayData& ray_data = _complete_ray_buffers.back();
	STRaTAHitReturn hit_return;
	rtm::Hit hit;
	hit.t = ray_data.raystate.hit_t;
	hit.id = ray_data.raystate.hit_id;
	hit.bc = rtm::vec2(0.0f);
	hit_return.hit = hit;
	hit_return.index = ray_data.raystate.id;
	MemoryReturn ret;
	ret.size = sizeof(STRaTAHitReturn);
	ret.port = hit_req.port;
	ret.dst = hit_req.dst;
	ret.paddr = hit_req.paddr;
	std::memcpy(ret.data, &hit_return, sizeof(STRaTAHitReturn));
	log.loads++;
	log.bytes_read += sizeof(STRaTAHitReturn);
	_complete_ray_buffers.pop_back();
	_ray_buffers_size -= sizeof(RayData);
	_return_network.write(ret, bank_index);
	queue.pop();
}

void UnitRayStreamBuffer::_issue_returns()
{
	std::vector<uint32_t> idle_banks(_banks.size(), 0u);
	// return higher priority hits first
	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if(!_hit_load_queue[0][bank_index].empty() && _return_network.is_write_valid(bank_index) && !_complete_ray_buffers.empty())
		{
			_return_hit(_hit_load_queue[0][bank_index], bank_index);
			idle_banks[bank_index] = ~0u;
		}
	}
	// return lower priority hits
	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if((idle_banks[bank_index] != ~0u) && !_hit_load_queue[1][bank_index].empty() && _return_network.is_write_valid(bank_index) && !_complete_ray_buffers.empty())
		{
			_return_hit(_hit_load_queue[1][bank_index], bank_index);
			idle_banks[bank_index] = ~0u;
		}
	}

	// process raydata load request
	for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
	{
		if ((idle_banks[bank_index] != ~0u) && !_raydata_request_queue[bank_index].empty() && _return_network.is_write_valid(bank_index))
		{
			for(auto itr = _raydata_request_queue[bank_index].begin(); itr != _raydata_request_queue[bank_index].end(); ++itr)
			{
				if (itr->second.empty())
					continue;
				uint32_t port = itr->first;
				uint32_t dst = itr->second.front();
				MemoryReturn ret;
				if (_tm_buffer_table[port] == ~0u)		// allocate a ray buffer to the tm
				{
					if (_idle_ray_buffer.empty())
					{
						continue;
					}
					ret = allocate_ray_buffer(port, dst);
				}
				else
				{
					uint32_t treelet_id = _tm_buffer_table[port];
					if (_ray_buffers[treelet_id].size() == 0)	// if the ray buffer is empty, allocate a new ray buffer to the tm
					{
						_ray_buffers.erase(treelet_id);
						_tm_buffer_table[port] = ~0u;
						if (_idle_ray_buffer.empty())
						{
							continue;
						}
						ret = allocate_ray_buffer(port, dst);
					}
					else
					{
						std::memcpy(ret.data, &_ray_buffers[treelet_id].back(), sizeof(RayData));
						ret.size = sizeof(RayData);
						ret.port = port;
						ret.dst = dst;
						_ray_buffers[treelet_id].pop_back();
					}
				}
				_ray_buffers_size -= sizeof(RayData);
				itr->second.pop();
				log.ray_request_pop_count++;
				log.loads++;
				log.bytes_read += sizeof(RayData);
				_return_network.write(ret, bank_index);
				break;
			}
		}
	}
}

}}}