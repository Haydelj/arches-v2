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
		if(req.type == MemoryRequest::Type::LOAD)
		{
			if(!_return_network.is_write_valid(bank_index)) continue;
			_assert(req.paddr < 0x1ull << 32);

			MemoryReturn ret;
			if(_tm_buffer_table[bank_index] == ~0u)		// allocate a ray buffer to the tm
			{
				if(_idle_ray_buffer.empty()) continue;
				ret = allocate_ray_buffer(bank_index, req);
			}
			else
			{
				uint32_t treelet_id = _tm_buffer_table[bank_index];
				if(_ray_buffers[treelet_id].size() == 0)	// ray buffer is empty, allocate a new ray buffer to the tm
				{
					_tm_buffer_table[bank_index] = ~0u;
					if(_idle_ray_buffer.empty()) continue;
					ret = allocate_ray_buffer(bank_index, req);
				}
				else
				{
					std::memcpy(ret.data, &_ray_buffers[treelet_id].rays.back(), req.size);
					_ray_buffers[treelet_id].pop_ray();
				}
			}

			log.loads++;
			log.bytes_read += req.size;
			_return_network.write(ret, bank_index);
			bank.data_pipline.read();
		}
		else if(req.type == MemoryRequest::Type::STORE)
		{
			log.stores++;
			log.bytes_written += req.size;
			
			RayData ray_data;
			std::memcpy(&ray_data, req.data, req.size);
			if(_ray_buffers.count(ray_data.raystate.treelet_id) == 0)
			{
				RayBuffer ray_buffer;
				ray_buffer.header.treelet_id = ray_data.raystate.treelet_id;
				ray_buffer.write_ray(ray_data);
				_ray_buffers[ray_data.raystate.treelet_id] = ray_buffer;
				_ray_buffers_size += sizeof(RayBuffer::Header) + sizeof(RayData);
				_idle_ray_buffer.insert(ray_data.raystate.treelet_id);
			}
			else
			{
				_ray_buffers[ray_data.raystate.treelet_id].write_ray(ray_data);
				_ray_buffers_size += sizeof(RayData);
			}

			bank.data_pipline.read();
		}
	}

	// issue_returns();
	_return_network.clock();
}

void UnitRayStreamBuffer::_proccess_request(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	if(!_request_network.is_read_valid(bank_index) || !bank.data_pipline.is_write_valid() || !(_ray_buffers_size < _max_size))
		return;

	bank.data_pipline.write(_request_network.read(bank_index));
}

MemoryReturn UnitRayStreamBuffer::allocate_ray_buffer(uint bank_index, const MemoryRequest& req)
{
	MemoryReturn ret;
	uint32_t treelet_id = *_idle_ray_buffer.begin();
	_tm_buffer_table[bank_index] = treelet_id;
	_assert(_ray_buffers[treelet_id].size() > 0);
	std::memcpy(ret.data, &_ray_buffers[treelet_id].rays.back(), req.size);
	_ray_buffers[treelet_id].pop_ray();
	_idle_ray_buffer.erase(treelet_id);
	return ret;
}

}}}