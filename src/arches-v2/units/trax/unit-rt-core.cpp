#include "unit-rt-core.hpp"

namespace Arches { namespace Units { namespace TRaX {

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 4)
//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 4 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

template<typename NT>
UnitRTCore<NT>::UnitRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _node_base_addr(config.node_base_addr), _tri_base_addr(config.tri_base_addr),
	_cache(config.cache), _request_network(config.num_clients, 1), _return_network(1, config.num_clients),
	_box_pipline(3), _tri_pipline(22), _cache_port(config.cache_port)
{
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_free_ray_ids.insert(i);
	}
}
template<typename NT>
void UnitRTCore<NT>::clock_rise()
{
	_request_network.clock();
	_read_requests();
	_read_returns();

	for(uint i = 0; i < 1; ++i) //n stack ops per cycle. In reality this would need to be multi banked
		_schedule_ray();

	_simualte_node_pipline();
	_simualte_tri_pipline();
}

template<typename NT>
void UnitRTCore<NT>::clock_fall()
{
	_issue_requests();
	_issue_returns();
	_return_network.clock();
}

template<typename NT>
bool UnitRTCore<NT>::_try_queue_node(uint ray_id, uint node_id)
{
	paddr_t start = _node_base_addr + node_id * sizeof(NT::Node);
	paddr_t end = start + sizeof(NT::Node);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_fetch_queue.push({addr, size, (uint16_t)(ray_id)});
		addr += size;
	}

	return true;
}

template<typename NT>
bool UnitRTCore<NT>::_try_queue_tris(uint ray_id, uint tri_id, uint num_tris)
{
	paddr_t start = _tri_base_addr + tri_id * sizeof(rtm::Triangle);
	paddr_t end = start + sizeof(rtm::Triangle) * num_tris;

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 1;
	ray_state.buffer.tri_id = tri_id;
	ray_state.buffer.num_tris = num_tris;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_fetch_queue.push({addr, size, (uint16_t)(ray_id)});
		addr += size;
	}

	return true;
}

template<typename NT>
void UnitRTCore<NT>::_read_requests()
{
	if(_request_network.is_read_valid(0) && !_free_ray_ids.empty())
	{
		//creates a ray entry and queue up the ray
		const MemoryRequest request = _request_network.read(0);

		uint ray_id = *_free_ray_ids.begin();
		_free_ray_ids.erase(ray_id);

		RayState& ray_state = _ray_states[ray_id];
		std::memcpy(&ray_state.ray, request.data, sizeof(rtm::Ray));
		ray_state.inv_d = rtm::vec3(1.0f) / ray_state.ray.d;
		ray_state.hit.t = ray_state.ray.t_max;
		ray_state.hit.bc = rtm::vec2(0.0f);
		ray_state.hit.id = ~0u;
		ray_state.stack_size = 1;
		ray_state.stack[0].t = ray_state.ray.t_min;
		ray_state.stack[0].data.is_int = 1;
		ray_state.stack[0].data.child_index = 0;
		ray_state.current_entry = 0;
		ray_state.flags = request.flags;
		ray_state.dst = request.dst;
		ray_state.dst.push(request.port, 8);

		ray_state.phase = RayState::Phase::SCHEDULER;
		_ray_scheduling_queue.push(ray_id);

		log.rays++;
	}
}

template<typename NT>
void UnitRTCore<NT>::_read_returns()
{
	if(_cache->return_port_read_valid(_cache_port))
	{
		const MemoryReturn ret = _cache->read_return(_cache_port);
		uint16_t ray_id = ret.dst.peek(10);
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		//if(ENABLE_RT_DEBUG_PRINTS) printf("ret %xll\n", ret.paddr);

		uint offset = (ret.paddr - buffer.address);
		std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
		buffer.bytes_filled += ret.size;

		if(buffer.type == 0)
		{
			if(buffer.bytes_filled == sizeof(NT::Node))
			{
				ray_state.phase = RayState::Phase::NODE_ISECT;
				_node_isect_queue.push(ray_id);
			}
		}
		else if(buffer.type == 1)
		{
			if(buffer.bytes_filled == sizeof(rtm::Triangle) * buffer.num_tris)
			{
				ray_state.phase = RayState::Phase::TRI_ISECT;
				_tri_isect_queue.push(ray_id);
			}
		}
	}
}

template<typename NT>
void UnitRTCore<NT>::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		RayState& ray_state = _ray_states[ray_id];
		//bool any_hit_found = (ray_state.flags & 0x1) && ray_state.hit.id != ~0u;
		if(ray_state.stack_size > 0)
		{
			StackEntry& entry = ray_state.stack[ray_state.stack_size - 1];
			if(entry.t < ray_state.hit.t) //pop cull
			{
				if(entry.data.is_int)
				{
					_try_queue_node(ray_id, entry.data.child_index);
					ray_state.phase = RayState::Phase::NODE_FETCH;
					ray_state.stack_size--;

					log.issue_counters[(uint)IssueType::NODE_FETCH]++;
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("%03d NODE_FETCH: %d\n", ray_id, entry.data.child_index);
				}
				else
				{
					_try_queue_tris(ray_id, entry.data.prim_index, entry.data.num_prims);
					ray_state.phase = RayState::Phase::TRI_FETCH;
					ray_state.stack_size--;

					log.issue_counters[(uint)IssueType::TRI_FETCH]++;
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("%03d TRI_FETCH: %d:%d\n", ray_id, entry.data.prim_index, entry.data.num_prims);
				}
			}
			else
			{
				ray_state.stack_size--;
				_ray_scheduling_queue.push(ray_id);

				log.issue_counters[(uint)IssueType::POP_CULL]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d POP_CULL\n", ray_id);
			}
		}
		else
		{
			//stack empty or anyhit found return the hit
			ray_state.phase = RayState::Phase::HIT_RETURN;
			_ray_return_queue.push(ray_id);

			log.issue_counters[(uint)IssueType::HIT_RETURN]++;
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("%03d HIT_RETURN: %d\n", ray_id, ray_state.hit.id);
		}
	}
	else
	{
		uint phase = (uint)_ray_states[_last_ray_id].phase;
		if(++_last_ray_id == _ray_states.size()) _last_ray_id = 0;
		log.stall_counters[phase]++;
	}
}

static rtm::WideBVH::Node decompress(const rtm::WideBVH::Node& node)
{
	return node;
}

static rtm::WideBVH::Node decompress(const rtm::CompressedWideBVH::Node& node)
{
	return node.decompress();
}


template<typename NT>
void UnitRTCore<NT>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		const rtm::WideBVH::Node& node = decompress(ray_state.buffer.node);

		_box_issue_count += 6;
		if(_box_issue_count >= node.num_aabb())
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;

			uint max_insert_depth = ray_state.stack_size;
			for(uint i = 0; i < rtm::WideBVH::WIDTH; ++i)
			{
				if(!node.is_valid(i)) continue;

				float t = rtm::intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = ray_state.stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(ray_state.stack[j - 1].t > t) break;
						ray_state.stack[j] = ray_state.stack[j - 1];
					}
					ray_state.stack[j].t = t;
					ray_state.stack[j].data = node.data[i];
				}
			}

			_box_pipline.write(ray_id);
			_node_isect_queue.pop();
			_box_issue_count = 0;
		}
		else
		{
			_box_pipline.write(~0);
		}
	}

	_box_pipline.clock();

	if(_box_pipline.is_read_valid())
	{
		uint ray_id = _box_pipline.read();
		if(ray_id != ~0u)
		{
			_ray_states[ray_id].phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
			log.nodes++;
		}
	}
}

template<typename NT>
void UnitRTCore<NT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		_tri_issue_count += 1;
		if(_tri_issue_count >= buffer.num_tris)
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;

			for(uint i = 0; i < buffer.num_tris; ++i)
				if(rtm::intersect(buffer.tris[i], ray, hit))
					hit.id = buffer.tri_id + i;

			_tri_pipline.write(ray_id);
			_tri_isect_queue.pop();
			_tri_issue_count = 0;
		}
		else
		{
			_tri_pipline.write(~0u);
		}
	}

	_tri_pipline.clock();

	if(_tri_pipline.is_read_valid())
	{
		uint ray_id = _tri_pipline.read();
		if(ray_id != ~0u)
		{
			_ray_states[ray_id].phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
		}

		log.tris++;
	}
}

template<typename NT>
void UnitRTCore<NT>::_issue_requests()
{
	if(!_fetch_queue.empty() && _cache->request_port_write_valid(_cache_port))
	{
		//if(ENABLE_RT_DEBUG_PRINTS) printf("req %llx\n", _fetch_queue.front().addr);

		//fetch the next block
		MemoryRequest request;
		request.type = MemoryRequest::Type::LOAD;
		request.size = _fetch_queue.front().size;
		request.dst.push(_fetch_queue.front().ray_id, 10);
		request.paddr = _fetch_queue.front().addr;
		request.port = _cache_port;
		_cache->write_request(request);
		_fetch_queue.pop();
	}
}

template<typename NT>
void UnitRTCore<NT>::_issue_returns()
{
	if(!_ray_return_queue.empty())
	{
		uint ray_id = _ray_return_queue.front();
		RayState& ray_state = _ray_states[ray_id];

		if(_return_network.is_write_valid(0))
		{
			//fetch the next block
			MemoryReturn ret;
			ret.size = sizeof(rtm::Hit);
			ret.port = ray_state.dst.pop(8);
			ret.dst = ray_state.dst;
			ret.paddr = 0xdeadbeefull;
			std::memcpy(ret.data, &ray_state.hit, sizeof(rtm::Hit));
			_return_network.write(ret, 0);

			ray_state.phase = RayState::Phase::RAY_FETCH;
			_free_ray_ids.insert(ray_id);
			_ray_return_queue.pop();
			log.hits_returned++;
		}
	}
}

template class UnitRTCore<rtm::WideBVH>;
template class UnitRTCore<rtm::CompressedWideBVH>;

}}}