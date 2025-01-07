#include "unit-prt-core.hpp"

namespace Arches {
namespace Units {
namespace TRaX {



template<typename NT>
UnitPRTCore<NT>::UnitPRTCore<NT>(const Configuration& config) :
	_max_rays(config.max_rays), _cache_port(config.cache_port), _num_clients(config.num_clients),
	_node_base_addr(config.node_base_addr), _tri_base_addr(config.tri_base_addr),
	_cache(config.cache), _request_network(config.num_clients, 1), _return_network(1, config.num_clients),
	_box_pipline(3), _tri_pipline(22)
{
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_ray_states[i].num_rays = 0;
		_ray_states[i].return_ray = 0;
		_ray_states[i].tile_id = 0;
		_free_ray_ids.insert(i);
	}

	_box_issues = 0;
	_tri_issues = 0;
}

template<typename NT>
void UnitPRTCore<NT>::clock_rise()
{
	_request_network.clock();
	_read_requests();
	_read_returns();

	if(_ray_scheduling_queue.empty())
	{
			log.stall_counters[(uint)_ray_states[last_ray_id].phase]++;
			if(++last_ray_id == _ray_states.size()) last_ray_id = 0;
	}

	for(uint i = 0; i < 1; ++i) //n stack ops per cycle. In reality this would need to be multi banked
		_schedule_ray();

	_simualte_node_pipline();
	_simualte_tri_pipline();
}

template<typename NT>
void UnitPRTCore<NT>::clock_fall()
{
	_issue_requests();
	_issue_returns();
	_return_network.clock();
}

template<typename NT>
bool UnitPRTCore<NT>::_try_queue_node(uint ray_id, uint node_id)
{
	paddr_t start = _node_base_addr + node_id * sizeof(NT::Node);
	paddr_t end = start + sizeof(NT::Node);

	_assert(start < 0x1ull << 32);

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
bool UnitPRTCore<NT>::_try_queue_tri(uint ray_id, uint tri_id, uint num_tris)
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
void UnitPRTCore<NT>::_read_requests()
{
	if(_request_network.is_read_valid(0))
	{
		//creates a ray entry and queue up the ray
		const MemoryRequest& request = _request_network.peek(0);

		rtm::Ray ray;
		std::memcpy(&ray, request.data, sizeof(rtm::Ray));
		uint packet_id = (*(uint*)&ray.t_min) & 0xffff;

		uint ray_id = ~0u;
		for(uint i = 0; i < _ray_states.size(); ++i)
		{
			if(_free_ray_ids.count(i) > 0) continue;
			if(_ray_states[i].num_rays >= PACKET_SIZE) continue;

			if(_ray_states[ray_id].tile_id == packet_id);
			{
				ray_id = i;
				break;
			}
		}

		if(ray_id == ~0u && !_free_ray_ids.empty())
		{
			ray_id = *_free_ray_ids.begin();
			_ray_states[ray_id].tile_id = packet_id;
			_assert(_ray_states[ray_id].num_rays == 0);
			_free_ray_ids.erase(ray_id);
		}

		if(ray_id == ~0u) return;

		RayState& ray_state = _ray_states[ray_id];
		_assert(ray_state.phase == RayState::Phase::RAY_FETCH);
		uint ray_index = ray_state.num_rays++;
		ray_state.ray[ray_index] = ray;
		ray_state.inv_d[ray_index] = rtm::vec3(1.0f) / ray_state.ray[ray_index].d;
		ray_state.hit[ray_index].t = ray_state.ray[ray_index].t_max;
		ray_state.hit[ray_index].bc = rtm::vec2(0.0f);
		ray_state.hit[ray_index].id = ~0u;
		ray_state.stack_size = 1;
		ray_state.stack[0].min_t = ray_state.ray[ray_index].t_min;
		ray_state.stack[0].data.is_int = 1;
		ray_state.stack[0].data.child_index = 0;
		ray_state.stack[0].mask = generate_nbit_mask(PACKET_SIZE);
		ray_state.max_t = T_MAX;
		ray_state.current_entry = 0;
		ray_state.dst[ray_index] = request.dst;
		ray_state.dst[ray_index].push(request.port, 8);

		if(ray_state.num_rays == PACKET_SIZE)
		{
			ray_state.phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
		}

		_request_network.read(0);

		log.rays++;
	}
}

template<typename NT>
void UnitPRTCore<NT>::_read_returns()
{
	if(_cache->return_port_read_valid(_cache_port))
	{
		const MemoryReturn ret = _cache->read_return(_cache_port);
		uint16_t ray_id = ret.dst.peek(10);
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

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
void UnitPRTCore<NT>::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		RayState& ray_state = _ray_states[ray_id];
		if(ray_state.stack_size > 0)
		{
			StackEntry& entry = ray_state.stack[ray_state.stack_size - 1];
			if(entry.min_t >= ray_state.max_t) //pop cull
			{
				if(entry.data.is_int)
				{
					if(_try_queue_node(ray_id, entry.data.child_index))
					{
						ray_state.phase = RayState::Phase::NODE_FETCH;
						ray_state.mask = entry.mask;

						log.issue_counters[(uint)IssueType::NODE_FETCH]++;
						if(ENABLE_RT_DEBUG_PRINTS)
							printf("%03d NODE_FETCH: %d\n", ray_id, entry.data.child_index);

						ray_state.stack_size--;
					}
					else
					{
						_ray_scheduling_queue.push(ray_id);
					}
				}
				else
				{
					if(_try_queue_tri(ray_id, entry.data.prim_index, entry.data.num_prims))
					{
						ray_state.phase = RayState::Phase::TRI_FETCH;
						ray_state.mask = entry.mask;
						ray_state.stack_size--;

						log.issue_counters[(uint)IssueType::TRI_FETCH]++;
						if(ENABLE_RT_DEBUG_PRINTS)
							printf("%03d TRI_FETCH: %d:%d\n", ray_id, entry.data.prim_index, entry.data.num_prims);
					}
					else
					{
						_ray_scheduling_queue.push(ray_id);
					}
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
			log.issue_counters[(uint)IssueType::HIT_RETURN]++;
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("Ret: %d\n", ray_state.hit[0].id);

			ray_state.phase = RayState::Phase::HIT_RETURN;
			_ray_return_queue.push(ray_id);
		}
	}
}

template<>
void UnitPRTCore<rtm::CompressedWideBVH>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		const rtm::WideBVH::Node node = ray_state.buffer.node.decompress();

		_box_issues++;
		if(_box_issues < popcnt(ray_state.mask) * node.num_aabb() / 3)
		{
			_box_pipline.write(~0u);
		}
		else
		{
			uint max_insert_depth = ray_state.stack_size;
			for(uint i = 0; i < rtm::CompressedWideBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float min_t = T_MAX;
				uint64_t mask = 0x0;
				for(uint j = 0; j < PACKET_SIZE; ++j)
				{
					if(((ray_state.mask >> j) & 0x1) == 0) continue;

					rtm::Ray& ray = ray_state.ray[j];
					rtm::vec3& inv_d = ray_state.inv_d[j];
					rtm::Hit& hit = ray_state.hit[j];

					float t = rtm::intersect(node.aabb[i], ray, inv_d);
					if(t < hit.t)
					{
						mask |= 0x1ull << j;
						min_t = rtm::min(min_t, t);
					}
				}

				if(mask)
				{
					uint j = ray_state.stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(ray_state.stack[j - 1].min_t > min_t) break;
						ray_state.stack[j] = ray_state.stack[j - 1];
					}

					ray_state.stack[j].min_t = min_t;
					ray_state.stack[j].data = node.data[i];
					ray_state.stack[j].mask = mask;
				}
			}

			_box_pipline.write(ray_id);
			_node_isect_queue.pop();
			_box_issues = 0;
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

template<>
void UnitPRTCore<rtm::WideBVH>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		uint max_insert_depth = ray_state.stack_size;

		_box_issues++;
		if(_box_issues < popcnt(ray_state.mask) * rtm::WideBVH::WIDTH)
		{
			_box_pipline.write(~0u);
		}
		else
		{
			for(uint i = 0; i < rtm::WideBVH::WIDTH; i++)
			{
				rtm::WideBVH::Node& node = ray_state.buffer.node;

				float min_t = T_MAX;
				uint64_t mask = 0x0;
				for(uint j = 0; j < PACKET_SIZE; ++j)
				{
					if(((ray_state.mask >> j) & 0x1) == 0) continue;

					rtm::Ray& ray = ray_state.ray[j];
					rtm::vec3& inv_d = ray_state.inv_d[j];
					rtm::Hit& hit = ray_state.hit[j];

					float t = rtm::intersect(node.aabb[i], ray, inv_d);
					if(t < hit.t)
					{
						mask |= 0x1ull << j;
						min_t = rtm::min(min_t, t);
					}
				}

				if(mask)
				{
					uint j = ray_state.stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(ray_state.stack[j - 1].min_t > min_t) break;
						ray_state.stack[j] = ray_state.stack[j - 1];
					}

					ray_state.stack[j].min_t = min_t;
					ray_state.stack[j].data = node.data[i];
					ray_state.stack[j].mask = mask;
				}
			}

			_box_pipline.write(ray_id);
			_node_isect_queue.pop();
			_box_issues = 0;
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
void UnitPRTCore<NT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];

		_tri_issues++;
		if(_tri_issues < popcnt(ray_state.mask) * ray_state.buffer.num_tris * 2)
		{
			_tri_pipline.write(~0u);
		}
		else
		{
			for(uint i = 0; i < PACKET_SIZE; ++i)
			{
				if(((ray_state.mask >> i) & 0x1) == 0x0) continue;

				rtm::Ray& ray = ray_state.ray[i];
				rtm::vec3& inv_d = ray_state.inv_d[i];
				rtm::Hit& hit = ray_state.hit[i];
				StagingBuffer& buffer = ray_state.buffer;

				for(uint j = 0; j < buffer.num_tris; ++j)
					if(rtm::intersect(buffer.tris[j], ray, hit))
						hit.id = buffer.tri_id;
			}

			ray_state.max_t = ray_state.hit[0].t;
			for(uint i = 1; i < PACKET_SIZE; ++i)
				ray_state.max_t = rtm::max(ray_state.hit[i].t, ray_state.max_t);

			_tri_pipline.write(ray_id);
			_tri_isect_queue.pop();
			_box_issues = 0;
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
			log.tris++;
		}
	}
}

template<typename NT>
void UnitPRTCore<NT>::_issue_requests()
{
	if(!_fetch_queue.empty() && _cache->request_port_write_valid(_cache_port))
	{
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
void UnitPRTCore<NT>::_issue_returns()
{
	if(!_ray_return_queue.empty())
	{
		uint ray_id = _ray_return_queue.front();
		RayState& ray_state = _ray_states[ray_id];

		if(_return_network.is_write_valid(0))
		{
			uint ray_index = ray_state.return_ray++;

			//fetch the next block
			MemoryReturn ret;
			ret.size = sizeof(rtm::Hit);
			ret.port = ray_state.dst[ray_index].pop(8);
			ret.dst = ray_state.dst[ray_index];
			ret.paddr = 0xdeadbeefull;
			std::memcpy(ret.data, &ray_state.hit[ray_index], sizeof(rtm::Hit));
			_return_network.write(ret, 0);

			if(ray_state.return_ray == PACKET_SIZE)
			{
				ray_state.phase = RayState::Phase::RAY_FETCH;
				ray_state.num_rays = 0;
				ray_state.return_ray = 0;
				ray_state.tile_id = 0;
				_free_ray_ids.insert(ray_id);
				_ray_return_queue.pop();
			}
		}
	}
}

template class UnitPRTCore<rtm::WideBVH>;
template class UnitPRTCore<rtm::CompressedWideBVH>;

}
}
}