#include "unit-treelet-rt-core.hpp"

namespace Arches { namespace Units { namespace STRaTA {

UnitTreeletRTCore::UnitTreeletRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _tm_index(config.tm_index), _treelet_base_addr(config.treelet_base_addr), _hit_record_base_addr(config.hit_record_base_addr),
	_cache(config.cache), _ray_stream_buffer(config.ray_stream_buffer), _request_network(config.num_tp, 1), _return_network(1, config.num_tp),
	_box_pipline(3, 1), _tri_pipline(22, 8)
{
	_tri_staging_buffers.resize(config.max_rays);
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_ray_data_load_queue.push(i);
	}
}

bool UnitTreeletRTCore::_try_queue_node(uint ray_id, uint treelet_id, uint node_id)
{
	paddr_t start = (paddr_t)&((rtm::WideTreeletSTRaTABVH::Treelet*)_treelet_base_addr)[treelet_id].nodes[node_id];
	_assert(start < 4ull * 1204 * 1024 * 1024);
	_fetch_queue.push({start, (uint8_t)(sizeof(rtm::WideTreeletSTRaTABVH::Treelet::Node)), (uint16_t)ray_id});
	return true;
}

bool UnitTreeletRTCore::_try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset, uint num_tris)
{
	paddr_t start = (paddr_t) & ((rtm::WideTreeletSTRaTABVH::Treelet*)_treelet_base_addr)[treelet_id].data[tri_offset];
	paddr_t end = start + sizeof(rtm::WideTreeletSTRaTABVH::Treelet::Triangle) * num_tris;

	_assert(start < 4ull * 1204 * 1024 * 1024);

	_tri_staging_buffers[ray_id].addr = start;
	_tri_staging_buffers[ray_id].num_tris = num_tris;
	_tri_staging_buffers[ray_id].bytes_filled = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		if(addr >= (paddr_t)(&((rtm::WideTreeletSTRaTABVH::Treelet*)_treelet_base_addr)[treelet_id + 1])) __debugbreak();

		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_fetch_queue.push({addr, size, (uint16_t)(ray_id | 0x8000u)});
		addr += size;
	}

	return true;
}

void UnitTreeletRTCore::_read_requests()
{
	if(_request_network.is_read_valid(0))
	{
		MemoryRequest request = _request_network.read(0);
		if (request.size == sizeof(RayData))			// forward ray buffer write
		{
			RayData ray_data;
			std::memcpy(&ray_data, request.data, sizeof(RayData));
			_ray_buffer_store_queue.push(ray_data);
			log.rays++;
		}
		else if(request.size == sizeof(rtm::Hit))		// load hit
		{
			_tp_hit_load_queue.push(request);
		}
		else _assert(false);
	}
}

void UnitTreeletRTCore::_read_returns()
{
	if(_ray_stream_buffer->return_port_read_valid(_tm_index))
	{
		MemoryReturn ret = _ray_stream_buffer->read_return(_tm_index);
		_assert(ret.size == sizeof(RayData));
		RayData ray_data;
		std::memcpy(&ray_data, ret.data, sizeof(RayData));
		uint ray_id = ret.dst;
		RayState& ray_state = _ray_states[ray_id];
		ray_state.ray_data = ray_data;
		ray_state.inv_d = rtm::vec3(1.0f) / ray_data.ray.d;
		ray_state.hit.t = ray_data.raystate.hit_t;
		ray_state.hit.bc = rtm::vec2(0.0f);
		ray_state.hit.id = ray_data.raystate.hit_id;
		ray_state.stack_size = 1;
		ray_state.stack.t = ray_data.ray.t_min;
		ray_state.stack.treelet = ray_data.raystate.treelet_id;
		ray_state.stack.child_index = ray_data.raystate.treelet_child_id;
		ray_state.stack.data.is_int = 1;
		ray_state.stack.data.is_child_treelet = 0;
		ray_state.stack.data.is_parent_treelet = 0;

		ray_state.phase = RayState::Phase::SCHEDULER;
		_ray_scheduling_queue.push(ray_id);

		log.rays++;
	}

	if(_cache->return_port_read_valid(_num_tp))
	{
		const MemoryReturn ret = _cache->read_return(_num_tp);
		uint16_t ray_id = ret.dst & ~0x8000u;
		if(ret.dst & 0x8000)
		{
			TriStagingBuffer& buffer = _tri_staging_buffers[ray_id];

			uint offset = (ret.paddr - buffer.addr);
			std::memcpy((uint8_t*)&buffer.tri + offset, ret.data, ret.size);

			buffer.bytes_filled += ret.size;
			if(buffer.bytes_filled == sizeof(rtm::WideTreeletSTRaTABVH::Treelet::Triangle))
			{
				_ray_states[ray_id].phase = RayState::Phase::TRI_ISECT;
				_tri_isect_queue.push(ray_id);
			}
		}
		else
		{
			_assert(sizeof(rtm::WideTreeletSTRaTABVH::Treelet::Node) == ret.size);

			NodeStagingBuffer buffer;
			buffer.ray_id = ray_id;
			std::memcpy(&buffer.node, ret.data, ret.size);

			_ray_states[ray_id].phase = RayState::Phase::NODE_ISECT;
			_node_isect_queue.push(buffer);
		}
	}
}

void UnitTreeletRTCore::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		if(ENABLE_RT_DEBUG_PRINTS)
			printf("Scheduling: %d\n", ray_id);

		RayState& ray_state = _ray_states[ray_id];

		if(ray_state.ray_data.traversal_stack > 1)
		{
			RayState::StackEntry& entry = ray_state.stack;
			if(entry.t < ray_state.hit.t) //pop cull
			{
				if(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::DOWN)
				{
					if(entry.data.is_int)
					{
						if(!entry.data.is_child_treelet)
						{
							if(_try_queue_node(ray_id, entry.treelet, entry.child_index))
							{
								ray_state.ray_data.raystate.treelet_id = entry.treelet;
								ray_state.phase = RayState::Phase::NODE_FETCH;

								if(ENABLE_RT_DEBUG_PRINTS)
									printf("Node: %d\n", entry.data.child_index);
							}
							else
							{
								_ray_scheduling_queue.push(ray_id);
							}
						}
						else
						{
							ray_state.ray_data.raystate.treelet_id = entry.data.child_index;
							ray_state.ray_data.raystate.treelet_child_id = 0;
							RayData ray_data;
							ray_data = ray_state.ray_data;
							ray_data.ray.t_max = rtm::min(ray_state.ray_data.ray.t_max, ray_state.hit.t);
							_ray_buffer_store_queue.push(ray_data);
							ray_state.ray_data.traversal_stack = 0;
							_ray_scheduling_queue.push(ray_id);
						}
					}
					else
					{
						if(_try_queue_tri(ray_id, ray_state.ray_data.raystate.treelet_id, entry.data.triangle_index * 4, entry.data.num_tri))
						{
							ray_state.ray_data.raystate.treelet_id = entry.treelet;
							ray_state.phase = RayState::Phase::TRI_FETCH;

							if(ENABLE_RT_DEBUG_PRINTS)
								printf("Tri: %d:%d\n", entry.data.triangle_index * 4, entry.data.num_tri + 1);
						}
						else
						{
							_ray_scheduling_queue.push(ray_id);
						}
					}
				}
				else if(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::UP)	// fetch parent node
				{
					if(entry.data.is_int)
					{
						if(!entry.data.is_parent_treelet)
						{
							if(_try_queue_node(ray_id, entry.treelet, entry.data.parent_child_index))
							{
								ray_state.ray_data.raystate.treelet_id = entry.treelet;
								ray_state.phase = RayState::Phase::NODE_FETCH;

								if(ENABLE_RT_DEBUG_PRINTS)
									printf("Node: %d\n", entry.data.child_index);
							}
							else
							{
								_ray_scheduling_queue.push(ray_id);
							}
						}
						else
						{
							ray_state.ray_data.raystate.treelet_id = entry.data.parent_index;
							ray_state.ray_data.raystate.treelet_child_id = entry.data.parent_child_index;
							RayData ray_data;
							ray_data = ray_state.ray_data;
							ray_data.ray.t_max = rtm::min(ray_state.ray_data.ray.t_max, ray_state.hit.t);
							_ray_buffer_store_queue.push(ray_data);
							ray_state.ray_data.traversal_stack = 0;
							_ray_scheduling_queue.push(ray_id);
						}
					}
					else
					{
						if(_try_queue_node(ray_id, entry.treelet, entry.data.leaf_parent_child_index))
						{
							ray_state.ray_data.raystate.treelet_id = entry.treelet;
							ray_state.phase = RayState::Phase::NODE_FETCH;

							if(ENABLE_RT_DEBUG_PRINTS)
								printf("Node: %d\n", entry.data.child_index);
						}
						else
						{
							_ray_scheduling_queue.push(ray_id);
						}
					}
				}
			}
			else
			{
				_ray_scheduling_queue.push(ray_id);
			}
		}
		else
		{
			if(ray_state.hit.id != ~0u)
			{
				_assert(ray_state.ray_data.traversal_stack == 1);
				//stack empty or anyhit found return the hit
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("Ret: %d\n", ray_state.hit.id);

				ray_state.phase = RayState::Phase::HIT_UPDATE;
				// _ray_return_queue.push(ray_id);
				paddr_t paddr = _hit_record_base_addr + ray_state.ray_data.raystate.id * sizeof(rtm::Hit);
				_hit_return_map[paddr] = ray_id;
			}
			else
			{
				ray_state.phase = RayState::Phase::RAY_FETCH;
				_ray_data_load_queue.push(ray_id);
			}
		}
	}
}

void UnitTreeletRTCore::_simualte_intersectors()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		NodeStagingBuffer& buffer = _node_isect_queue.front();

		uint ray_id = buffer.ray_id;
		RayState& ray_state = _ray_states[ray_id];

		rtm::Ray& ray = ray_state.ray_data.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;
		rtm::WideTreeletSTRaTABVH::Treelet::Node& node = buffer.node;

		if(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::DOWN)
		{
			float hit_ts[2] = {rtm::intersect(node.aabb[0], ray, inv_d), rtm::intersect(node.aabb[1], ray, inv_d)};
			uint32_t near_side = (hit_ts[0] < hit_ts[1]) ? 0 : 1;

			if(hit_ts[near_side] < hit.t)
			{
				ray_state.stack.t = hit_ts[near_side];
				ray_state.stack.treelet = ray_state.ray_data.raystate.treelet_id;
				ray_state.stack.child_index = node.data[near_side].child_index;
				ray_state.stack.data = node.data[near_side];
				ray_state.ray_data.traversal_stack = (ray_state.ray_data.traversal_stack << 1) | near_side;
				if((hit_ts[0] < hit.t) && (hit_ts[1] < hit.t))
					ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack << 1;	// near node first
				else
					ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack << 0;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::DOWN;
			}
			else
			{
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::UP;
			}
		}
		else if(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::UP)
		{
			uint32_t near_visited = ray_state.ray_data.visited_stack & 1;
			if(near_visited)
			{
				uint32_t next_visit = !(ray_state.ray_data.traversal_stack & 1);
				ray_state.stack.t = rtm::intersect(node.aabb[next_visit], ray, inv_d);
				ray_state.stack.treelet = ray_state.ray_data.raystate.treelet_id;
				ray_state.stack.child_index = node.data[next_visit].child_index;
				ray_state.stack.data = node.data[next_visit];
				ray_state.ray_data.traversal_stack = ((ray_state.ray_data.traversal_stack >> 1) << 1) | next_visit;
				ray_state.ray_data.visited_stack = (ray_state.ray_data.visited_stack >> 1) << 0;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::DOWN;
			}
			else
			{
				ray_state.ray_data.traversal_stack = ray_state.ray_data.traversal_stack >> 1;
				ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack >> 1;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::UP;
			}
		}

		_box_pipline.write(ray_id);
		_node_isect_queue.pop();
	}

	_box_pipline.clock();

	if(_box_pipline.is_read_valid())
	{
		uint ray_id = _box_pipline.read();
		if(ray_id != ~0u)
		{
			_ray_states[ray_id].phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
		}

		log.nodes += 1;
	}

	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		TriStagingBuffer& buffer = _tri_staging_buffers[ray_id];

		RayState& ray_state = _ray_states[ray_id];

		rtm::Ray& ray = ray_state.ray_data.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;

		if(rtm::intersect(buffer.tri.tri, ray, hit))
		{
			hit.id = buffer.tri.id;
			ray_state.ray_data.raystate.hit_id = buffer.tri.id;
			ray_state.ray_data.raystate.hit_t = hit.t;
		}
		_assert(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::DOWN);
		ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::UP;

		_tri_pipline.write(ray_id);
		_tri_isect_queue.pop();
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

void UnitTreeletRTCore::_issue_requests()
{
	if(!_fetch_queue.empty() && _cache->request_port_write_valid(_num_tp))
	{
		//fetch the next block
		MemoryRequest request;
		request.type = MemoryRequest::Type::LOAD;
		request.size = _fetch_queue.front().size;
		request.dst = _fetch_queue.front().dst;
		request.paddr = _fetch_queue.front().addr;
		request.port = _num_tp;
		_cache->write_request(request);
		_fetch_queue.pop();
	}

	if(_ray_stream_buffer->request_port_write_valid(_tm_index))
	{
		if(!_ray_buffer_store_queue.empty())
		{
			RayData ray_data = _ray_buffer_store_queue.front();
			_ray_buffer_store_queue.pop();

			MemoryRequest req;
			req.type = MemoryRequest::Type::STORE;
			req.size = sizeof(RayData);
			req.port = _tm_index;
			req.paddr = 0xdeadbeefull;
			std::memcpy(req.data, &ray_data, req.size);
			_ray_stream_buffer->write_request(req);
		}
		else if(!_ray_data_load_queue.empty())
		{
			uint ray_id = _ray_data_load_queue.front();
			_ray_data_load_queue.pop();

			MemoryRequest req;
			req.type = MemoryRequest::Type::LOAD;
			req.size = sizeof(RayData);
			req.port = _tm_index;
			req.dst = ray_id;
			// req.flags = _ray_states[ray_id].current_treelet_id;
			req.paddr = 0xdeadbeefull;
			_ray_stream_buffer->write_request(req);
		}
	}
}

void UnitTreeletRTCore::_issue_returns()
{
	if(!_tp_hit_load_queue.empty() && _return_network.is_write_valid(0))
	{
		MemoryRequest req = _tp_hit_load_queue.front();
		if(_hit_return_map.find(req.paddr) != _hit_return_map.end())
		{
			uint ray_id = _hit_return_map[req.paddr];
			RayState& ray_state = _ray_states[ray_id];
			MemoryReturn ret;
			ret.size = sizeof(rtm::Hit);
			ret.dst = req.dst;
			ret.port = req.port;
			ret.paddr = req.paddr;
			std::memcpy(ret.data, &ray_state.hit, sizeof(rtm::Hit));
			_return_network.write(ret, 0);

			ray_state.phase = RayState::Phase::RAY_FETCH;
			_ray_data_load_queue.push(ray_id);
			_hit_return_map.erase(req.paddr);
			_tp_hit_load_queue.pop();
		}
	}
}


}}}