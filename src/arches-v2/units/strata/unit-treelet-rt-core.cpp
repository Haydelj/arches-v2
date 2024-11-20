#include "unit-treelet-rt-core.hpp"

namespace Arches { namespace Units { namespace STRaTA {

UnitTreeletRTCore::UnitTreeletRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _tm_index(config.tm_index), _treelet_base_addr(config.treelet_base_addr), _hit_record_base_addr(config.hit_record_base_addr),
	_cache(config.cache), _ray_stream_buffer(config.ray_stream_buffer), _request_network(config.num_tp, 1), _return_network(1, config.num_tp),
	_box_pipline(3, 1), _tri_pipline(22, 8)
{
	_tri_staging_buffers.resize(config.max_rays);
	_ray_states.resize(config.max_rays);
	_leaf_isect_buffers.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_ray_data_load_queue.push(i);
		_leaf_isect_buffers[i].ray_id = ~0u;
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
			log.store_rays++;
		}
		else if(request.size == sizeof(STRaTAHitReturn))		// load hit
		{
			_hit_return_port_map[request.paddr] = request.port;
			_tp_hit_load_queue.push(request);
			log.load_hits++;
		}
		else _assert(false);
	}
}

void UnitTreeletRTCore::_read_returns()
{
	if(_ray_stream_buffer->return_port_read_valid(_tm_index))
	{
		MemoryReturn ret = _ray_stream_buffer->read_return(_tm_index);
		if(ret.size == sizeof(RayData))
		{
			RayData ray_data;
			std::memcpy(&ray_data, ret.data, sizeof(RayData));
			uint ray_id = ret.dst;
			RayState& ray_state = _ray_states[ray_id];
			ray_state.ray_data = ray_data;
			ray_state.inv_d = rtm::vec3(1.0f) / ray_data.ray.d;
			ray_state.hit.t = ray_data.raystate.hit_t;
			ray_state.hit.bc = rtm::vec2(0.0f);
			ray_state.hit.id = ray_data.raystate.hit_id;
			ray_state.stack.t = ray_data.ray.t_min;
			ray_state.stack.treelet = ray_data.raystate.treelet_id;
			ray_state.stack.data.is_int = 1;
			ray_state.stack.data.is_child_treelet = 0;
			ray_state.stack.parent_data.is_parent_treelet = 0;
			ray_state.stack.parent_data.parent_treelet_index = ray_data.raystate.treelet_id;
			ray_state.stack.parent_data.parent_node_index = ray_data.raystate.treelet_child_id;
			ray_state.stack.data.child_index = ray_data.raystate.treelet_child_id;

			ray_state.phase = RayState::Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);

			log.rays++;
		}
		else if(ret.size == sizeof(STRaTAHitReturn))
		{
			ret.port = _hit_return_port_map[ret.paddr];
			_hit_return_port_map.erase(ret.port);
			_tp_hit_return_queue.push(ret);
		}
		else
			_assert(0);
	}

	if(_cache->return_port_read_valid(_num_tp))
	{
		const MemoryReturn ret = _cache->read_return(_num_tp);
		uint16_t ray_id = ret.dst & ~0x8000u;
		if(ret.dst & 0x8000)
		{
			TriStagingBuffer& buffer = _tri_staging_buffers[ray_id];

			uint offset = (ret.paddr - buffer.addr);
			std::memcpy((uint8_t*)&buffer.tris + offset, ret.data, ret.size);

			buffer.bytes_filled += ret.size;
			if(buffer.bytes_filled == sizeof(rtm::WideTreeletSTRaTABVH::Treelet::Triangle) * buffer.num_tris)
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

		RayState& ray_state = _ray_states[ray_id];

		if((ray_state.ray_data.traversal_stack > 1) || (ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::DOWN))
		{
			RayState::StackEntry& entry = ray_state.stack;
			if(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::DOWN)
			{
				if(entry.data.is_int)
				{
					if(!entry.data.is_child_treelet)
					{
						if(_try_queue_node(ray_id, entry.treelet, entry.data.child_index))
						{
							ray_state.ray_data.raystate.treelet_id = entry.treelet;
							ray_state.ray_data.raystate.treelet_child_id = entry.data.child_index;
							ray_state.phase = RayState::Phase::NODE_FETCH;

							if(ENABLE_RT_DEBUG_PRINTS)
								printf("TM: %d, Ray_id: %d, global_id: %d, treelet: %d, node: %d, [Down] Fetch Node: %d, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, ray_state.ray_data.raystate.treelet_id, ray_state.ray_data.raystate.treelet_child_id, entry.data.child_index, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());
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
						ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::DOWN;
						_ray_buffer_store_queue.push(ray_data);
						ray_state.phase = RayState::Phase::RAY_FETCH;
						_ray_data_load_queue.push(ray_id);
						if (ENABLE_RT_DEBUG_PRINTS)
							printf("TM: %d, Ray_id: %d, global_id: %d, treelet: %d, node: %d, [Down] Store Ray: %d, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, ray_state.ray_data.raystate.treelet_id, ray_state.ray_data.raystate.treelet_child_id, entry.data.child_index, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());
					}
				}
				else
				{
					if(_try_queue_tri(ray_id, entry.treelet, entry.data.triangle_index * 4, entry.data.num_tri))
					{
						ray_state.ray_data.raystate.treelet_id = entry.treelet;
						ray_state.phase = RayState::Phase::TRI_FETCH;

						if(ENABLE_RT_DEBUG_PRINTS)
							printf("TM: %d, Ray_id: %d, global_id: %d, treelet: %d, node: %d, [Down] Fetch Tri: %d:%d, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, ray_state.ray_data.raystate.treelet_id, ray_state.ray_data.raystate.treelet_child_id, entry.data.triangle_index * 4, entry.data.num_tri, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());
					}
					else
					{
						_ray_scheduling_queue.push(ray_id);
					}
				}
			}
			else if(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::UP)	// fetch parent node
			{
				if(!entry.parent_data.is_parent_treelet)
				{
					if(_try_queue_node(ray_id, entry.treelet, entry.parent_data.parent_node_index))
					{
						ray_state.ray_data.raystate.treelet_id = entry.treelet;
						ray_state.ray_data.raystate.treelet_child_id = entry.parent_data.parent_node_index;
						ray_state.phase = RayState::Phase::NODE_FETCH;

						if(ENABLE_RT_DEBUG_PRINTS)
							printf("TM: %d, Ray_id: %d, global_id: %d, treelet: %d, node: %d, [Up] Fetch Node: %d, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, ray_state.ray_data.raystate.treelet_id, ray_state.ray_data.raystate.treelet_child_id, entry.parent_data.parent_node_index, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());
					}
					else
					{
						_ray_scheduling_queue.push(ray_id);
					}
				}
				else
				{
					ray_state.ray_data.raystate.treelet_id = entry.parent_data.parent_treelet_index;
					ray_state.ray_data.raystate.treelet_child_id = entry.parent_data.parent_node_index;
					RayData ray_data;
					ray_data = ray_state.ray_data;
					ray_data.ray.t_max = rtm::min(ray_state.ray_data.ray.t_max, ray_state.hit.t);
					_ray_buffer_store_queue.push(ray_data);
					ray_state.phase = RayState::Phase::RAY_FETCH;
					_ray_data_load_queue.push(ray_id);
					if (ENABLE_RT_DEBUG_PRINTS)
						printf("TM: %d, Ray_id: %d, global_id: %d, [Up] Store Ray Treelet: %d, Node: %d, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, entry.parent_data.parent_treelet_index, entry.parent_data.parent_node_index, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());
				}
			}
		}
		else
		{
			_assert(ray_state.ray_data.traversal_stack == 1);
			if(ray_state.hit.id != ~0u)
			{
				//stack empty or anyhit found return the hit
				if(ENABLE_RT_DEBUG_PRINTS || ENABLE_HIT_DEBUG_PRINTS)
					printf("TM: %d, Ray_id: %d, global_id: %d, treelet: %d, node: %d, Store Hit: %d, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, ray_state.hit.id, ray_state.ray_data.raystate.treelet_id, ray_state.ray_data.raystate.treelet_child_id, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());

				ray_state.phase = RayState::Phase::HIT_UPDATE;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::OVER;
				_hit_store_queue.push(ray_id);
				log.get_hits++;
			}
			else
			{
				if (ENABLE_RT_DEBUG_PRINTS || ENABLE_HIT_DEBUG_PRINTS)
					printf("TM: %d, Ray_id: %d, global_id: %d, treelet: %d, node: %d, Hit Nothing, traversal stack: %s, visited stack: %s\n", _tm_index, ray_id, ray_state.ray_data.raystate.id, ray_state.ray_data.raystate.treelet_id, ray_state.ray_data.raystate.treelet_child_id, std::bitset<32>(ray_state.ray_data.traversal_stack).to_string().c_str(), std::bitset<32>(ray_state.ray_data.visited_stack).to_string().c_str());
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::OVER;
				_hit_store_queue.push(ray_id);
			}
			log.hits++;
		}
	}
}

void UnitTreeletRTCore::_simualte_intersectors()
{
	// box intersection
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		NodeStagingBuffer buffer = _node_isect_queue.front();

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
				ray_state.stack.data = node.data[near_side];
				ray_state.stack.parent_data = node.parent_data;
				_assert(ray_state.ray_data.traversal_stack >> 31 != 1);
				_assert(ray_state.ray_data.visited_stack >> 31 != 1);
				ray_state.ray_data.traversal_stack = (ray_state.ray_data.traversal_stack << 1) | near_side;
				if((hit_ts[0] < hit.t) && (hit_ts[1] < hit.t))
					ray_state.ray_data.visited_stack = (ray_state.ray_data.visited_stack << 1) | 1;	// near node first
				else
					ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack << 1;
				if(!ray_state.stack.data.is_int)	// stay
					_leaf_isect_buffers[ray_id] = buffer;
			}
			else
			{
				ray_state.stack.treelet = ray_state.ray_data.raystate.treelet_id;
				ray_state.stack.data = node.data[0];
				ray_state.stack.parent_data = node.parent_data;
				// ray_state.ray_data.traversal_stack = ray_state.ray_data.traversal_stack >> 1;
				// ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack >> 1;
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
				ray_state.stack.data = node.data[next_visit];
				ray_state.stack.parent_data = node.parent_data;
				_assert(ray_state.ray_data.traversal_stack >> 31 != 1);
				_assert(ray_state.ray_data.visited_stack >> 31 != 1);
				ray_state.ray_data.traversal_stack = ((ray_state.ray_data.traversal_stack >> 1) << 1) | next_visit;
				ray_state.ray_data.visited_stack = (ray_state.ray_data.visited_stack >> 1) << 1;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::DOWN;
				if(!ray_state.stack.data.is_int)	// stay
					_leaf_isect_buffers[ray_id] = buffer;
			}
			else
			{	//TODO: no need to intersect
				ray_state.stack.treelet = ray_state.ray_data.raystate.treelet_id;
				ray_state.stack.data = node.data[0];
				ray_state.stack.parent_data = node.parent_data;
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

	// triangle intersection
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		TriStagingBuffer& buffer = _tri_staging_buffers[ray_id];

		RayState& ray_state = _ray_states[ray_id];

		rtm::Ray& ray = ray_state.ray_data.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;

		if(rtm::intersect(buffer.tris[_tri_isect_index].tri, ray, hit))
		{
			hit.id = buffer.tris[_tri_isect_index].id;
			ray_state.ray_data.raystate.hit_id = buffer.tris[_tri_isect_index].id;
			ray_state.ray_data.raystate.hit_t = hit.t;
		}

		_tri_isect_index++;
		if(_tri_isect_index >= buffer.num_tris)
		{
			_assert(ray_state.ray_data.raystate.traversal_state == RayData::RayState::Traversal_State::DOWN);
			// ray_state.ray_data.traversal_stack = ray_state.ray_data.traversal_stack >> 1;
			// ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack >> 1;
			// ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::LEAF_STAY;
			_assert(_leaf_isect_buffers[ray_id].ray_id != ~0u);
			NodeStagingBuffer buffer = _leaf_isect_buffers[ray_id];
			rtm::WideTreeletSTRaTABVH::Treelet::Node& node = buffer.node;
			uint32_t near_visited = ray_state.ray_data.visited_stack & 1;
			if(near_visited)
			{
				uint32_t next_visit = !(ray_state.ray_data.traversal_stack & 1);
				ray_state.stack.treelet = ray_state.ray_data.raystate.treelet_id;
				ray_state.stack.data = node.data[next_visit];
				ray_state.stack.parent_data = node.parent_data;
				_assert(ray_state.ray_data.traversal_stack >> 31 != 1);
				_assert(ray_state.ray_data.visited_stack >> 31 != 1);
				ray_state.ray_data.traversal_stack = ((ray_state.ray_data.traversal_stack >> 1) << 1) | next_visit;
				ray_state.ray_data.visited_stack = (ray_state.ray_data.visited_stack >> 1) << 1;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::DOWN;
			}
			else
			{
				ray_state.stack.treelet = ray_state.ray_data.raystate.treelet_id;
				ray_state.stack.data = node.data[0];
				ray_state.stack.parent_data = node.parent_data;
				ray_state.ray_data.traversal_stack = ray_state.ray_data.traversal_stack >> 1;
				ray_state.ray_data.visited_stack = ray_state.ray_data.visited_stack >> 1;
				ray_state.ray_data.raystate.traversal_state = RayData::RayState::Traversal_State::UP;
				_leaf_isect_buffers[ray_id].ray_id = ~0u;
			}

			_tri_isect_index = 0;
			_tri_isect_queue.pop();
			_tri_pipline.write(ray_id);
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
		if(!_hit_store_queue.empty())
		{
			uint ray_id = _hit_store_queue.front();
			_hit_store_queue.pop();

			RayState& ray_state = _ray_states[ray_id];

			MemoryRequest req;
			req.type = MemoryRequest::Type::STORE;
			req.size = sizeof(RayData);
			req.port = _tm_index;
			req.paddr = ray_state.ray_data.raystate.id;
			std::memcpy(req.data, &ray_state.ray_data, req.size);
			_ray_stream_buffer->write_request(req);

			ray_state.phase = RayState::Phase::RAY_FETCH;
			_ray_data_load_queue.push(ray_id);
		}
		else if(!_ray_buffer_store_queue.empty())
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
			req.paddr = 0xdeadbeefull;
			_ray_stream_buffer->write_request(req);
		}
		else if(!_tp_hit_load_queue.empty())
		{
			//issue hit requests
			MemoryRequest& req = _tp_hit_load_queue.front();
			req.port = _tm_index;
			_ray_stream_buffer->write_request(req);
			_tp_hit_load_queue.pop();
		}
	}
}

void UnitTreeletRTCore::_issue_returns()
{
	if(!_tp_hit_return_queue.empty() && _return_network.is_write_valid(0))
	{
		const MemoryReturn& ret = _tp_hit_return_queue.front();
		_return_network.write(ret, 0);
		_tp_hit_return_queue.pop();
		log.return_hits++;
	}
}


}}}