#include "unit-treelet-rt-core.hpp"

namespace Arches { namespace Units { namespace DualStreaming {

UnitTreeletRTCore::UnitTreeletRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _treelet_base_addr(config.treelet_base_addr), _hit_record_base_addr(config.hit_record_base_addr),
	_use_early_termination(config.use_early_termination), _cache(config.cache), _rsb(config.rsb), _request_network(config.num_tp, 1), _return_network(1, config.num_tp),
	_box_pipline(3, 1), _tri_pipline(22, 1)
{
	_tri_staging_buffers.resize(config.max_rays);
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_work_item_load_queue.push(i);
	}
	_active_ray_slots = _ray_states.size();

	unit_name = config.unit_name;
}

bool UnitTreeletRTCore::_try_queue_node(uint ray_id, uint treelet_id, uint node_id)
{
	paddr_t start = (paddr_t)&((rtm::PackedTreelet*)_treelet_base_addr)[treelet_id].nodes[node_id];
	_assert(start < 4ull * 1204 * 1024 * 1024);
	_fetch_queue.push({start, (uint8_t)(sizeof(rtm::PackedTreelet::Node)), (uint16_t)ray_id});
	return true;
}

bool UnitTreeletRTCore::_try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset)
{
	paddr_t start = (paddr_t)&((rtm::PackedTreelet*)_treelet_base_addr)[treelet_id].bytes[tri_offset];
	paddr_t end = start + sizeof(rtm::PackedTreelet::Triangle);

	_assert(start < 4ull * 1204 * 1024 * 1024);

	_tri_staging_buffers[ray_id].addr = start;
	_tri_staging_buffers[ray_id].bytes_filled = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		if(addr >= (paddr_t)(&((rtm::PackedTreelet*)_treelet_base_addr)[treelet_id + 1])) __debugbreak();

		paddr_t next_boundry = std::min(end, block_address(addr + CACHE_BLOCK_SIZE));
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
		//forward work item write
		MemoryRequest request = _request_network.read(0);
		if(request.size == sizeof(WorkItem))
		{
			WorkItem work_item;
			std::memcpy(&work_item, request.data, sizeof(WorkItem));
			_work_item_store_queue.push(work_item);
			log.rays++;
		}
		else if(request.size == sizeof(rtm::Hit))
		{
			_hit_return_port_map[request.paddr] = request.port;
			request.port = 0;
			_tp_hit_load_queue.push(request);
		}
		else _assert(false);
	}
}

void UnitTreeletRTCore::_read_returns()
{
	if(_rsb->return_port_read_valid(0))
	{
		MemoryReturn ret = _rsb->read_return(0);
		if(ret.size == sizeof(WorkItem))
		{
			//creates a ray entry and queues up the ray
			WorkItem work_item;
			std::memcpy(&work_item, ret.data, sizeof(WorkItem));
			uint ray_id = ret.dst;
			RayState& ray_state = _ray_states[ray_id];

			if(work_item.segment_id != INVALID_SEGMENT_ID)
			{
				ray_state = RayState(work_item);
				if(_use_early_termination)
				{
					ray_state.phase = RayState::Phase::HIT_FETCH;
					_hit_load_queue.push(ray_id);
				}
				else
				{
					ray_state.phase = RayState::Phase::SCHEDULER;
					_ray_scheduling_queue.push(ray_id);
				}
			}
			else
			{
				ray_state.phase = RayState::Phase::NONE;
				--_active_ray_slots;
			}
		}
		else if(ret.size == sizeof(rtm::Hit))
		{
			if(ret.dst >> 15)
			{
				//update hit record for dst
				uint ray_id = ret.dst & ~(0x1 << 15);
				RayState& ray_state = _ray_states[ray_id];

				rtm::Hit hit;
				std::memcpy(&hit, ret.data, ret.size);
				if(hit.t < ray_state.hit.t)
				{
					ray_state.hit_found = false;
					ray_state.hit = hit;
				}

				ray_state.phase = RayState::Phase::SCHEDULER;
				_ray_scheduling_queue.push(ray_id);
			}
			else
			{
				ret.port = _hit_return_port_map[ret.paddr];
				_hit_return_port_map.erase(ret.port);
				_hit_return_queue.push(ret);
			}
		}
		else _assert(false);
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
			if(buffer.bytes_filled == sizeof(rtm::PackedTreelet::Triangle))
			{
				_ray_states[ray_id].phase = RayState::Phase::TRI_ISECT;
				_tri_isect_queue.push(ray_id);
			}
		}
		else
		{
			_assert(sizeof(rtm::PackedTreelet::Node) == ret.size);

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
		_assert(ray_state.treelet_id < 1024 * 1024);
		if(ray_state.nstack_size > 0)
		{
			RayState::NodeStackEntry& entry = ray_state.nstack[ray_state.nstack_size - 1];
			if(entry.t < ray_state.hit.t) //pop cull
			{
				if(entry.data.is_leaf)
				{
					if(_try_queue_tri(ray_id, ray_state.treelet_id, entry.data.tri_offset))
					{
						ray_state.phase = RayState::Phase::TRI_FETCH;
						if(entry.data.num_tri == 0)
						{
							ray_state.nstack_size--;
						}
						else
						{
							entry.data.tri_offset += sizeof(rtm::PackedTreelet::Triangle);
							entry.data.num_tri--;
						}
					}
					else
					{
						_ray_scheduling_queue.push(ray_id);
					}
				}
				else
				{
					if(entry.data.is_child_treelet)
					{
					#if 1
						WorkItem work_item;
						work_item.bray.ray = ray_state.ray;
						work_item.bray.ray.t_max = rtm::min(ray_state.ray.t_max, ray_state.hit.t);
						work_item.bray.id = ray_state.global_ray_id;
						work_item.segment_id = entry.data.child_index;
						work_item.order_hint = ray_state.order_hint++;
						_work_item_store_queue.push(work_item);
					#else
						//this is needed for post traversal early termination
						ray_state.tqueue[ray_state.tqueue_tail++] = {entry.t, entry.data.child_index};
					#endif

						ray_state.nstack_size--;
						_ray_scheduling_queue.push(ray_id);
					}
					else
					{
						if(_try_queue_node(ray_id, ray_state.treelet_id, entry.data.child_index))
						{
							ray_state.phase = RayState::Phase::NODE_FETCH;
							ray_state.nstack_size--;
						}
						else _ray_scheduling_queue.push(ray_id);
					}
				}
			}
			else
			{
				ray_state.nstack_size--;
				_ray_scheduling_queue.push(ray_id);
			}
		}
		else if(ray_state.tqueue_head < ray_state.tqueue_tail)
		{
			RayState::TreeletStackEntry& entry = ray_state.tqueue[ray_state.tqueue_head++];
			if(entry.t < ray_state.hit.t)
			{
				WorkItem work_item;
				work_item.bray.ray = ray_state.ray;
				work_item.bray.ray.t_max = rtm::min(ray_state.ray.t_max, ray_state.hit.t);
				work_item.bray.id = ray_state.global_ray_id;
				work_item.segment_id = entry.index;
				work_item.order_hint = ray_state.order_hint++;
				_work_item_store_queue.push(work_item);
			}
			_ray_scheduling_queue.push(ray_id);
		}
		else
		{
			if(ray_state.hit_found)
			{
				_ray_states[ray_id].phase = RayState::Phase::HIT_UPDATE;
				_hit_store_queue.push(ray_id);
			}
			else
			{
				_ray_states[ray_id].phase = RayState::Phase::RAY_FETCH;
				_work_item_load_queue.push(ray_id);
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

		rtm::Ray& ray = ray_state.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;
		rtm::PackedTreelet::Node& node = buffer.node;

		float hit_ts[2] = {rtm::intersect(node.aabb[0], ray, inv_d), rtm::intersect(node.aabb[1], ray, inv_d)};
		if(hit_ts[0] < hit_ts[1])
		{
			if(hit_ts[1] < hit.t) ray_state.nstack[ray_state.nstack_size++] = {hit_ts[1], node.data[1]};
			if(hit_ts[0] < hit.t) ray_state.nstack[ray_state.nstack_size++] = {hit_ts[0], node.data[0]};
		}
		else
		{
			if(hit_ts[0] < hit.t) ray_state.nstack[ray_state.nstack_size++] = {hit_ts[0], node.data[0]};
			if(hit_ts[1] < hit.t) ray_state.nstack[ray_state.nstack_size++] = {hit_ts[1], node.data[1]};
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

		log.nodes += 2;
	}

	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		TriStagingBuffer& buffer = _tri_staging_buffers[ray_id];

		RayState& ray_state = _ray_states[ray_id];

		rtm::Ray& ray = ray_state.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;

		if(rtm::intersect(buffer.tri.tri, ray, hit))
		{
			ray_state.hit_found = true;
			hit.id = buffer.tri.id;
		}

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
		request.unit_name = unit_name;
		_cache->write_request(request);
		_fetch_queue.pop();
	}

	if(_rsb->request_port_write_valid(0))
	{
		if(!_hit_store_queue.empty())
		{
			uint ray_id = _hit_store_queue.front();
			_hit_store_queue.pop();

			RayState& ray_state = _ray_states[ray_id];

			MemoryRequest req;
			req.type = MemoryRequest::Type::STORE;
			req.size = sizeof(rtm::Hit);
			req.port = 0;
			req.paddr = _hit_record_base_addr + ray_state.global_ray_id * sizeof(rtm::Hit);
			std::memcpy(req.data, &ray_state.hit, req.size);
			_rsb->write_request(req);

			ray_state.phase = RayState::Phase::RAY_FETCH;
			_work_item_load_queue.push(ray_id);
		}
		//stores must be higher priority so that they land before a given thread issues it's next load
		else if(!_work_item_store_queue.empty())
		{
			WorkItem work_item = _work_item_store_queue.front();
			_work_item_store_queue.pop();

			MemoryRequest req;
			req.type = MemoryRequest::Type::STORE;
			req.size = sizeof(WorkItem);
			req.port = 0;
			req.paddr = 0xdeadbeefull;
			std::memcpy(req.data, &work_item, req.size);
			_rsb->write_request(req);
		}
		else if(!_work_item_load_queue.empty())
		{
			uint ray_id = _work_item_load_queue.front();
			_work_item_load_queue.pop();

			MemoryRequest req;
			req.type = MemoryRequest::Type::LOAD;
			req.size = sizeof(WorkItem);
			req.port = 0;
			req.dst = ray_id;
			req.paddr = 0xdeadbeefull;
			_rsb->write_request(req);
		}
		else if(!_hit_load_queue.empty())
		{
			//issue hit requests
			int ray_id = _hit_load_queue.front();
			_hit_load_queue.pop();

			RayState& ray_state = _ray_states[ray_id];

			MemoryRequest req;
			req.type = MemoryRequest::Type::LOAD;
			req.size = sizeof(rtm::Hit);
			req.port = 0;
			req.dst = ray_id | (1 << 15);
			req.paddr = _hit_record_base_addr + ray_state.global_ray_id * sizeof(rtm::Hit);
			_rsb->write_request(req);
		}
		else if(!_tp_hit_load_queue.empty() && _active_ray_slots == 0)
		{
			//issue hit requests
			const MemoryRequest& req = _tp_hit_load_queue.front();
			_rsb->write_request(req);
			_tp_hit_load_queue.pop();
		}
	}
}

void UnitTreeletRTCore::_issue_returns()
{
	//issue hit returns
	if(!_hit_return_queue.empty() && _return_network.is_write_valid(0))
	{
		const MemoryReturn& ret = _hit_return_queue.front();
		_return_network.write(ret, 0);
		_hit_return_queue.pop();
	}
}

}}}