#include "unit-treelet-rt-core.hpp"
#include "ric-kernel/include.hpp"

namespace Arches { namespace Units { namespace RIC {

template<typename TT>
UnitTreeletRTCore<TT>::UnitTreeletRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _treelet_base_addr(config.treelet_base_addr), _ray_state_base_addr(config.ray_state_base_addr),
	_cache(config.cache), _cache_port(config.cache_port), _ray_coalescer(config.ray_coalescer), _rtc_index(config.rtc_index), _ray_id_bits(log2i(_max_rays)), _early_t(config.early_t),
	_request_network(config.num_tp, 1), _return_network(1, config.num_tp), _box_pipline(3), _tri_pipline(22)
{
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_INDEX_FETCH;
		_free_ray_ids.insert(i);
	}
}

template<typename TT>
bool UnitTreeletRTCore<TT>::_try_queue_node(uint ray_id, uint treelet_id, uint node_id)
{
	paddr_t start = (paddr_t)&((TT*)_treelet_base_addr)[treelet_id].nodes[node_id];
	paddr_t end = start + sizeof(TT::Node);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, _ray_id_bits);
		req.dst.push((uint)Destination::NODE, 2);
		req.port = _cache_port;
		req.paddr = addr;

		_cache_fetch_queue.push(req);
		addr += req.size;
	}

	return true;
}

template<typename TT>
bool UnitTreeletRTCore<TT>::_try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset, uint num_tris)
{
	paddr_t start = (paddr_t)&((TT*)_treelet_base_addr)[treelet_id].data[tri_offset];
	paddr_t end = start + sizeof(TT::Triangle) * num_tris;

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.num_tris = num_tris;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, _ray_id_bits);
		req.dst.push((uint)Destination::TRI, 2);
		req.port = _cache_port;
		req.paddr = addr;

		_cache_fetch_queue.push(req);
		addr += req.size;
	}

	return true;
}

template<typename TT>
bool UnitTreeletRTCore<TT>::_try_queue_ray_load(uint ray_id)
{
	RayState& ray_state = _ray_states[ray_id];

	MemoryRequest req;
	req.type = MemoryRequest::Type::LOAD;
	req.size = sizeof(MinRayState);
	req.flags.omit_cache = 0b011;
	req.dst.push(ray_id, _ray_id_bits);
	req.dst.push((uint)Destination::RAY, 2);
	req.port = _cache_port;
	req.paddr = _ray_state_base_addr + sizeof(MinRayState) * ray_state.global_ray_id;

	_cache_fetch_queue.push(req);

	return true;
}

template<typename TT>
bool UnitTreeletRTCore<TT>::_try_queue_cshit(uint ray_id)
{
	RayState& ray_state = _ray_states[ray_id];

	MemoryRequest req;
	req.type = MemoryRequest::Type::CSHIT;
	req.size = sizeof(rtm::Hit);
	req.flags.omit_cache = 0b011;
	req.port = _cache_port;
	req.paddr = _ray_state_base_addr + sizeof(MinRayState) * ray_state.global_ray_id + 32;
	std::memcpy(req.data, &ray_state.hit, sizeof(rtm::Hit));

	_cache_fetch_queue.push(req);

	return true;
}

template<typename TT>
bool UnitTreeletRTCore<TT>::_try_queue_prefetch(uint treelet_id, uint size, uint cache_mask)
{
	//printf("%3d Prefetching: %d\n", _rtc_index, treelet_id);
	paddr_t start = (paddr_t)&((TT*)_treelet_base_addr)[treelet_id].data[0];
	paddr_t end = start + size;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::PREFECTH;
		req.size = next_boundry - addr;
		req.port = _cache_port;
		req.paddr = addr;
		req.flags.omit_cache = cache_mask;

		_cache_fetch_queue.push(req);
		addr += req.size;
	}

	return true;
}



template<typename TT>
void UnitTreeletRTCore<TT>::_read_requests()
{
	if(_request_network.is_read_valid(0))
	{
		//forward work item write
		MemoryRequest request = _request_network.read(0);
		if(request.size == sizeof(uint32_t))
		{
			UnitRayCoalescer::WorkItem wi;
			std::memcpy(&wi.ray_id, request.data, sizeof(uint32_t));
			wi.segment_id = 0;
			wi.order_hint = 0;
			_work_item_store_queue.push(wi);
			log.rays++;
		}
		else if(request.size == sizeof(rtm::Hit))
		{
			_tp_hit_load_queue.push(request);
		}
		else _assert(false);
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_read_returns()
{
	if(_ray_coalescer->return_port_read_valid(_rtc_index))
	{
		const MemoryReturn ret = _ray_coalescer->read_return(_rtc_index);

		uint offset = ret.paddr % sizeof(RayBucket);
		std::memcpy(_ray_bucket_buffer.data + offset, ret.data, ret.size);
		_ray_bucket_buffer.bytes_filled += ret.size;
		if(ret.size == 0) 
			_block_for_traversal = false;
	}

	if(_ray_bucket_buffer.bytes_filled == sizeof(RayBucket))
	{
		uint segment_id = _ray_bucket_buffer.bucket.header.segment_id;
		uint num_rays = _ray_bucket_buffer.bucket.header.num_rays;

		if(_segment_states.count(segment_id) > 0 || _segment_states.size() < 2)
		{
			if(_segment_states.size() == 0)
				_try_queue_prefetch(segment_id, 256 * 64, 0b000);

			_segment_states[segment_id].buckets++;
			_segment_states[segment_id].rays += num_rays;

			for(uint i = 0; i < num_rays; ++i)
				_work_item_return_queue.push({_ray_bucket_buffer.bucket.ray_ids[i], (uint16_t)segment_id, 0});

			_ray_bucket_buffer.bytes_filled = 0;
			_ray_bucket_buffer.busy = false;
		}
	}

	if(_cache->return_port_read_valid(_cache_port))
	{
		MemoryReturn ret = _cache->read_return(_cache_port);
		Destination dst = (Destination)ret.dst.pop(2);
		if(dst == Destination::TP)
		{
			ret.port = ret.dst.pop(8);
			_tp_hit_return_queue.push(ret);
		}
		else
		{
			uint16_t ray_id = ret.dst.pop(_ray_id_bits);
			RayState& ray_state = _ray_states[ray_id];
			if(dst == Destination::NODE)
			{
				StagingBuffer& buffer = ray_state.buffer;
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == sizeof(TT::Node))
				{
					ray_state.phase = RayState::Phase::NODE_ISECT;
					_node_isect_queue.push(ray_id);
				}

				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d NODE_RET\n", ray_id);
			}
			else if(dst == Destination::TRI)
			{
				StagingBuffer& buffer = ray_state.buffer;
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == sizeof(TT::Triangle) * buffer.num_tris)
				{
					ray_state.phase = RayState::Phase::TRI_ISECT;
					_tri_isect_queue.push(ray_id);
				}

				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d TRI_RET: %d\n", ray_id, buffer.bytes_filled);
			}
			else if(dst == Destination::RAY)
			{
				MinRayState ray_state_load;
				std::memcpy(&ray_state_load, ret.data, sizeof(MinRayState));
				ray_state.ray = ray_state_load.ray;
				ray_state.hit = ray_state_load.hit;
				ray_state.inv_d = rtm::vec3(1.0) / ray_state.ray.d;
				ray_state.hit.t = std::min(ray_state.hit.t, ray_state.ray.t_max);

				ray_state.phase = RayState::Phase::SCHEDULER;
				_ray_scheduling_queue.push_back(ray_id);

				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d RAY_RET: %d\n", ray_id, ray_state.global_ray_id);
			}
		}
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_init_ray()
{
	if(!_free_ray_ids.empty() && !_work_item_return_queue.empty())
	{
		uint ray_id = *_free_ray_ids.begin();
		RayState& ray_state = _ray_states[ray_id];
		ray_state = RayState(_work_item_return_queue.front());
		if(_try_queue_ray_load(ray_id))
		{
			ray_state.phase = RayState::Phase::RAY_FETCH;
			_free_ray_ids.erase(ray_id);
			_work_item_return_queue.pop();
			log.rays++;
		}
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop_front();

		RayState& ray_state = _ray_states[ray_id];
		_assert(ray_state.treelet_id < 1024 * 1024);
		if(ray_state.nstack_size > 0)
		{
			NodeStackEntry& entry = ray_state.nstack[--ray_state.nstack_size];
			if(entry.t < ray_state.hit.t) //pop cull
			{
				if(entry.data.is_int)
				{
					if(entry.data.is_child_treelet)
					{
						UnitRayCoalescer::WorkItem work_item;
						work_item.ray_id = ray_state.global_ray_id;
						work_item.segment_id = entry.data.child_index;
						work_item.order_hint = ray_state.order_hint++;
						_work_item_store_queue.push(work_item);
						_ray_scheduling_queue.push_front(ray_id);

						log.issue_counters[(uint)IssueType::STORE_WORK_ITEM]++;
						if(ENABLE_RT_DEBUG_PRINTS)
							printf("%03d STORE_WORK_ITEM: %d\n", ray_id, entry.data.child_index);
					}
					else
					{
						_try_queue_node(ray_id, ray_state.treelet_id, entry.data.child_index);
						ray_state.phase = RayState::Phase::NODE_FETCH;

						log.issue_counters[(uint)IssueType::NODE_FETCH]++;
						if(ENABLE_RT_DEBUG_PRINTS)
							printf("%03d NODE_FETCH: %d\n", ray_id, entry.data.child_index);
					}
				}
				else
				{
					_assert(entry.data.num_tri > 0);
					_try_queue_tri(ray_id, ray_state.treelet_id, entry.data.triangle_index * 4, entry.data.num_tri);
					ray_state.phase = RayState::Phase::TRI_FETCH;

					log.issue_counters[(uint)IssueType::TRI_FETCH]++;
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("%03d TRI_FETCH: %d:%d\n", ray_id, entry.data.triangle_index, entry.data.num_tri);
				}
			}
			else
			{
				_ray_scheduling_queue.push_front(ray_id);

				log.issue_counters[(uint)IssueType::POP_CULL]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d POP_CULL\n", ray_id);
			}
		}
		else
		{
			if(ray_state.hit_found)
			{
				log.hits++;
				_try_queue_cshit(ray_id);
			}

			_segment_states[ray_state.treelet_id].rays--;
			if(_segment_states[ray_state.treelet_id].rays == 0)
			{
				for(uint i = 0; i < _segment_states[ray_state.treelet_id].buckets; ++i)
					_completed_buckets.push(ray_state.treelet_id);
				_segment_states.erase(ray_state.treelet_id);
			}

			ray_state.phase = RayState::Phase::RAY_INDEX_FETCH;
			_free_ray_ids.insert(ray_id);

			log.issue_counters[(uint)IssueType::RETIRE_RAY]++;
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("%03d RETIRE_RAY\n", ray_id);
		}
	}
	else
	{
		uint phase = (uint)_ray_states[_last_ray_id].phase;
		if(++_last_ray_id == _ray_states.size()) _last_ray_id = 0;
		log.stall_counters[phase]++;
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_issue_requests()
{
	if(_ray_coalescer->request_port_write_valid(_rtc_index))
	{
		//stores must be higher priority so that they land before a given thread issues it's next load
		if(!_work_item_store_queue.empty())
		{
			//store a workitem
			UnitRayCoalescer::Request req;
			req.port = _rtc_index;
			req.type = UnitRayCoalescer::Request::Type::STORE_WORKITEM;
			req.swi = _work_item_store_queue.front();
			_ray_coalescer->write_request(req, req.port);
			_work_item_store_queue.pop();
		}
		else if(!_completed_buckets.empty())
		{
			//a bucket completed
			UnitRayCoalescer::Request req;
			req.port = _rtc_index;
			req.type = UnitRayCoalescer::Request::Type::BUCKET_COMPLETE;
			req.bc.segment_id = _completed_buckets.front();
			_ray_coalescer->write_request(req, req.port);
			_completed_buckets.pop();
		}
		else if(!_ray_bucket_buffer.busy && _work_item_return_queue.size() < 16)
		{
			//fetch a new bucket
			UnitRayCoalescer::Request req;
			req.port = _rtc_index;
			req.type = UnitRayCoalescer::Request::Type::LOAD_BUCKET;
			req.lb.previous_segment_id = 0;
			_ray_coalescer->write_request(req, req.port);

			_ray_bucket_buffer.bytes_filled = 0;
			_ray_bucket_buffer.busy = true;
		}
	}

	if(_cache->request_port_write_valid(_cache_port))
	{
		if(!_cache_fetch_queue.empty())
		{
			_cache->write_request(_cache_fetch_queue.front());
			_cache_fetch_queue.pop();
		}
		else if(!_tp_hit_load_queue.empty() && _free_ray_ids.size() == _max_rays && !_block_for_traversal)
		{
			//issue hit requests
			MemoryRequest req = _tp_hit_load_queue.front();
			req.dst.push(req.port, 8);
			req.dst.push((uint)Destination::TP, 2);
			req.port = _cache_port;
			_cache->write_request(req);
			_tp_hit_load_queue.pop();
		}
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_issue_returns()
{
	//issue hit returns
	if(!_tp_hit_return_queue.empty() && _return_network.is_write_valid(0))
	{
		_return_network.write(_tp_hit_return_queue.front(), 0);
		_tp_hit_return_queue.pop();
	}
}

static  rtm::WideTreeletBVH::Treelet::Node decompress(const rtm::WideTreeletBVH::Treelet::Node& node)
{
	return node;
}

static  rtm::WideTreeletBVH::Treelet::Node decompress(const rtm::CompressedWideTreeletBVH::Treelet::Node& node)
{
	return node.decompress();
}

template<typename TT>
void UnitTreeletRTCore<TT>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;
		const rtm::WideTreeletBVH::Treelet::Node node = decompress(buffer.node);

		_boxes_issued += 6;
		if(_boxes_issued >= rtm::WideTreeletBVH::WIDTH)
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;

			uint max_insert_depth = ray_state.nstack_size;
			for(int i = 0; i < rtm::WideTreeletBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = rtm::intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = ray_state.nstack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(ray_state.nstack[j - 1].t > t) break;
						ray_state.nstack[j] = ray_state.nstack[j - 1];
					}
					ray_state.nstack[j].t = t;
					ray_state.nstack[j].data = node.data[i];
				}
			}

			_box_pipline.write(ray_id);
			_node_isect_queue.pop();
			_boxes_issued = 0;
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
			_ray_states[ray_id].steps++;
			_ray_scheduling_queue.push_back(ray_id);
			log.nodes++;
		}
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		_tris_issued++;
		if(_tris_issued >= buffer.num_tris)
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;

			for(uint i = 0; i < buffer.num_tris; ++i)
				if(rtm::intersect(buffer.tris[i].tri, ray, hit))
				{
					ray_state.hit_found = true;
					hit.id = buffer.tris[i].id;
				}

			_tri_pipline.write(ray_id);
			_tri_isect_queue.pop();
			_tris_issued = 0;
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
			_ray_states[ray_id].steps++;
			_ray_scheduling_queue.push_back(ray_id);
		}
		log.tris++;
	}
}

template class UnitTreeletRTCore<rtm::WideTreeletBVH::Treelet>;
template class UnitTreeletRTCore<rtm::CompressedWideTreeletBVH::Treelet>;

}}}