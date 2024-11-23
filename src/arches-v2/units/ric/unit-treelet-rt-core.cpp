#include "unit-treelet-rt-core.hpp"

namespace Arches { namespace Units { namespace RIC {

template<typename TT>
UnitTreeletRTCore<TT>::UnitTreeletRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _treelet_base_addr(config.treelet_base_addr), _hit_record_base_addr(config.hit_record_base_addr), _ray_buffer_base_addr(config.ray_buffer_base_addr),
	_cache(config.cache), _cache_port(config.cache_port), _ray_coalescer(config.ray_coalescer), _hit_record_updater(config.hit_record_updater), _rtc_index(config.rtc_index), _ray_id_bits(log2i(_max_rays)),
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
	ray_state.buffer.type = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_cache_fetch_queue.push({addr, size, (uint16_t)(ray_id)});
		addr += size;
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
	ray_state.buffer.type = 1;
	ray_state.buffer.num_tris = num_tris;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_cache_fetch_queue.push({addr, size, (uint16_t)(ray_id)});
		addr += size;
	}

	return true;
}

template<typename TT>
bool UnitTreeletRTCore<TT>::_try_queue_ray(uint ray_id, uint global_ray_id)
{
	paddr_t start = _ray_buffer_base_addr + sizeof(rtm::Ray) * global_ray_id;
	paddr_t end = start + sizeof(rtm::Ray);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 2;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_cache_fetch_queue.push({addr, size, (uint16_t)(ray_id)});
		addr += size;
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

		if(_ray_bucket_buffer.bytes_filled == sizeof(RayBucket))
		{
			uint segment_id = _ray_bucket_buffer.bucket.header.segment_id;
			uint num_rays = _ray_bucket_buffer.bucket.header.num_rays;
			_segment_states[segment_id].buckets++;
			_segment_states[segment_id].rays += num_rays;

			for(uint i = 0; i < num_rays; ++i)
				_work_item_return_queue.push({_ray_bucket_buffer.bucket.ray_ids[i], (uint16_t)segment_id, 0});

			_ray_bucket_buffer.bytes_filled = 0;
			_ray_bucket_buffer.busy = false;
		}
	}

	if(_hit_record_updater->return_port_read_valid(_rtc_index))
	{
		const MemoryReturn hit_ret = _hit_record_updater->read_return(_rtc_index);
		if(_hit_return_map.count(hit_ret.paddr) > 0)
		{
			MemoryReturn& rtc_ret = _hit_return_map[hit_ret.paddr];
			std::memcpy(rtc_ret.data, hit_ret.data, hit_ret.size);
			_tp_hit_return_queue.push(rtc_ret);
			_hit_return_map.erase(hit_ret.paddr);
		}
	}

	if(_cache->return_port_read_valid(_cache_port))
	{
		const MemoryReturn ret = _cache->read_return(_cache_port);
		uint16_t ray_id = ret.dst.peek(_ray_id_bits);
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		uint offset = (ret.paddr - buffer.address);
		std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
		buffer.bytes_filled += ret.size;

		if(buffer.type == 0)
		{
			if(buffer.bytes_filled == sizeof(TT::Node))
			{
				ray_state.phase = RayState::Phase::NODE_ISECT;
				_node_isect_queue.push(ray_id);
			}
		}
		else if(buffer.type == 1)
		{
			if(buffer.bytes_filled == sizeof(TT::Triangle) * buffer.num_tris)
			{
				ray_state.phase = RayState::Phase::TRI_ISECT;
				_tri_isect_queue.push(ray_id);
			}
		}
		else if(buffer.type == 2)
		{
			if(buffer.bytes_filled == sizeof(rtm::Ray))
			{
				ray_state.ray = ray_state.buffer.ray;
				ray_state.inv_d = rtm::vec3(1.0) / ray_state.ray.d;
				ray_state.hit.t = ray_state.ray.t_max;

				ray_state.phase = RayState::Phase::SCHEDULER;
				_ray_scheduling_queue.push(ray_id);
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

		if(_try_queue_ray(ray_id, ray_state.global_ray_id))
		{
			ray_state.phase = RayState::Phase::RAY_FETCH;
			_free_ray_ids.erase(ray_id);
			_work_item_return_queue.pop();
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
		_ray_scheduling_queue.pop();

		RayState& ray_state = _ray_states[ray_id];
		_assert(ray_state.treelet_id < 1024 * 1024);
		if(ray_state.nstack_size > 0)
		{
			NodeStackEntry& entry = ray_state.nstack[ray_state.nstack_size - 1];
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
				else
				{
					if(_try_queue_tri(ray_id, ray_state.treelet_id, entry.data.triangle_index * 4, entry.data.num_tri))
					{
						ray_state.phase = RayState::Phase::TRI_FETCH;
						ray_state.nstack_size--;
					}
					else _ray_scheduling_queue.push(ray_id);
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
			TreeletStackEntry& entry = ray_state.tqueue[ray_state.tqueue_head++];
			if(entry.t < ray_state.hit.t)
			{
				UnitRayCoalescer::WorkItem work_item;
				work_item.ray_id = ray_state.global_ray_id;
				work_item.segment_id = entry.index;
				work_item.order_hint = ray_state.order_hint++;
				_work_item_store_queue.push(work_item);
			}
			_ray_scheduling_queue.push(ray_id);
		}
		else
		{
			_ray_states[ray_id].phase = RayState::Phase::HIT_UPDATE;
			_hit_store_queue.push(ray_id);
		}
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_issue_requests()
{
	if(_ray_coalescer->request_port_write_valid(_rtc_index))
	{
		//stores must be higher priority so that they land before a given thread issues it's next load
		if(!_ray_bucket_buffer.busy && _work_item_return_queue.size() < _max_rays)
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
		else if(!_work_item_store_queue.empty())
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
	}

	if(_hit_record_updater->request_port_write_valid(_rtc_index))
	{
		if(!_hit_store_queue.empty())
		{
			uint ray_id = _hit_store_queue.front();
			_hit_store_queue.pop();
			RayState& ray_state = _ray_states[ray_id];

			if(ray_state.hit_found)
			{
				DualStreaming::HitRecordUpdaterRequest req;
				req.port = _rtc_index;
				req.type = DualStreaming::HitRecordUpdaterRequest::TYPE::STORE;
				req.hit_info.hit = ray_state.hit;
				req.hit_info.hit_address = _hit_record_base_addr + sizeof(rtm::Hit) * ray_state.global_ray_id;
				_hit_record_updater->write_request(req, req.port);
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
		}
		else if(!_tp_hit_load_queue.empty() && _free_ray_ids.size() == _max_rays)
		{
			//issue hit requests
			DualStreaming::HitRecordUpdaterRequest req;
			req.port = _rtc_index;
			req.type = DualStreaming::HitRecordUpdaterRequest::TYPE::LOAD;
			req.hit_info.hit_address = _tp_hit_load_queue.front().paddr;
			_hit_record_updater->write_request(req, req.port);
			_hit_return_map[req.hit_info.hit_address] = MemoryReturn(_tp_hit_load_queue.front());
			_tp_hit_load_queue.pop();
		}
	}

	if(_cache->request_port_write_valid(_cache_port))
	{
		if(!_cache_fetch_queue.empty())
		{
			//fetch the next block
			MemoryRequest request;
			request.type = MemoryRequest::Type::LOAD;
			request.size = _cache_fetch_queue.front().size;
			request.dst.push(_cache_fetch_queue.front().ray_id, _ray_id_bits);
			request.paddr = _cache_fetch_queue.front().addr;
			request.port = _cache_port;
			_cache->write_request(request);
			_cache_fetch_queue.pop();
		}
	}
}

template<typename TT>
void UnitTreeletRTCore<TT>::_issue_returns()
{
	//issue hit returns
	if(!_tp_hit_return_queue.empty() && _return_network.is_write_valid(0))
	{
		const MemoryReturn& ret = _tp_hit_return_queue.front();
		_return_network.write(ret, 0);
		_tp_hit_return_queue.pop();
	}
}


template<typename TT>
void UnitTreeletRTCore<TT>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		rtm::Ray& ray = ray_state.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;
		const typename TT::Node& node = buffer.node;

		uint max_insert_depth = ray_state.nstack_size;
		for(int i = 0; i < rtm::CompressedWideBVH::WIDTH; i++)
		{
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
void UnitTreeletRTCore<rtm::CompressedWideTreeletBVH::Treelet>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		rtm::Ray& ray = ray_state.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;
		const rtm::WideTreeletBVH::Treelet::Node node = buffer.node.decompress();

		uint max_insert_depth = ray_state.nstack_size;
		for(int i = 0; i < rtm::CompressedWideBVH::WIDTH; i++)
		{
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

template<typename TT>
void UnitTreeletRTCore<TT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		rtm::Ray& ray = ray_state.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;

		if(rtm::intersect(buffer.tris[_tri_isect_index].tri, ray, hit))
		{
			ray_state.hit_found = true;
			hit.id = buffer.tris[_tri_isect_index].id;
		}

		_tri_isect_index++;
		if(_tri_isect_index >= buffer.num_tris)
		{
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

template class UnitTreeletRTCore<rtm::WideTreeletBVH::Treelet>;
template class UnitTreeletRTCore<rtm::CompressedWideTreeletBVH::Treelet>;

}}}