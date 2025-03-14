#include "unit-treelet-rt-core.hpp"

namespace Arches { namespace Units { namespace STRaTART {

constexpr uint RAY_ID_BITS = 9;

UnitTreeletRTCore::UnitTreeletRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _tm_index(config.tm_index), 
	_treelet_base_addr(config.treelet_base_addr), _hit_record_base_addr(config.hit_record_base_addr),
	_cache(config.cache), _ray_stream_buffer(config.ray_stream_buffer), 
	_request_network(config.num_tp, 1), _return_network(1, config.num_tp),
	_box_pipline(3), _tri_pipline(22)
{
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < config.max_rays; ++i)
	{
		_ray_states[i].phase = Phase::RAY_FETCH;
		_ray_data_load_queue.push(i);
	}
}

bool UnitTreeletRTCore::_try_queue_node(uint ray_id, uint treelet_id, uint node_id)
{
	paddr_t start = (paddr_t)&((Treelet*)_treelet_base_addr)[treelet_id].nodes[node_id];
	paddr_t end = start + sizeof(Treelet::Node);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while (addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, RAY_ID_BITS);
		req.dst.push((uint)Destination::NODE, 1);
		req.port = _num_tp;
		req.paddr = addr;

		_cache_fetch_queue.push(req);
		addr += req.size;
	}
	return true;
}

bool UnitTreeletRTCore::_try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset, uint num_tris)
{
	paddr_t start = (paddr_t)&((Treelet*)_treelet_base_addr)[treelet_id].data[tri_offset];
	paddr_t end = start + sizeof(Treelet::Triangle) * num_tris;

	_assert(start < 4ull * 1204 * 1024 * 1024);

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
		req.dst.push(ray_id, RAY_ID_BITS);
		req.dst.push((uint)Destination::TRI, 1);
		req.port = _num_tp;
		req.paddr = addr;

		_cache_fetch_queue.push(req);
		addr += req.size;
	}

	return true;
}

bool UnitTreeletRTCore::_try_queue_prefetch(paddr_t addr, uint size, uint cache_mask)
{
	//printf("%3d Prefetching: %d\n", _rtc_index, treelet_id);
	paddr_t start = addr;
	paddr_t end = start + size;

	//split request at cache boundries
	//queue the requests to fill the buffer
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::PREFECTH;
		req.size = next_boundry - addr;
		req.port = _num_tp;
		req.paddr = addr;
		req.flags.omit_cache = cache_mask;

		_cache_fetch_queue.push(req);
		addr += req.size;
	}
	log.issue_counters[(uint)IssueType::PREFETCH]++;

	return true;
}


void UnitTreeletRTCore::_read_requests()
{
	if(_request_network.is_read_valid(0))
	{
		MemoryRequest request = _request_network.read(0);
		if (request.size == sizeof(STRaTARTKernel::RayData))			// forward ray buffer write
		{
			STRaTARTKernel::RayData ray_data;
			std::memcpy(&ray_data, request.data, sizeof(STRaTARTKernel::RayData));
			_ray_buffer_store_queue.push(ray_data);
		}
		else if(request.size == sizeof(STRaTARTKernel::HitReturn))		// load hit
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
		if(ret.size == sizeof(STRaTARTKernel::RayData))
		{
			STRaTARTKernel::RayData ray_data;
			std::memcpy(&ray_data, ret.data, sizeof(STRaTARTKernel::RayData));
			uint ray_id = ret.dst.pop(RAY_ID_BITS);
			RayState& ray_state = _ray_states[ray_id];
			ray_state = RayState(ray_data);

			for(uint i = 0; i < 8; ++i)
				if((ray_data.pf_mask >> i) & 0x1)
					_try_queue_prefetch(_treelet_base_addr + ray_data.treelet_id * sizeof(Treelet) + Treelet::PAGE_SIZE * i, Treelet::PAGE_SIZE, 0b010);

			ray_state.phase = Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);

			log.rays++;
			_processing_rays++;
		}
		else if(ret.size == sizeof(STRaTARTKernel::HitReturn))
		{
			_tp_hit_return_queue.push(ret);
		}
		else
			_assert(0);
	}

	if(_cache->return_port_read_valid(_num_tp))
	{
		MemoryReturn ret = _cache->read_return(_num_tp);
		uint type = ret.dst.pop(1);
		uint16_t ray_id = ret.dst.pop(RAY_ID_BITS);
		StagingBuffer& buffer = _ray_states[ray_id].buffer;
		if(type == TRI)
		{
			uint offset = (ret.paddr - buffer.address);
			std::memcpy((uint8_t*)&buffer.tris + offset, ret.data, ret.size);

			buffer.bytes_filled += ret.size;
			if(buffer.bytes_filled == sizeof(Treelet::Triangle) * buffer.num_tris)
			{
				_ray_states[ray_id].phase = Phase::TRI_ISECT;
				_tri_isect_queue.push(ray_id);
			}
		}
		else if(type == NODE)
		{
			uint offset = (ret.paddr - buffer.address);
			std::memcpy((uint8_t*)&buffer.node + offset, ret.data, ret.size);
			buffer.bytes_filled += ret.size;
			if (buffer.bytes_filled == sizeof(Treelet::Node))
			{
				_ray_states[ray_id].phase = Phase::NODE_ISECT;
				_node_isect_queue.push(ray_id);
			}
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
		STRaTARTKernel::RayData& ray_data = ray_state.ray_data;
		
		StackEntry entry;
		if(ray_state.update_restart_trail)
		{
			uint parent_level = ray_data.restart_trail.find_parent_level(ray_data.level);
			if(parent_level == ~0u)
			{
				//Ray complete
				ray_data.restart_trail.mark_done();
				_ray_buffer_store_queue.push(ray_data);

				ray_state.phase = Phase::RAY_FETCH;
				_ray_data_load_queue.push(ray_id);

				log.issue_counters[(uint)IssueType::RAY_COMPLETE]++;
				if(ENABLE_RT_DEBUG_PRINTS || ENABLE_HIT_DEBUG_PRINTS)
					printf("[%lld] [%03d:%03d:%06d] RAY_COMPLETE, hit_id: %d, global_ray_id: %d\n", simulator->current_cycle, _tm_index, ray_id, ray_data.global_ray_id, ray_data.hit.id, ray_data.global_ray_id);
				_processing_rays--;

				return;
			}

			ray_data.restart_trail.set(parent_level, ray_data.restart_trail.get(parent_level) + 1);
			ray_data.restart_trail.clear(parent_level + 1);

			if(ray_state.stack_size == 0)
			{
				//Restart
				ray_data.level = parent_level;
				//ray_data.treelet_id = 0;
				_ray_buffer_store_queue.push(ray_data);

				ray_state.phase = Phase::RAY_FETCH;
				_ray_data_load_queue.push(ray_id);

				log.issue_counters[(uint)IssueType::RESTART]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("[%lld] [%03d:%03d:%06d] RESTART\n", simulator->current_cycle, _tm_index, ray_id, ray_data.global_ray_id);
				_processing_rays--;

				return;
			}
			else
			{
				entry = ray_state.stack[--ray_state.stack_size];
				if(entry.is_last)
					ray_data.restart_trail.set(parent_level, STRaTARTKernel::RestartTrail::N);
				ray_data.level = parent_level + 1;
			}
		}
		else
		{
			_assert(ray_state.stack_size > 0);
			entry = ray_state.stack[--ray_state.stack_size];
		}

		ray_state.update_restart_trail = true;

		if(entry.t < ray_state.ray_data.hit.t) //pop cull
		{
			if(entry.data.is_int)
			{
				if(entry.data.is_child_treelet)
				{
					//We crossed a treelet boundry so write ray back
					ray_data.treelet_id = entry.data.child_index;
					_ray_buffer_store_queue.push(ray_data);

					ray_state.phase = Phase::RAY_FETCH;
					_ray_data_load_queue.push(ray_id);

					log.issue_counters[(uint)IssueType::RAY_STORE]++;
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("[%lld] [%03d:%03d:%06d] TREELET_JUMP: %d\n", simulator->current_cycle, _tm_index, ray_id, ray_data.global_ray_id, ray_data.treelet_id);
					_processing_rays--;
				}
				else
				{
					_try_queue_node(ray_id, ray_state.ray_data.treelet_id, entry.data.child_index);
					ray_state.phase = Phase::NODE_FETCH;

					log.issue_counters[(uint)IssueType::NODE_ISSUE]++;
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("[%lld] [%03d:%03d:%06d] NODE_ISSUE: %d:%d\n", simulator->current_cycle, _tm_index, ray_id, ray_data.global_ray_id, ray_data.treelet_id, entry.data.child_index);
				}
			}
			else
			{
				_try_queue_tri(ray_id, ray_state.ray_data.treelet_id, entry.data.triangle_index * 4, entry.data.num_tri);
				ray_state.phase = Phase::TRI_FETCH;

				log.issue_counters[(uint)IssueType::TRI_ISSUE]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("[%lld] [%03d:%03d:%06d] TRI: %d:%d\n", simulator->current_cycle, _tm_index, ray_id, ray_data.global_ray_id, entry.data.triangle_index, entry.data.num_tri);
			}
		}
		else
		{
			_ray_scheduling_queue.push(ray_id);
			log.issue_counters[(uint)IssueType::POP_CULL]++;
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("[%lld] [%03d:%03d:%06d] POP_CULL\n", simulator->current_cycle, _tm_index, ray_id, ray_data.global_ray_id);
		}
	}
}

void UnitTreeletRTCore::_simualte_intersectors()
{
	// box intersection
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		STRaTARTKernel::RayData& ray_data = ray_state.ray_data;
		StagingBuffer& buffer = ray_state.buffer;

		rtm::Ray& ray = ray_state.ray_data.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.ray_data.hit;
		const rtm::WideTreeletBVH::Treelet::Node node = buffer.node.decompress();

		_boxes_issued += 6;
		if(_boxes_issued >= rtm::WideTreeletBVH::WIDTH)
		{
			uint k = ray_data.restart_trail.get(ray_data.level);

			uint nodes_pushed = 0;
			for(int i = 0; i < rtm::WideTreeletBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = rtm::intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = ray_state.stack_size + nodes_pushed++;
					for(;j > ray_state.stack_size; --j)
					{
						if(ray_state.stack[j - 1].t > t) break;
						ray_state.stack[j] = ray_state.stack[j - 1];
					}

					ray_state.stack[j].t = t;
					ray_state.stack[j].is_last = false;
					ray_state.stack[j].data = node.data[i];
				}
			}

			if(k == STRaTARTKernel::RestartTrail::N) nodes_pushed = 1;
			else                     nodes_pushed -= std::min(nodes_pushed, k);

			if(nodes_pushed > 0)
			{
				ray_state.update_restart_trail = false;
				if(nodes_pushed == 1) ray_data.restart_trail.set(ray_data.level, STRaTARTKernel::RestartTrail::N);
				else                  ray_state.stack[ray_state.stack_size].is_last = true;
				ray_state.stack_size += nodes_pushed;
				ray_data.level++;
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
			_ray_states[ray_id].phase = Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
			log.nodes++;
		}
	}

	// triangle intersection
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		_tris_issued++;
		if(_tris_issued >= buffer.num_tris)
		{
			rtm::Ray& ray = ray_state.ray_data.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.ray_data.hit;

			for(uint i = 0; i < buffer.num_tris; ++i)
				if(rtm::intersect(buffer.tris[i].tri, ray, hit))
					hit.id = buffer.tris[i].id;

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
			_ray_states[ray_id].phase = Phase::SCHEDULER;
			_ray_scheduling_queue.push(ray_id);
		}

		log.tris++;
	}
}

void UnitTreeletRTCore::_issue_requests()
{
	if(!_cache_fetch_queue.empty() && _cache->request_port_write_valid(_num_tp))
	{
		//fetch the next block
		_cache->write_request(_cache_fetch_queue.front());
		_cache_fetch_queue.pop();
	}

	if(_ray_stream_buffer->request_port_write_valid(_tm_index))
	{
		if(!_ray_buffer_store_queue.empty())
		{
			STRaTARTKernel::RayData ray_data = _ray_buffer_store_queue.front();
			_ray_buffer_store_queue.pop();

			MemoryRequest req;
			req.type = MemoryRequest::Type::STORE;
			req.size = sizeof(STRaTARTKernel::RayData);
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
			req.size = sizeof(STRaTARTKernel::RayData);
			req.port = _tm_index;
			req.dst.push(ray_id, RAY_ID_BITS);
			req.paddr = 0xdeadbeefull;
			_ray_stream_buffer->write_request(req);
		}
		else if(!_tp_hit_load_queue.empty())
		{
			//issue hit requests
			MemoryRequest& req = _tp_hit_load_queue.front();
			req.dst.push(req.port, 8);
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
		MemoryReturn ret = _tp_hit_return_queue.front();
		ret.port = ret.dst.pop(8);
		_return_network.write(ret, 0);
		_tp_hit_return_queue.pop();
	}
}


}}}