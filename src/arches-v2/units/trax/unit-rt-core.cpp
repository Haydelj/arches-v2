#include "unit-rt-core.hpp"
#include "rtm/hecwbvh.hpp"

namespace Arches { namespace Units { namespace TRaX {

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 4)
//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 4 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

template<typename NT, typename PT>
UnitRTCore<NT, PT>::UnitRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _node_base_addr(config.node_base_addr), _tri_base_addr(config.tri_base_addr), _vrt_base_addr(config.vrt_base_addr),
	_cache(config.cache), _request_network(config.num_clients, 1), _return_network(1, config.num_clients),
	_box_pipline(3), _tri_pipline(22), _cache_port(config.cache_port), _cache_port_stride(config.cache_port_stride)
{
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
	{
		_ray_states[i].phase = RayState::Phase::RAY_FETCH;
		_free_ray_ids.insert(i);
	}

	_cache_fetch_queues.resize(config.num_cache_ports);
}
template<typename NT, typename PT>
void UnitRTCore<NT, PT>::clock_rise()
{
	_request_network.clock();
	_read_requests();
	_read_returns();

	for(uint i = 0; i < 1; ++i) //n stack ops per cycle. In reality this would need to be multi banked
		_schedule_ray();

	_simualte_node_pipline();
	_simualte_tri_pipline();
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::clock_fall()
{
	_issue_requests();
	_issue_returns();
	_return_network.clock();
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_node(uint ray_id, uint node_id)
{
	paddr_t start = _node_base_addr + node_id * sizeof(NT);
	paddr_t end = start + sizeof(NT);

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 0;
	ray_state.buffer.id = node_id;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _align_address(addr + MemoryRequest::MAX_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.paddr = addr;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, 10);
		_cache_fetch_queues[ray_id % _cache_fetch_queues.size()].push(req);

		addr += req.size;
	}

	return true;
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_tri(uint ray_id, uint tri_id)
{
	paddr_t start = _tri_base_addr + tri_id * sizeof(PT);
	if(typeid(NT) == typeid(rtm::HECWBVH::Node))
		start = _node_base_addr + tri_id * sizeof(NT);

	paddr_t end = start + sizeof(PT);
	if(_vrt_base_addr != 0x0ull)
	{
		start = _tri_base_addr + tri_id * sizeof(rtm::IndexStrip);
		end = start + sizeof(rtm::IndexStrip);
	}

	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.address = start;
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 1;
	ray_state.buffer.id = tri_id;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _align_address(addr + MemoryRequest::MAX_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.paddr = addr;
		req.size = next_boundry - addr;
		req.dst.push(ray_id, 10);
		_cache_fetch_queues[ray_id % _cache_fetch_queues.size()].push(req);

		addr += req.size;
	}

	return true;
}

template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_vrts(uint ray_id)
{
	RayState& ray_state = _ray_states[ray_id];
	ray_state.buffer.bytes_filled = 0;
	ray_state.buffer.type = 2;

	rtm::IndexStrip& index_strip = ray_state.buffer.index_strip;
	for(uint i = 0; i < index_strip.num_tris + 2; ++i)
	{
		uint index = index_strip.inds[i];

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.paddr = _vrt_base_addr + 16 * index;
		req.size = 16;
		req.dst.push(i, 8);
		req.dst.push(ray_id, 10);
		_cache_fetch_queues[ray_id % _cache_fetch_queues.size()].push(req);
	}

	return true;
}



template<typename NT, typename PT>
bool UnitRTCore<NT, PT>::_try_queue_prefetch(paddr_t addr, uint size, uint cache_mask)
{
	//printf("%3d Prefetching: %d\n", _rtc_index, treelet_id);
	paddr_t start = addr;
	paddr_t end = start + size;

	//split request at cache boundries
	//queue the requests to fill the buffer
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _align_address(addr + MemoryRequest::MAX_SIZE));

		MemoryRequest req;
		req.type = MemoryRequest::Type::PREFECTH;
		req.size = next_boundry - addr;
		req.paddr = addr;
		req.flags.omit_cache = cache_mask;
		_cache_fetch_queues[0].push(req);
		addr += req.size;
	}

	return true;
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_read_requests()
{
	//simulate waves of rays
	//if(_free_ray_ids.size() == _max_rays) _drain_phase = false;

	if(!_drain_phase && !_free_ray_ids.empty() && _request_network.is_read_valid(0))
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
		ray_state.stack[0].t = ray_state.ray.t_min;
		ray_state.stack[0].data.is_int = 1;
		ray_state.stack[0].data.child_index = 0;
		ray_state.stack[0].is_last = false;
		ray_state.stack_size = 1;
		ray_state.level = 0;
		ray_state.update_restart_trail = false;
		ray_state.restart_trail = rtm::RestartTrail();
		ray_state.flags = request.flags;
		ray_state.dst = request.dst;
		ray_state.dst.push(request.port, 8);
		ray_state.phase = RayState::Phase::SCHEDULER;
		_ray_scheduling_queue.push(ray_id);

		log.rays++;

		//_ray_return_queue.push(ray_id);
	}

	//if(_free_ray_ids.empty()) _drain_phase = true;

	_stall_cycles++;
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_read_returns()
{
	for(uint i = 0; i < _cache_fetch_queues.size(); ++i)
	{
		uint port = _cache_port + i * _cache_port_stride;
		if(_cache->return_port_read_valid(port))
		{
			MemoryReturn ret = _cache->read_return(port);
			uint16_t ray_id = ret.dst.pop(10);
			RayState& ray_state = _ray_states[ray_id];
			StagingBuffer& buffer = ray_state.buffer;

			//if(ENABLE_RT_DEBUG_PRINTS) printf("ret %xll\n", ret.paddr);

			if(buffer.type == 0)
			{
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == sizeof(NT))
				{
					ray_state.phase = RayState::Phase::NODE_ISECT;
					_node_isect_queue.push(ray_id);
				}
			}
			else if(buffer.type == 1)
			{
				uint offset = (ret.paddr - buffer.address);
				std::memcpy((uint8_t*)&buffer.data + offset, ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(_vrt_base_addr == 0x0ull)
				{
					if(buffer.bytes_filled == sizeof(PT))
					{

						ray_state.phase = RayState::Phase::TRI_ISECT;
						_tri_isect_queue.push(ray_id);
					}
				}
				else
				{
					if(buffer.bytes_filled == sizeof(rtm::IndexStrip))
					{
						ray_state.phase = RayState::Phase::VRT_FETCH;
						_try_queue_vrts(ray_id);
					}
				}
			}
			else if(buffer.type == 2)
			{
				rtm::vec4 vrt;
				std::memcpy(&vrt, ret.data, sizeof(rtm::vec4));
				uint index = ret.dst.pop(8);
				buffer.tri_strip.vrts[index].x = vrt.x;
				buffer.tri_strip.vrts[index].y = vrt.y;
				buffer.tri_strip.vrts[index].z = vrt.z;

				buffer.bytes_filled += 12;
				if((buffer.tri_strip.num_tris + 2) * 12 == buffer.bytes_filled)
				{
					ray_state.phase = RayState::Phase::TRI_ISECT;
					_tri_isect_queue.push(ray_id);
				}
			}
		}
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		_stall_cycles = 0;
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		RayState& ray_state = _ray_states[ray_id];

		StackEntry entry;
		if(ray_state.update_restart_trail)
		{
			uint parent_level = ray_state.restart_trail.find_parent_level(ray_state.level);
			if(parent_level == ~0u)
			{
				//Ray complete
				//stack empty or anyhit found return the hit
				ray_state.phase = RayState::Phase::HIT_RETURN;
				_ray_return_queue.push(ray_id);

				log.issue_counters[(uint)IssueType::HIT_RETURN]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d HIT_RETURN: %d\n", ray_id, ray_state.hit.id);

				return;
			}

			ray_state.restart_trail.set(parent_level, ray_state.restart_trail.get(parent_level) + 1);
			ray_state.restart_trail.clear(parent_level + 1);

			if(ray_state.stack_size == 0)
			{
				//Restart
				entry.t = ray_state.ray.t_min;
				entry.data.is_int = 1;
				entry.data.child_index = 0;
				ray_state.level = 0;
				log.restarts++;
			}
			else
			{
				entry = ray_state.stack[--ray_state.stack_size];
				if(entry.is_last)
					ray_state.restart_trail.set(parent_level, rtm::RestartTrail::N);
				ray_state.level = parent_level + 1;
			}
		}
		else
		{
			_assert(ray_state.stack_size > 0);
			entry = ray_state.stack[--ray_state.stack_size];
		}

		ray_state.update_restart_trail = true;

		if(!_pop_culling || entry.t < ray_state.hit.t)
		{
			if(entry.data.is_int)
			{
				_try_queue_node(ray_id, entry.data.child_index);
				ray_state.phase = RayState::Phase::NODE_FETCH;

				log.issue_counters[(uint)IssueType::NODE_FETCH]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d NODE_FETCH: %d\n", ray_id, entry.data.child_index);
			}
			else
			{
				_try_queue_tri(ray_id, entry.data.prim_index);
				if(entry.data.num_prims > 1)
				{
					entry.data.num_prims--;
					entry.data.prim_index++;
					ray_state.stack[ray_state.stack_size++] = entry;
					ray_state.update_restart_trail = false;
				}

				ray_state.phase = RayState::Phase::TRI_FETCH;

				log.issue_counters[(uint)IssueType::TRI_FETCH]++;
				if(ENABLE_RT_DEBUG_PRINTS)
					printf("%03d TRI_FETCH: %d:%d\n", ray_id, entry.data.prim_index, entry.data.num_prims);
			}
		}
		else //pop cull
		{
			_ray_scheduling_queue.push(ray_id);

			log.issue_counters[(uint)IssueType::POP_CULL]++;
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("%03d POP_CULL\n", ray_id);
		}
	}
	else
	{
		uint phase = (uint)_ray_states[_last_ray_id].phase;
		if(++_last_ray_id == _ray_states.size()) _last_ray_id = 0;
		log.stall_counters[phase]++;
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_simualte_node_pipline()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		_stall_cycles = 0;
		uint ray_id = _node_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];

		rtm::Hit& hit = ray_state.hit;
		const rtm::Ray& ray = ray_state.ray;
		const rtm::vec3& inv_d = ray_state.inv_d;
		const rtm::WBVH::Node node = rtm::decompress(ray_state.buffer.node);

		_box_issue_count += rtm::WBVH::WIDTH;
		if(_box_issue_count >= node.num_aabb())
		{
			uint k = ray_state.restart_trail.get(ray_state.level);

			uint nodes_pushed = 0;
			for(uint i = 0; i < rtm::WBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = rtm::intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = ray_state.stack_size + nodes_pushed++;
					for(; j > ray_state.stack_size; --j)
					{
						if(ray_state.stack[j - 1].t > t) break;
						ray_state.stack[j] = ray_state.stack[j - 1];
					}

					ray_state.stack[j].t = t;
					ray_state.stack[j].is_last = false;
					ray_state.stack[j].data = node.data[i];
				}
			}

			if(k == rtm::RestartTrail::N) nodes_pushed = 1;
			else                          nodes_pushed -= std::min(nodes_pushed, k);

			if(nodes_pushed > 0)
			{
				ray_state.update_restart_trail = false;
				if(nodes_pushed == 1) ray_state.restart_trail.set(ray_state.level, rtm::RestartTrail::N);
				else                  ray_state.stack[ray_state.stack_size].is_last = true;
				ray_state.stack_size += nodes_pushed;
				ray_state.level++;

				if(ray_state.stack_size > RayState::STACK_SIZE)
				{
					uint drain_count = ray_state.stack_size - RayState::STACK_SIZE;
					for(uint i = 0; i < RayState::STACK_SIZE; ++i)
						ray_state.stack[i] = ray_state.stack[i + drain_count];
					ray_state.stack_size = RayState::STACK_SIZE;
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

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_simualte_tri_pipline()
{
	if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
	{
		_stall_cycles = 0;
		uint ray_id = _tri_isect_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		StagingBuffer& buffer = ray_state.buffer;

		rtm::IntersectionTriangle tris[DGF::MAX_TRIS];
		uint tri_count = rtm::decompress(buffer.prim, buffer.id, tris);

		_tri_issue_count += tri_count;
		if(_tri_issue_count >= tri_count)
		{
			rtm::Ray& ray = ray_state.ray;
			rtm::vec3& inv_d = ray_state.inv_d;
			rtm::Hit& hit = ray_state.hit;

			for(uint i = 0; i < tri_count; ++i)
				if(rtm::intersect(tris[i].tri, ray, hit))
					hit.id = buffer.id + i;

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
			log.strips++;
		}

		log.tris++;
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_issue_requests()
{
	for(uint i = 0; i < _cache_fetch_queues.size(); ++i)
	{
		uint port = _cache_port + i * _cache_port_stride;
		if(!_cache_fetch_queues[i].empty() && _cache->request_port_write_valid(port))
		{
			_cache_fetch_queues[i].front().port = port;
			_cache->write_request(_cache_fetch_queues[i].front());
			_cache_fetch_queues[i].pop();
		}
	}
}

template<typename NT, typename PT>
void UnitRTCore<NT, PT>::_issue_returns()
{
	if(!_ray_return_queue.empty())
	{
		uint ray_id = _ray_return_queue.front();
		RayState& ray_state = _ray_states[ray_id];
		if(ray_state.phase != RayState::Phase::HIT_RETURN) return;

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

template class UnitRTCore<rtm::WBVH::Node, rtm::FTB>;
template class UnitRTCore<rtm::WBVH::Node, rtm::QTB>;
template class UnitRTCore<rtm::WBVH::Node, rtm::DGFMesh::Block>;
template class UnitRTCore<rtm::NVCWBVH::Node, rtm::FTB>;
template class UnitRTCore<rtm::NVCWBVH::Node, rtm::QTB>;
template class UnitRTCore<rtm::NVCWBVH::Node, rtm::DGFMesh::Block>;
template class UnitRTCore<rtm::HECWBVH::Node, rtm::FTB>;
template class UnitRTCore<rtm::HECWBVH::Node, rtm::QTB>;
template class UnitRTCore<rtm::HECWBVH::Node, rtm::DGFMesh::Block>;



}}}