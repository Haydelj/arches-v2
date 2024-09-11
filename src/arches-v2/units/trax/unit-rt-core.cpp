#include "unit-rt-core.hpp"

namespace Arches { namespace Units { namespace TRaX {

UnitRTCore::UnitRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _node_base_addr(config.node_base_addr), _tri_base_addr(config.tri_base_addr),
	_cache(config.cache), _request_network(config.num_tp, 1), _return_network(1, config.num_tp),
	_box_pipline(3, 1), _tri_pipline(22, 1)
{
	_tri_staging_buffers.resize(config.max_rays);
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
		_free_ray_ids.insert(i);
}

bool UnitRTCore::_try_queue_node(uint ray_id, uint node_id)
{
	paddr_t start = _node_base_addr + node_id * sizeof(rtm::WideBVH::WideBVHNode);
	_fetch_queue.push({start, (uint8_t)(sizeof(rtm::WideBVH::WideBVHNode)), (uint16_t)ray_id});
	return true;
}

bool UnitRTCore::_try_queue_tri(uint ray_id, uint tri_id)
{
	paddr_t start = _tri_base_addr + tri_id * sizeof(rtm::Triangle);
	paddr_t end = start + sizeof(rtm::Triangle);

	_tri_staging_buffers[ray_id].tri_id = tri_id;
	_tri_staging_buffers[ray_id].bytes_filled = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_fetch_queue.push({addr, size, (uint16_t)(ray_id | 0x8000u)});
		addr += size;
	}

	return true;
}

void UnitRTCore::_read_requests()
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
		ray_state.stack[0].data.is_leaf = 0;
		ray_state.stack[0].data.child_index = 0;
		ray_state.current_entry = 0;
		ray_state.flags = request.flags;
		ray_state.dst = request.dst;
		ray_state.port = request.port;

		ray_state.phase = RayState::Phase::SCHEDULER;
		_ray_scheduling_queue.push(ray_id);

		log.rays++;
	}
}

void UnitRTCore::_read_returns()
{
	if(_cache->return_port_read_valid(_num_tp))
	{
		const MemoryReturn ret = _cache->read_return(_num_tp);
		uint16_t ray_id = ret.dst & ~0x8000u;
		if(ret.dst & 0x8000)
		{
			TriStagingBuffer& buffer = _tri_staging_buffers[ray_id];

			paddr_t buffer_addr = _tri_base_addr + buffer.tri_id * sizeof(rtm::Triangle);
			uint offset = (ret.paddr - buffer_addr);

			std::memcpy((uint8_t*)&buffer.tri + offset, ret.data, ret.size);

			buffer.bytes_filled += ret.size;
			if(buffer.bytes_filled == sizeof(rtm::Triangle))
			{
				_ray_states[ray_id].phase = RayState::Phase::TRI_ISECT;
				_tri_isect_queue.push(ray_id);
			}
		}
		else
		{
			_assert(sizeof(rtm::WideBVH::WideBVHNode) == ret.size);

			NodeStagingBuffer buffer;
			buffer.ray_id = ray_id;
			std::memcpy(&buffer.node, ret.data, ret.size);

			_ray_states[ray_id].phase = RayState::Phase::NODE_ISECT;
			_node_isect_queue.push(buffer);
		}
	}
}

void UnitRTCore::_schedule_ray()
{
	//pop a entry from next rays stack and queue it up
	if(!_ray_scheduling_queue.empty())
	{
		uint ray_id = _ray_scheduling_queue.front();
		_ray_scheduling_queue.pop();

		if(ENABLE_RT_DEBUG_PRINTS)
			printf("Scheduling: %d\n", ray_id);

		RayState& ray_state = _ray_states[ray_id];

		bool any_hit_found = (ray_state.flags & 0x1) && ray_state.hit.id != ~0u;
		if(!any_hit_found && ray_state.stack_size > 0)
		{
			RayState::StackEntry& entry = ray_state.stack[ray_state.stack_size - 1];
			if(entry.t < ray_state.hit.t) //pop cull
			{
				if(entry.data.is_leaf)
				{
					if(_try_queue_tri(ray_id, entry.data.prim_index))
					{
						ray_state.phase = RayState::Phase::TRI_FETCH;

						if(ENABLE_RT_DEBUG_PRINTS)
							printf("Tri: %d:%d\n", entry.data.prim_index, entry.data.num_prims + 1);

						if(entry.data.num_prims == 0)
						{
							ray_state.stack_size--;
						}
						else
						{
							entry.data.prim_index++;
							entry.data.num_prims--;
						}
					}
					else
					{
						_ray_scheduling_queue.push(ray_id);
					}
				}
				else
				{
					if(_try_queue_node(ray_id, entry.data.child_index))
					{
						ray_state.phase = RayState::Phase::NODE_FETCH;

						if(ENABLE_RT_DEBUG_PRINTS)
							printf("Node: %d\n", entry.data.child_index);

						ray_state.stack_size--;
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
			}
		}
		else
		{
			//stack empty or anyhit found return the hit
			if(ENABLE_RT_DEBUG_PRINTS)
				printf("Ret: %d\n", ray_state.hit.id);

			ray_state.phase = RayState::Phase::HIT_RETURN;
			_ray_return_queue.push(ray_id);
		}
	}
}

void UnitRTCore::_simualte_intersectors()
{
	if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
	{
		NodeStagingBuffer& buffer = _node_isect_queue.front();

		uint ray_id = buffer.ray_id;
		RayState& ray_state = _ray_states[ray_id];

		rtm::Ray& ray = ray_state.ray;
		rtm::vec3& inv_d = ray_state.inv_d;
		rtm::Hit& hit = ray_state.hit;
		rtm::WideBVH::WideBVHNode& node = buffer.node;

		//float hit_ts[2] = {rtm::intersect(node.aabb[0], ray, inv_d), rtm::intersect(node.aabb[1], ray, inv_d)};
		//if(hit_ts[0] < hit_ts[1])
		//{
		//	if(hit_ts[1] < hit.t) ray_state.stack[ray_state.stack_size++] = {hit_ts[1], node.data[1]};
		//	if(hit_ts[0] < hit.t) ray_state.stack[ray_state.stack_size++] = {hit_ts[0], node.data[0]};
		//}
		//else
		//{
		//	if(hit_ts[0] < hit.t) ray_state.stack[ray_state.stack_size++] = {hit_ts[0], node.data[0]};
		//	if(hit_ts[1] < hit.t) ray_state.stack[ray_state.stack_size++] = {hit_ts[1], node.data[1]};
		//}

		int childCount;
		rtm::BVH2::Node dnodes[n_ary_sz];
		node.decompress(dnodes, childCount);

		uint max_insert_depth = ray_state.stack_size;
		for (int i = 0; i < childCount; i++)
		{
			float t = rtm::intersect(dnodes[i].aabb, ray, inv_d);
			if (t < hit.t)
			{
				uint j = ray_state.stack_size++;
				for (; j > max_insert_depth; --j)
				{
					if (ray_state.stack[j - 1].t > t) break;
					ray_state.stack[j] = ray_state.stack[j - 1];
				}
				ray_state.stack[j].t = t;
				ray_state.stack[j].data = dnodes[i].data;
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

		if(rtm::intersect(buffer.tri, ray, hit))
			hit.id = buffer.tri_id;

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

void UnitRTCore::_issue_requests()
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
}

void UnitRTCore::_issue_returns()
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
			ret.dst = ray_state.dst;
			ret.port = ray_state.port;
			ret.paddr = 0xdeadbeefull;
			std::memcpy(ret.data, &ray_state.hit, sizeof(rtm::Hit));
			_return_network.write(ret, 0);

			ray_state.phase = RayState::Phase::NONE;
			_free_ray_ids.insert(ray_id);
			_ray_return_queue.pop();
		}
	}
}

}}}