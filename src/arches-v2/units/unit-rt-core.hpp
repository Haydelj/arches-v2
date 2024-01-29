#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "unit-base.hpp"
#include "unit-memory-base.hpp"

namespace Arches { namespace Units {

class UnitRTCore : public UnitMemoryBase
{
private:
	Casscade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;
	UnitMemoryBase*         _cache;

	struct RayState
	{
		struct StackEntry
		{
			float t;
			rtm::BVH::Node::Data data;
		};

		rtm::Ray ray;
		rtm::vec3 inv_d;

		rtm::Hit hit;

		StackEntry stack[32];
		uint8_t stack_size;
		uint8_t current_entry;
		uint16_t flags;

		uint16_t port;
		uint16_t dst;
	};

	struct NodeStagingBuffer
	{
		union
		{
			uint8_t data[1];
			rtm::BVH::Node nodes[2];
		};

		uint32_t first_id;
		uint16_t bytes_filled;
		uint8_t  num_entry;

		std::queue<uint> rays;

		NodeStagingBuffer() {};
	};

	struct TriStagingBuffer
	{
		union
		{
			uint8_t data[1];
			rtm::Triangle tris[8];
		};

		uint32_t first_id;
		uint16_t bytes_filled;
		uint8_t  num_entry;

		std::queue<uint> rays;

		TriStagingBuffer() {};
	};

	struct FetchItem
	{
		paddr_t addr;
		uint8_t size;
		uint16_t dst;
	};

	//ray scheduling hardware
	std::queue<uint> _ray_scheduling_queue;
	std::queue<uint> _ray_return_queue;
	std::queue<FetchItem> _fetch_queue;

	std::set<uint> _free_ray_ids;
	std::vector<RayState> _ray_states;

	//node pipline
	std::set<uint16_t> _free_node_staging_buffers;
	std::vector<NodeStagingBuffer> _node_staging_buffers;
	std::map<uint, uint16_t> _node_staging_buffer_map;
	std::queue<uint> _node_isect_queue;
	Pipline<uint> _box_pipline;

	//tri pipline
	std::set<uint16_t> _free_tri_staging_buffers;
	std::vector<TriStagingBuffer> _tri_staging_buffers;
	std::map<uint, uint16_t> _tri_staging_buffer_map;
	std::queue<uint> _tri_isect_queue;
	Pipline<uint> _tri_pipline;

	//meta data
	uint _max_rays;
	uint _num_tp;
	paddr_t _nodes_base_addr;
	paddr_t _triangles_base_addr;

public:
	UnitRTCore(uint max_rays, uint num_tp, paddr_t nodes_base_addr, paddr_t triangles_base_addr, UnitMemoryBase* cache) : 
		_max_rays(max_rays), _num_tp(num_tp), _nodes_base_addr(nodes_base_addr), _triangles_base_addr(triangles_base_addr),
		_cache(cache), _request_network(num_tp, 1), _return_network(num_tp),
		_box_pipline(3, 1), _tri_pipline(22, 4)
	{
		_node_staging_buffers.resize(max_rays);
		for(uint i = 0; i < _node_staging_buffers.size(); ++i)
			_free_node_staging_buffers.insert(i);

		_tri_staging_buffers.resize(max_rays);
		for(uint i = 0; i < _tri_staging_buffers.size(); ++i)
			_free_tri_staging_buffers.insert(i);

		_ray_states.resize(max_rays);
		for(uint i = 0; i < _ray_states.size(); ++i)
			_free_ray_ids.insert(i);
	}

	paddr_t block_address(paddr_t addr)
	{
		return (addr >> log2i(CACHE_BLOCK_SIZE)) << log2i(CACHE_BLOCK_SIZE);
	}

	bool try_queue_nodes(uint ray_id, uint first_node_id, uint num_nodes)
	{
		paddr_t start = _nodes_base_addr + first_node_id * sizeof(rtm::BVH::Node);

		uint16_t buffer_id;
		if(_node_staging_buffer_map.find(first_node_id) == _node_staging_buffer_map.end())
		{
			if(_free_node_staging_buffers.empty()) return false;
			buffer_id = *_free_node_staging_buffers.begin();

			assert(_node_staging_buffers[buffer_id].rays.empty());

			_free_node_staging_buffers.erase(buffer_id);
			_node_staging_buffer_map[first_node_id] = buffer_id;

			_node_staging_buffers[buffer_id].first_id = first_node_id;
			_node_staging_buffers[buffer_id].num_entry = num_nodes;
			_node_staging_buffers[buffer_id].bytes_filled = 0;


			_fetch_queue.push({start, (uint8_t)(sizeof(rtm::BVH::Node) * num_nodes), buffer_id});
		}
		else
		{
			buffer_id = _node_staging_buffer_map[first_node_id];
		}

		_node_staging_buffers[buffer_id].rays.push(ray_id);
		return true;
	}

	bool try_queue_tris(uint ray_id, uint first_tri_id, uint num_tris)
	{
		paddr_t start = _triangles_base_addr + first_tri_id * sizeof(rtm::Triangle);
		paddr_t end = _triangles_base_addr + (first_tri_id + num_tris) * sizeof(rtm::Triangle);

		uint16_t buffer_id;
		if(_tri_staging_buffer_map.find(first_tri_id) == _tri_staging_buffer_map.end())
		{
			if(_free_tri_staging_buffers.empty()) return false;
			buffer_id = *_free_tri_staging_buffers.begin();

			assert(_tri_staging_buffers[buffer_id].rays.empty());

			_free_tri_staging_buffers.erase(buffer_id);
			_tri_staging_buffer_map[first_tri_id] = buffer_id;

			_tri_staging_buffers[buffer_id].first_id = first_tri_id;
			_tri_staging_buffers[buffer_id].num_entry = num_tris;
			_tri_staging_buffers[buffer_id].bytes_filled = 0;

			//split request at cache boundries
			//queue the requests to fill the buffer
			paddr_t addr = start;
			while(addr < end)
			{
				paddr_t next_boundry = std::min(end, block_address(addr + CACHE_BLOCK_SIZE));
				uint8_t size = next_boundry - addr;
				_fetch_queue.push({addr, size, (uint16_t)(buffer_id | 0x8000u)});
				addr += size;
			}
		}
		else
		{
			buffer_id = _tri_staging_buffer_map[first_tri_id];
		}

		_tri_staging_buffers[buffer_id].rays.push(ray_id);
		return true;
	}

	void clock_rise() override
	{
		//read requests
		_request_network.clock();

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
			ray_state.stack[0].data.fst_chld_ind = 0;
			ray_state.stack[0].data.lst_chld_ofst = 0;
			ray_state.stack[0].data.is_leaf = 0;
			ray_state.current_entry = 0;
			ray_state.flags = request.flags;
			ray_state.dst = request.dst;
			ray_state.port = request.port;

			_ray_scheduling_queue.push(ray_id);
		}


		//read returns
		if(_cache->return_port_read_valid(_num_tp))
		{
			const MemoryReturn ret = _cache->read_return(_num_tp);
			uint16_t buffer_id = ret.dst & ~0x8000u;
			if(ret.dst & 0x8000)
			{
				TriStagingBuffer& buffer = _tri_staging_buffers[buffer_id];
				paddr_t buffer_addr = buffer.first_id * sizeof(rtm::Triangle) + _triangles_base_addr;
				std::memcpy(buffer.data + (ret.paddr - buffer_addr), ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == buffer.num_entry * sizeof(rtm::Triangle))
				{
					_tri_isect_queue.push(buffer_id);
				}
			}
			else
			{
				NodeStagingBuffer& buffer = _node_staging_buffers[buffer_id];
				paddr_t buffer_addr = buffer.first_id * sizeof(rtm::BVH::Node) + _nodes_base_addr;
				std::memcpy(buffer.data + (ret.paddr - buffer_addr), ret.data, ret.size);
				buffer.bytes_filled += ret.size;
				if(buffer.bytes_filled == buffer.num_entry * sizeof(rtm::BVH::Node))
				{
					_node_isect_queue.push(buffer_id);
				}
			}
		}


		//pop a entry from next rays stack and queue it up
		if(!_ray_scheduling_queue.empty())
		{
			uint ray_id = _ray_scheduling_queue.front();
			RayState& ray_state = _ray_states[ray_id];

			//TODO handle any hit
			bool any_hit_found = (ray_state.flags & 0x1) && ray_state.hit.id != ~0u;
			if(!any_hit_found && ray_state.stack_size > 0)
			{
				RayState::StackEntry& entry = ray_state.stack[--ray_state.stack_size];
				if(entry.t < ray_state.hit.t) //pop cull
				{
					if(entry.data.is_leaf) 
					{
						if(try_queue_tris(ray_id, entry.data.fst_chld_ind, entry.data.lst_chld_ofst + 1))
						{
							_ray_scheduling_queue.pop();
						}
						else
						{
							++ray_state.stack_size;
						}
					}
					else                   
					{
						if(try_queue_nodes(ray_id, entry.data.fst_chld_ind, entry.data.lst_chld_ofst + 1))
						{
							_ray_scheduling_queue.pop();
						}
						else
						{
							++ray_state.stack_size;
						}
					}
				}
			}
			else
			{
				//stack empty or anyhit found return the hit
				_ray_return_queue.push(ray_id);
				_ray_scheduling_queue.pop();
			}
		}


		//Simualte intersectors
		if(!_node_isect_queue.empty() && _box_pipline.is_write_valid())
		{
			uint buffer_id = _node_isect_queue.front();
			NodeStagingBuffer& buffer = _node_staging_buffers[buffer_id];

			if(!buffer.rays.empty())
			{
				uint ray_id = buffer.rays.front();
				RayState& ray_state = _ray_states[ray_id];

				rtm::Ray& ray = ray_state.ray;
				rtm::vec3& inv_d = ray_state.inv_d;
				rtm::Hit& hit = ray_state.hit;

				uint temp_stack_size = ray_state.stack_size;
				for(uint i = 0; i < buffer.num_entry; ++i)
				{
					float t = rtm::intersect(buffer.nodes[i].aabb, ray, inv_d);
					if(t < hit.t) //push cull
					{
						//insertion sort
						uint index = ray_state.stack_size++;
						for(; index > temp_stack_size; --index)
						{
							if(ray_state.stack[index - 1].t < t) break;
							ray_state.stack[index] = ray_state.stack[index - 1];
						}
						ray_state.stack[index] = {t, buffer.nodes[i].data};
					}
				}

				buffer.rays.pop();
				_box_pipline.write(ray_id);
			}

			if(buffer.rays.empty())
			{
				//if all rays are drained from the buffer then free it
				_node_staging_buffer_map.erase(buffer.first_id);
				_free_node_staging_buffers.insert(buffer_id);
				_node_isect_queue.pop();
			}
		}

		_box_pipline.clock();

		if(_box_pipline.is_read_valid())
		{
			uint ray_id = _box_pipline.read();
			if(ray_id != ~0u)
				_ray_scheduling_queue.push(ray_id);
		}

		if(!_tri_isect_queue.empty() && _tri_pipline.is_write_valid())
		{
			uint buffer_id = _tri_isect_queue.front();
			TriStagingBuffer& buffer = _tri_staging_buffers[buffer_id];

			if(!buffer.rays.empty())
			{
				uint ray_id = buffer.rays.front();
				RayState& ray_state = _ray_states[ray_id];

				rtm::Ray& ray = ray_state.ray;
				rtm::vec3& inv_d = ray_state.inv_d;
				rtm::Hit& hit = ray_state.hit;

				uint current_tri = ray_state.current_entry++;

				if(rtm::intersect(buffer.tris[current_tri], ray, hit))
					hit.id = buffer.first_id + current_tri;

				//Only the last box triggers stack pop
				if(ray_state.current_entry == buffer.num_entry)
				{
					ray_state.current_entry = 0;
					buffer.rays.pop();
					_tri_pipline.write(ray_id);
				}
				else _tri_pipline.write(~0);
			}

			if(buffer.rays.empty())
			{
				//if all rays are drained from the buffer then free it
				_tri_staging_buffer_map.erase(buffer.first_id);
				_free_tri_staging_buffers.insert(buffer_id);
				_tri_isect_queue.pop();
			}
		}

		_tri_pipline.clock();

		if(_tri_pipline.is_read_valid())
		{
			uint ray_id = _tri_pipline.read();
			if(ray_id != ~0u)
				_ray_scheduling_queue.push(ray_id);
		}
	}

	void clock_fall() override
	{
		//issue requests
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


		//issue returns
		if(!_ray_return_queue.empty())
		{
			uint ray_id = _ray_return_queue.front();
			const RayState& ray_state = _ray_states[ray_id];

			if(_return_network.is_write_valid(ray_state.port))
			{
				//fetch the next block
				MemoryReturn ret;
				ret.size = sizeof(rtm::Hit);
				ret.dst = ray_state.dst;
				ret.port = ray_state.port;
				ret.paddr = 0xdeadbeefull;
				std::memcpy(ret.data, &ray_state.hit, sizeof(rtm::Hit));
				_return_network.write(ret, ret.port);

				_free_ray_ids.insert(ray_id);
				_ray_return_queue.pop();
			}
		}

		_return_network.clock();
	}

	bool request_port_write_valid(uint port_index) override
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request) override
	{
		_request_network.write(request, request.port);
	}

	bool return_port_read_valid(uint port_index) override
	{
		return _return_network.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index) override
	{
		return _return_network.peek(port_index);
	}

	const MemoryReturn read_return(uint port_index) override
	{
		return _return_network.read(port_index);
	}
};

}}