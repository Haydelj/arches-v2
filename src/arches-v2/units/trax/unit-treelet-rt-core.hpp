#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 12 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

namespace Arches { namespace Units { namespace TRaX {

class UnitTreeletRTCore : public UnitMemoryBase
{
private:
	Cascade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;
	UnitMemoryBase*         _cache;

	struct RayState
	{
		struct StackEntry
		{
			float t;
			rtm::PackedTreelet::Node::Data data;
		};

		rtm::Ray ray;
		rtm::vec3 inv_d;

		rtm::Hit hit;

		StackEntry nstack[32];
		uint16_t nstack_size;

		uint32_t tstack[256];
		uint16_t tstack_size;
		uint16_t pre_tstack_size;
		uint32_t treelet_id;

		uint16_t flags;
		uint16_t port;
		uint16_t dst;
	};

	struct NodeStagingBuffer
	{
		rtm::PackedTreelet::Node node;
		uint16_t ray_id;

		NodeStagingBuffer() {};
	};

	struct TriStagingBuffer
	{
		rtm::PackedTreelet::Triangle tri;
		paddr_t addr;
		uint16_t bytes_filled;

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
	std::queue<NodeStagingBuffer> _node_isect_queue;
	Pipline<uint> _box_pipline;

	//tri pipline
	std::vector<TriStagingBuffer> _tri_staging_buffers;
	std::queue<uint> _tri_isect_queue;
	Pipline<uint> _tri_pipline;

	//meta data
	uint _max_rays;
	uint _num_tp;
	paddr_t _treelet_base_addr;

public:
	UnitTreeletRTCore(uint max_rays, uint num_tp, paddr_t treelet_base_addr, UnitMemoryBase* cache) :
		_max_rays(max_rays), _num_tp(num_tp), _treelet_base_addr(treelet_base_addr),
		_cache(cache), _request_network(num_tp, 1), _return_network(num_tp),
		_box_pipline(3, 1), _tri_pipline(22, 8)
	{
		_tri_staging_buffers.resize(max_rays);
		_ray_states.resize(max_rays);
		for(uint i = 0; i < _ray_states.size(); ++i)
			_free_ray_ids.insert(i);
	}

	paddr_t block_address(paddr_t addr)
	{
		return (addr >> log2i(CACHE_BLOCK_SIZE)) << log2i(CACHE_BLOCK_SIZE);
	}

	bool try_queue_node(uint ray_id, uint treelet_id, uint node_id)
	{
		paddr_t start = (paddr_t)&((rtm::PackedTreelet*)_treelet_base_addr)[treelet_id].nodes[node_id];
		_fetch_queue.push({start, (uint8_t)(sizeof(rtm::PackedTreelet::Node)), (uint16_t)ray_id});
		return true;
	}

	bool try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset)
	{
		paddr_t start = (paddr_t)&((rtm::PackedTreelet*)_treelet_base_addr)[treelet_id].bytes[tri_offset];
		paddr_t end = start + sizeof(rtm::PackedTreelet::Triangle);

		_tri_staging_buffers[ray_id].addr = start;
		_tri_staging_buffers[ray_id].bytes_filled = 0;

		//split request at cache boundries
		//queue the requests to fill the buffer
		paddr_t addr = start;
		while(addr < end)
		{
			paddr_t next_boundry = std::min(end, block_address(addr + CACHE_BLOCK_SIZE));
			uint8_t size = next_boundry - addr;
			_fetch_queue.push({addr, size, (uint16_t)(ray_id | 0x8000u)});
			addr += size;
		}

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
			ray_state.nstack_size = 0;
			ray_state.tstack[0] = 0;
			ray_state.tstack_size = 1;
			ray_state.pre_tstack_size = 1;
			ray_state.flags = request.flags;
			ray_state.dst = request.dst;
			ray_state.port = request.port;

			_ray_scheduling_queue.push(ray_id);

			log.rays++;
		}


		//read returns
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
					_tri_isect_queue.push(ray_id);
			}
			else
			{
				_assert(sizeof(rtm::PackedTreelet::Node) == ret.size);

				NodeStagingBuffer buffer;
				buffer.ray_id = ray_id;
				std::memcpy(&buffer.node, ret.data, ret.size);

				_node_isect_queue.push(buffer);
			}
		}


		for(uint i = 0; i < 2; ++i) //2 pops per cycle. In reality this would need to be multi banked
		{
			//pop a entry from next rays stack and queue it up
			if(!_ray_scheduling_queue.empty())
			{
				uint ray_id = _ray_scheduling_queue.front();
				_ray_scheduling_queue.pop();

				RayState& ray_state = _ray_states[ray_id];
				bool any_hit_found = (ray_state.flags & 0x1) && ray_state.hit.id != ~0u;
				if(any_hit_found) //anyhit found return the hit
				{
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("AH Ret: %d\n", ray_state.hit.id);

					_ray_return_queue.push(ray_id);
				}
				else if(ray_state.nstack_size > 0)
				{
					RayState::StackEntry& entry = ray_state.nstack[ray_state.nstack_size - 1];
					if(entry.t < ray_state.hit.t) //pop cull
					{
						if(entry.data.is_leaf)
						{
							if(try_queue_tri(ray_id, ray_state.treelet_id, entry.data.tri_offset))
							{
								if(ENABLE_RT_DEBUG_PRINTS)
									printf("Pop Tri: %d:%d:%d\n", ray_state.treelet_id, entry.data.tri_offset, entry.data.num_tri + 1);

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
								ray_state.tstack[ray_state.tstack_size++] = entry.data.child_index;
								ray_state.nstack_size--;
								_ray_scheduling_queue.push(ray_id);
							}
							else
							{
								if(try_queue_node(ray_id, ray_state.treelet_id, entry.data.child_index))
								{
									if(ENABLE_RT_DEBUG_PRINTS)
										printf("Pop Node: %d:%d\n", ray_state.treelet_id, entry.data.child_index);

									ray_state.nstack_size--;
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
						if(ENABLE_RT_DEBUG_PRINTS)
							printf("Pop Cull\n");
						ray_state.nstack_size--;
						_ray_scheduling_queue.push(ray_id);
					}
				}
				else if(ray_state.tstack_size > 0)
				{
					//treelets are pushed in nearest first order so we need to flip such that we pop nearest first
					if(ray_state.tstack_size > 0)
					{
						uint treelet_stack_start = ray_state.pre_tstack_size;
						uint treelet_stack_end = ray_state.tstack_size - 1;
						while(treelet_stack_start < treelet_stack_end)
						{
							uint temp = ray_state.tstack[treelet_stack_start];
							ray_state.tstack[treelet_stack_start] = ray_state.tstack[treelet_stack_end];
							ray_state.tstack[treelet_stack_end] = temp;
							treelet_stack_start++;
							treelet_stack_end--;
						}
					}

					ray_state.treelet_id = ray_state.tstack[--ray_state.tstack_size];
					ray_state.pre_tstack_size = ray_state.tstack_size;

					if(ENABLE_RT_DEBUG_PRINTS)
						printf("Pop Treelet: %d\n", ray_state.treelet_id);

					ray_state.nstack_size = 1;
					ray_state.nstack[0].t = ray_state.ray.t_min;
					ray_state.nstack[0].data.is_leaf = 0;
					ray_state.nstack[0].data.is_child_treelet = 0;
					ray_state.nstack[0].data.child_index = 0;

					_ray_scheduling_queue.push(ray_id);
				}
				else //stack empty return the hit
				{
					if(ENABLE_RT_DEBUG_PRINTS)
						printf("CH Ret: %d\n", ray_state.hit.id);

					_ray_return_queue.push(ray_id);
				}
			}
		}


		//Simualte intersectors
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
				_ray_scheduling_queue.push(ray_id);

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
				hit.id = buffer.tri.id;

			_tri_pipline.write(ray_id);
			_tri_isect_queue.pop();
		}

		_tri_pipline.clock();

		if(_tri_pipline.is_read_valid())
		{
			uint ray_id = _tri_pipline.read();
			if(ray_id != ~0u)
				_ray_scheduling_queue.push(ray_id);

			log.tris++;
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

	class Log
	{
	public:
		union
		{
			struct
			{
				uint64_t rays;
				uint64_t nodes;
				uint64_t tris;
			};
			uint64_t counters[8];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < 8; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < 8; ++i)
				counters[i] += other.counters[i];
		}

		void print_log(cycles_t cycles, uint units = 1)
		{
			printf("Nodes/Ray: %.2f\n", (double)nodes / rays);
			printf("Tris/Ray: %.2f\n", (double)tris / rays);
			printf("Nodes/Tri: %.2f\n", (double)nodes / tris);
		};
	}log;
};

}}}