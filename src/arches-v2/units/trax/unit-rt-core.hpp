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

class UnitRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint max_rays;
		uint num_tp;

		paddr_t node_base_addr;
		paddr_t tri_base_addr;

		UnitMemoryBase* cache;
	};

private:
	struct RayState
	{
		struct StackEntry
		{
			float t;
			rtm::PackedBVH2::Node::Data data;
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
		rtm::PackedBVH2::Node node;
		uint16_t ray_id;

		NodeStagingBuffer() {};
	};

	struct TriStagingBuffer
	{
		rtm::Triangle tri;
		uint32_t tri_id;
		uint16_t bytes_filled;

		TriStagingBuffer() {};
	};

	struct FetchItem
	{
		paddr_t addr;
		uint8_t size;
		uint16_t dst;
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;
	UnitMemoryBase* _cache;

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
	paddr_t _node_base_addr;
	paddr_t _tri_base_addr;

public:
	UnitRTCore(const Configuration& config);

	void clock_rise() override
	{
		_request_network.clock();
		_read_requests();
		_read_returns();
		for(uint i = 0; i < 2; ++i) //2 pops per cycle. In reality this would need to be multi banked
			_schedule_ray();
		_simualte_intersectors();
	}

	void clock_fall() override
	{
		_issue_requests();
		_issue_returns();
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

private:
	paddr_t _block_address(paddr_t addr)
	{
		return (addr >> log2i(CACHE_BLOCK_SIZE)) << log2i(CACHE_BLOCK_SIZE);
	}

	bool _try_queue_node(uint ray_id, uint node_id);
	bool _try_queue_tri(uint ray_id, uint tri_id);

	void _read_requests();
	void _read_returns();
	void _schedule_ray();
	void _simualte_intersectors();

	void _issue_requests();
	void _issue_returns();

public:
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