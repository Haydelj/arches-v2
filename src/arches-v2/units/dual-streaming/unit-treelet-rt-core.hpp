#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-memory-base.hpp"
#include "unit-ray-staging-buffer.hpp"

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 12 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

namespace Arches { namespace Units { namespace DualStreaming {

class UnitTreeletRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint max_rays;
		uint num_tp;

		paddr_t treelet_base_addr;
		paddr_t hit_record_base_addr;

		bool use_early_termination;

		UnitRayStagingBuffer* rsb;
		UnitMemoryBase* cache;
	};

private:
	struct RayState
	{
		struct NodeStackEntry
		{
			float t;
			rtm::PackedTreelet::Node::Data data;
		};

		struct TreeletStackEntry
		{
			float t;
			uint index;
		};

		rtm::Ray ray;
		uint32_t global_ray_id;
		uint32_t treelet_id;
		rtm::vec3 inv_d;

		rtm::Hit hit;
		bool hit_found;
		bool lhit_returned;

		NodeStackEntry nstack[16];
		uint nstack_size;

		TreeletStackEntry tqueue[16];
		uint tqueue_tail;
		uint tqueue_head;

		uint order_hint;

		RayState() = default;

		RayState(const WorkItem& wi)
		{
			ray = wi.bray.ray;
			global_ray_id = wi.bray.id;
			treelet_id = wi.segment_id;
			inv_d = rtm::vec3(1.0f) / ray.d;
			hit.t = ray.t_max;
			hit.bc = rtm::vec2(0.0f);
			hit.id = ~0u;
			hit_found = false;
			lhit_returned = false;
			nstack_size = 1;
			nstack[0].t = ray.t_min;
			nstack[0].data.is_leaf = 0;
			nstack[0].data.is_child_treelet = 0;
			nstack[0].data.child_index = 0;
			tqueue_head = 0;
			tqueue_tail = 0;
			order_hint = 0;
		}
	};

	struct NodeStagingBuffer
	{
		rtm::PackedTreelet::Node node;
		uint16_t ray_id;
	};

	struct TriStagingBuffer
	{
		rtm::PackedTreelet::Triangle tri;
		paddr_t addr;
		uint16_t bytes_filled;
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
	UnitRayStagingBuffer* _rsb;
	UnitMemoryBase* _cache;

	//ray scheduling hardware
	std::vector<RayState> _ray_states;
	std::queue<uint> _ray_scheduling_queue;
	std::queue<uint> _hit_load_queue;
	std::queue<uint> _hit_store_queue;
	std::queue<uint> _work_item_load_queue;
	std::queue<WorkItem> _work_item_store_queue;
	std::queue<FetchItem> _fetch_queue;

	//hit record loading
	std::queue<MemoryRequest> _tp_hit_load_queue;
	std::map<paddr_t, uint16_t> _hit_return_port_map;
	std::queue<MemoryReturn> _hit_return_queue;
	uint _active_ray_slots;

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
	paddr_t _hit_record_base_addr;
	bool _use_early_termination;

public:
	UnitTreeletRTCore(const Configuration& config);

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
	paddr_t block_address(paddr_t addr)
	{
		return (addr >> log2i(CACHE_BLOCK_SIZE)) << log2i(CACHE_BLOCK_SIZE);
	}

	bool _try_queue_node(uint ray_id, uint treelet_id, uint node_id);
	bool _try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset);

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