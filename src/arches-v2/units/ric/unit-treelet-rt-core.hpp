#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-memory-base.hpp"
#include "unit-ray-coalescer.hpp"
#include "units/dual-streaming/unit-hit-record-updater.hpp"

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 10 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

namespace Arches { namespace Units { namespace RIC {

template<typename TT>
class UnitTreeletRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		bool early_t{false};
		uint max_rays;
		uint num_tp;
		uint rtc_index;

		paddr_t treelet_base_addr;
		paddr_t ray_state_base_addr;

		uint cache_port;
		UnitMemoryBase* cache;
		UnitRayCoalescer* ray_coalescer;
		DualStreaming::UnitHitRecordUpdater* hit_record_updater;
	};

private:
	enum class IssueType
	{
		NODE_FETCH,
		TRI_FETCH,
		POP_CULL,
		STORE_WORK_ITEM,
		RETIRE_RAY,
		NUM_TYPES,
	};

	enum class Destination
	{
		NODE,
		TRI,
		RAY,
		TP
	};

	struct NodeStackEntry
	{
		float t;
		rtm::WideTreeletBVH::Treelet::Node::Data data;
	};

	struct TreeletStackEntry
	{
		float t;
		uint index;
	};

	struct StagingBuffer
	{
		paddr_t address;
		uint bytes_filled;

		union
		{
			uint8_t data[1];
			TT::Node node;
			struct
			{
				TT::Triangle tris[3];
				uint num_tris;
			};
		};

		StagingBuffer() {}
		StagingBuffer& operator=(const StagingBuffer& other)
		{
			std::memcpy(this, &other, sizeof(StagingBuffer));
			return *this;
		}
	};

	struct RayState
	{
		enum class Phase
		{
			RAY_INDEX_FETCH,
			RAY_FETCH,
			HIT_FETCH,
			NODE_FETCH,
			TRI_FETCH,
			SCHEDULER,
			NODE_ISECT,
			TRI_ISECT,
			HIT_UPDATE,
			NUM_PHASES,
		}
		phase;

		rtm::Ray ray;
		uint32_t global_ray_id;
		uint32_t treelet_id;
		rtm::vec3 inv_d;

		rtm::Hit hit;
		bool hit_found;
		bool lhit_returned;

		NodeStackEntry nstack[32 * (rtm::WideTreeletBVH::WIDTH - 1)];
		uint nstack_size;

		uint order_hint;
		StagingBuffer buffer;

		uint steps;

		RayState() : phase(Phase::RAY_FETCH) {};

		RayState(const UnitRayCoalescer::WorkItem& wi)
		{
			global_ray_id = wi.ray_id;
			treelet_id = wi.segment_id;
			hit.bc = rtm::vec2(0.0f);
			hit.id = ~0u;
			hit_found = false;
			lhit_returned = false;
			nstack_size = 1;
			nstack[0].t = ray.t_min;
			nstack[0].data.is_int = 1;
			nstack[0].data.is_child_treelet = 0;
			nstack[0].data.child_index = 0;
			order_hint = 0;
			steps = 0;
		}
	};

	struct SegmentState
	{
		uint rays{0};
		uint buckets{0};
	};

	struct RayBucketBuffer
	{
		bool busy{false};
		uint bytes_filled{0};

		union
		{
			uint8_t data[1];
			RayBucket bucket;
		};

		RayBucketBuffer() {}
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;

	//memory units
	uint _cache_port;
	UnitMemoryBase* _cache;
	UnitRayCoalescer* _ray_coalescer;

	//ray scheduling hardware
	std::map<uint, SegmentState> _segment_states;
	RayBucketBuffer _ray_bucket_buffer;
	std::queue<UnitRayCoalescer::WorkItem> _work_item_return_queue;
	std::queue<UnitRayCoalescer::WorkItem> _work_item_store_queue;

	std::set<uint> _free_ray_ids;
	std::vector<RayState> _ray_states;
	std::deque<uint> _ray_scheduling_queue;
	std::queue<MemoryRequest> _cache_fetch_queue;
	std::queue<uint> _completed_buckets;

	//hit record loading
	std::queue<MemoryRequest> _tp_hit_load_queue;
	std::queue<MemoryReturn> _tp_hit_return_queue;

	//node pipline
	std::queue<uint> _node_isect_queue;
	Pipline<uint> _box_pipline;

	//tri pipline
	std::queue<uint> _tri_isect_queue;
	Pipline<uint> _tri_pipline;
	uint _tri_isect_index{0};

	//meta data
	bool _block_for_traversal{true};
	bool _early_t;
	uint _max_rays;
	uint _ray_id_bits;
	uint _num_tp;
	uint _rtc_index;
	paddr_t _treelet_base_addr;
	paddr_t _ray_state_base_addr;
	uint _last_ray_id{0};

public:
	UnitTreeletRTCore(const Configuration& config);

	void clock_rise() override
	{
		_request_network.clock();
		_read_requests();
		_read_returns();
		_init_ray();

		for(uint i = 0; i < 1; ++i) _schedule_ray(); //n pops per cycle. In reality this would need to be multi banked

		_simualte_node_pipline();
		_simualte_tri_pipline();
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

	bool _try_queue_node(uint ray_id, uint treelet_id, uint node_id);
	bool _try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset, uint num_tris);
	bool _try_queue_ray_load(uint ray_id);
	bool _try_queue_cshit(uint ray_id);
	bool _try_queue_prefetch(uint treelet_id);

	void _read_requests();
	void _read_returns();
	void _init_ray();
	void _schedule_ray();
	void _simualte_node_pipline();
	void _simualte_tri_pipline();

	void _issue_requests();
	void _issue_returns();

public:
	class Log
	{
	private:
		constexpr static uint NUM_COUNTERS = 24;

	public:
		union
		{
			struct
			{
				uint64_t rays;
				uint64_t nodes;
				uint64_t tris;
				uint64_t hits;
				uint64_t issue_counters[(uint)IssueType::NUM_TYPES];
				uint64_t stall_counters[(uint)RayState::Phase::NUM_PHASES];
			};
			uint64_t counters[NUM_COUNTERS];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];
		}

		void print(cycles_t cycles, uint num_units = 1)
		{
			const static std::string phase_names[] =
			{
				"RAY_INDEX_FETCH",
				"RAY_FETCH",
				"HIT_FETCH",
				"NODE_FETCH",
				"TRI_FETCH",
				"SCHEDULER",
				"NODE_ISECT",
				"TRI_ISECT",
				"HIT_UPDATE",
				"NUM_PHASES"
			};

			const static std::string issue_names[] =
			{
				"NODE_FETCH",
				"TRI_FETCH",
				"POP_CULL",
				"STORE_WORK_ITEM",
				"RETIRE_RAY",
				"NUM_TYPES",
			};

			printf("Rays: %lld\n", rays / num_units);
			printf("Nodes: %lld\n", nodes / num_units);
			printf("Tris: %lld\n", tris / num_units);
			printf("Hits: %lld\n", hits / num_units);
			printf("\n");
			printf("Nodes/Ray: %.2f\n", (double)nodes / rays);
			printf("Tris/Ray: %.2f\n", (double)tris / rays);
			printf("Hits/Ray: %.2f\n", (double)hits / rays);
			printf("Nodes/Tri: %.2f\n", (double)nodes / tris);

			uint64_t issue_total = 0;
			std::vector<std::pair<const char*, uint64_t>> _issue_counter_pairs;
			for(uint i = 0; i < (uint)IssueType::NUM_TYPES; ++i)
			{
				issue_total += issue_counters[i];
				_issue_counter_pairs.push_back({issue_names[i].c_str(), issue_counters[i]});
			}
			std::sort(_issue_counter_pairs.begin(), _issue_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t stall_total = 0;
			std::vector<std::pair<const char*, uint64_t>> _data_stall_counter_pairs;
			for(uint i = 0; i < (uint)RayState::Phase::NUM_PHASES; ++i)
			{
				stall_total += stall_counters[i];
				_data_stall_counter_pairs.push_back({phase_names[i].c_str(), stall_counters[i]});
			}
			std::sort(_data_stall_counter_pairs.begin(), _data_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t total = stall_total + issue_total;

			printf("\nIssue Cycles: %lld (%.2f%%)\n", issue_total / num_units, 100.0f * issue_total / total);
			for(uint i = 0; i < _issue_counter_pairs.size(); ++i)
				if(_issue_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _issue_counter_pairs[i].first, _issue_counter_pairs[i].second / num_units, 100.0 * _issue_counter_pairs[i].second / num_units / cycles);

			printf("\nStall Cycles: %lld (%.2f%%)\n", stall_total / num_units, 100.0f * stall_total / total);
			for(uint i = 0; i < _data_stall_counter_pairs.size(); ++i)
				if(_data_stall_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _data_stall_counter_pairs[i].first, _data_stall_counter_pairs[i].second / num_units, 100.0 * _data_stall_counter_pairs[i].second / num_units / cycles);
		};
	}log;
};

}}}