#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"

#include "strata-rt-kernel/ray-data.hpp"
#include "unit-ray-stream-buffer.hpp"

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 8 && ray_id == 0)
//#define ENABLE_RT_DEBUG_PRINTS (ray_state.ray_data.global_ray_id == 314159)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

namespace Arches { namespace Units { namespace STRaTART {

using Treelet = rtm::CompressedWideTreeletBVH::Treelet;

class UnitTreeletRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint max_rays;
		uint num_tp;
		uint tm_index;

		paddr_t treelet_base_addr;
		paddr_t hit_record_base_addr;

		UnitMemoryBase* cache;
		UnitRayStreamBuffer* ray_stream_buffer;
	};

private:
	enum Phase : uint
	{
		SCHEDULER,
		RAY_FETCH,
		NODE_FETCH,
		TRI_FETCH,
		NODE_ISECT,
		TRI_ISECT,
		HIT_UPDATE,
		NUM_PHASES,
	};

	enum IssueType : uint
	{
		NODE_ISSUE,
		TRI_ISSUE,
		RAY_STORE,
		RESTART,
		POP_CULL,
		RAY_COMPLETE,
		NUM_TYPES,
	};

	enum Destination : uint
	{
		NODE,
		TRI,
		RAY,
		TP
	};

	struct StagingBuffer
	{
		paddr_t address;
		uint bytes_filled;

		union
		{
			uint8_t data[1];
			Treelet::Node node;
			struct
			{
				Treelet::Triangle tris[3];
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

	struct StackEntry
	{
		float t;
		bool is_last;
		rtm::WideTreeletBVH::Treelet::Node::Data data;
	};

	struct RayState
	{
		STRaTARTKernel::RayData ray_data;
		rtm::vec3 inv_d;

		bool update_restart_trail;
		StackEntry stack[32 * (rtm::CompressedWideTreeletBVH::WIDTH - 1)];
		uint stack_size;

		Phase phase;

		StagingBuffer buffer;


		RayState() : phase(RAY_FETCH) {};

		RayState(const STRaTARTKernel::RayData& ray_data) : ray_data(ray_data)
		{
			inv_d = rtm::vec3(1.0f) / ray_data.ray.d;
			stack[0].t = ray_data.ray.t_min;
			stack[0].data.is_int = 1;
			stack[0].data.is_child_treelet = 0;
			stack[0].data.child_index = 0;
			stack_size = 1;
			update_restart_trail = false;
		}
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;
	UnitMemoryBase* _cache;
	UnitRayStreamBuffer* _ray_stream_buffer;

	//ray scheduling hardware
	std::vector<RayState> _ray_states;
	std::queue<uint> _ray_scheduling_queue;
	std::queue<uint> _ray_data_load_queue;
	std::queue<MemoryRequest> _cache_fetch_queue;
	std::queue<STRaTARTKernel::RayData> _ray_buffer_store_queue;

	// hit record loading
	std::queue<MemoryRequest> _tp_hit_load_queue;
	std::queue<MemoryReturn> _tp_hit_return_queue;

	//node pipline
	std::queue<uint> _node_isect_queue;
	Pipline<uint> _box_pipline;
	uint _boxes_issued{0};

	//tri pipline
	std::queue<uint> _tri_isect_queue;
	Pipline<uint> _tri_pipline;
	uint _tris_issued{0};

	//meta data
	uint _max_rays;
	uint _num_tp;
	uint _tm_index;
	paddr_t _treelet_base_addr;
	paddr_t _hit_record_base_addr;
	uint _last_ray_id{0};

public:
	UnitTreeletRTCore(const Configuration& config);

	void clock_rise() override
	{
		_request_network.clock();
		_read_requests();
		_read_returns();

		if(_ray_scheduling_queue.empty())
		{
			uint phase = (uint)_ray_states[_last_ray_id].phase;
			if(++_last_ray_id == _ray_states.size()) _last_ray_id = 0;
			log.stall_counters[phase]++;
		}

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

	bool _try_queue_node(uint ray_id, uint treelet_id, uint node_id);
	bool _try_queue_tri(uint ray_id, uint treelet_id, uint tri_offset, uint num_tris);
	bool _try_queue_prefetch(paddr_t addr, uint size, uint cache_mask);

	void _read_requests();
	void _read_returns();
	void _schedule_ray();
	void _simualte_intersectors();

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
				uint64_t issue_counters[IssueType::NUM_TYPES];
				uint64_t stall_counters[Phase::NUM_PHASES];
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

		void print(uint num_units = 1)
		{
			const static std::string phase_names[] =
			{
				"SCHEDULER",
				"RAY_FETCH",
				"NODE_FETCH",
				"TRI_FETCH",
				"NODE_ISECT",
				"TRI_ISECT",
				"HIT_UPDATE",
				"NUM_PHASES",
			};

			const static std::string issue_names[] =
			{
				"NODE_ISSUE",
				"TRI_ISSUE",
				"RAY_STORE",
				"RESTART",
				"POP_CULL",
				"RAY_COMPLETE",
				"NUM_TYPES",
			};

			printf("Rays: %lld\n", rays / num_units);
			printf("Nodes: %lld\n", nodes / num_units);
			printf("Tris: %lld\n", tris / num_units);
			printf("\n");
			printf("Nodes/Ray: %.2f\n", (double)nodes / rays);
			printf("Tris/Ray: %.2f\n", (double)tris / rays);
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
			for(uint i = 0; i < Phase::NUM_PHASES; ++i)
			{
				stall_total += stall_counters[i];
				_data_stall_counter_pairs.push_back({phase_names[i].c_str(), stall_counters[i]});
			}
			std::sort(_data_stall_counter_pairs.begin(), _data_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t total = stall_total + issue_total;

			printf("\nIssue Cycles: %lld (%.2f%%)\n", issue_total / num_units, 100.0f * issue_total / total);
			for(uint i = 0; i < _issue_counter_pairs.size(); ++i)
				if(_issue_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _issue_counter_pairs[i].first, _issue_counter_pairs[i].second / num_units, 100.0 * _issue_counter_pairs[i].second / total);

			printf("\nStall Cycles: %lld (%.2f%%)\n", stall_total / num_units, 100.0f * stall_total / total);
			for(uint i = 0; i < _data_stall_counter_pairs.size(); ++i)
				if(_data_stall_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _data_stall_counter_pairs[i].first, _data_stall_counter_pairs[i].second / num_units, 100.0 * _data_stall_counter_pairs[i].second / total);
		};
	}log;
};

}}}