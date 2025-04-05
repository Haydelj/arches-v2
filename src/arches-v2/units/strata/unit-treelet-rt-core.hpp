#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"
#include "unit-ray-stream-buffer.hpp"

#include "strata-kernel/include.hpp"

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 12 && ray_id == 0)
//#define ENABLE_RT_DEBUG_PRINTS (ray_state.ray_data.global_ray_id == 9416)
//#define ENABLE_RT_DEBUG_PRINTS (_tm_index == 0)
#define ENABLE_HIT_DEBUG_PRINTS (false)
#define ENABLE_REQUEST_DEBUG_PRINTS (false)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

#ifdef USE_COMPRESSED_WIDE_BVH
typedef rtm::CompressedWideTreeletBVHSTRaTA::Treelet TREELET;
#else
typedef rtm::WideTreeletBVHSTRaTA::Treelet TREELET;
#endif

namespace Arches { namespace Units { namespace STRaTA {

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
	struct RayState
	{
		struct StackEntry
		{
			float t;
			uint32_t treelet;
			rtm::WideTreeletBVHSTRaTA::Treelet::Node::Data data;
			rtm::WideTreeletBVHSTRaTA::Treelet::Node::ParentData parent_data;
		};

		enum class Phase
		{
			NONE,
			SCHEDULER,
			RAY_FETCH,
			HIT_UPDATE,
			NODE_FETCH,
			TRI_FETCH,
			NODE_ISECT,
			TRI_ISECT,
			NUM_PHASES,
		}
		phase;

		STRaTAKernel::RayData ray_data;
		rtm::vec3 inv_d;

		StackEntry stack;

		uint16_t flags;
		uint16_t port;
		uint16_t dst;

	};

	struct NodeStagingBuffer
	{
		TREELET::Node node;
		
		paddr_t addr;
		uint16_t bytes_filled;

		NodeStagingBuffer() {};
	};

	struct TriStagingBuffer
	{
		TREELET::Triangle tris[3];
		paddr_t addr;
		uint16_t num_tris;
		uint16_t bytes_filled;

		TriStagingBuffer() {};
	};

	struct FetchItem
	{
		paddr_t addr;
		uint8_t size;
		BitStack58 dst;
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;
	UnitMemoryBase* _cache;
	UnitRayStreamBuffer* _ray_stream_buffer;

	//ray scheduling hardware
	std::queue<uint> _ray_scheduling_queue;
	std::queue<uint> _ray_data_load_queue;
	std::queue<FetchItem> _fetch_queue;
	std::queue<STRaTAKernel::RayData> _ray_buffer_store_queue;
	std::vector<RayState> _ray_states;

	// hit record loading
	std::queue<MemoryRequest> _tp_hit_load_queue;
	std::map<paddr_t, uint16_t> _hit_return_port_map;
	std::queue<MemoryReturn> _tp_hit_return_queue;
	std::queue<uint> _hit_store_queue;

	//node pipline
	std::vector<NodeStagingBuffer> _node_staging_buffers;
	std::queue<uint> _node_isect_queue;
	std::vector<NodeStagingBuffer> _leaf_isect_buffers;
	LatencyFIFO<uint> _box_pipline;

	//tri pipline
	std::vector<TriStagingBuffer> _tri_staging_buffers;
	std::queue<uint> _tri_isect_queue;
	LatencyFIFO<uint> _tri_pipline;
	uint _tri_isect_index{0};

	//meta data
	uint _max_rays;
	uint _num_tp;
	uint _tm_index;
	paddr_t _treelet_base_addr;
	paddr_t _hit_record_base_addr;
	uint last_ray_id{0};

public:
	UnitTreeletRTCore(const Configuration& config);

	void clock_rise() override
	{
		_request_network.clock();
		_read_requests();
		_read_returns();

		if(_ray_scheduling_queue.empty())
		{
			for(uint i = 0; i < _ray_states.size(); ++i)
			{
				uint phase = (uint)_ray_states[last_ray_id].phase;
				if(++last_ray_id == _ray_states.size()) last_ray_id = 0;
				if(phase != 0)
				{
					log.stall_counters[phase]++;
					break;
				}
			}
		}

		//for(uint i = 0; i < 2; ++i) //2 pops per cycle. In reality this would need to be multi banked
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
				uint64_t hits;
				uint64_t get_hits;
				uint64_t load_hits;
				uint64_t return_hits;
				uint64_t store_rays;
				uint64_t stall_counters[(uint)RayState::Phase::NUM_PHASES];
			};
			uint64_t counters[16];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < 16; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < 16; ++i)
				counters[i] += other.counters[i];
		}

		void print(cycles_t cycles, uint num_units = 1)
		{
			const static std::string phase_names[] =
			{
				"NONE",
				"SCHEDULER",
				"RAY_FETCH",
				"HIT_UPDATE",
				"NODE_FETCH",
				"TRI_FETCH",
				"NODE_ISECT",
				"TRI_ISECT",
				"NUM_PHASES",
			};

			printf("Nodes/Ray: %.2f\n", (double)nodes / rays);
			printf("Tris/Ray: %.2f\n", (double)tris / rays);
			printf("Nodes/Tri: %.2f\n", (double)nodes / tris);

			uint64_t total = 0;
			std::vector<std::pair<const char*, uint64_t>> _data_stall_counter_pairs;
			for(uint i = 0; i < (uint)RayState::Phase::NUM_PHASES; ++i)
			{
				total += stall_counters[i];
				_data_stall_counter_pairs.push_back({phase_names[i].c_str(), stall_counters[i]});
			}
			std::sort(_data_stall_counter_pairs.begin(), _data_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			printf("\nStall Cycles: %lld (%.2f%%)\n", total / num_units, 100.0f * total / num_units / cycles);
			for(uint i = 0; i < _data_stall_counter_pairs.size(); ++i)
				if(_data_stall_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _data_stall_counter_pairs[i].first, _data_stall_counter_pairs[i].second / num_units, 100.0 * _data_stall_counter_pairs[i].second / num_units / cycles);
		};
	}log;
};

}}}