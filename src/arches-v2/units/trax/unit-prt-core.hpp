#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"

//#define ENABLE_RT_DEBUG_PRINTS (unit_id == 12 && ray_id == 0)

#ifndef ENABLE_RT_DEBUG_PRINTS 
#define ENABLE_RT_DEBUG_PRINTS (false)
#endif

#define PACKET_SIZE

namespace Arches { namespace Units { namespace TRaX {

class UnitPRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint max_rays;
		uint num_tp;

		paddr_t node_base_addr;
		paddr_t tri_base_addr;

		rtm::Frustum frustum;

		UnitMemoryBase* cache;
	};

private:
	struct RayState
	{
		struct StackEntry
		{
			float t;
			rtm::PackedBVH2::NodePack::Data data;
			uint64_t mask;
		};

		enum class Phase
		{
			NONE,
			SCHEDULER,
			HIT_RETURN,
			NODE_FETCH,
			TRI_FETCH,
			NODE_ISECT,
			TRI_ISECT,
			NUM_PHASES,
		}
		phase;

		rtm::Frustum sub_frustum;

		rtm::Hit hit_buffer[16];

		StackEntry stack[32];
		uint8_t stack_size;
		uint8_t current_entry;
		uint16_t flags;
	};

	rtm::Frustum frustum;

	struct NodeStagingBuffer
	{
		rtm::PackedBVH2::NodePack node;
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
	uint last_ray_id{0};

public:
	UnitPRTCore(const Configuration& config);

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

		for(uint i = 0; i < 1; ++i) //n pops per cycle. In reality this would need to be multi banked
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
				"HIT_RETURN",
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