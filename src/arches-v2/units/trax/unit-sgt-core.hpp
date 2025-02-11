#pragma once
#include "stdafx.hpp"
#include "rtm/rtm.hpp"

#include "sg-kernel/include.hpp"
#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"

namespace Arches { namespace Units { namespace TRaXSG {

template<typename NT>
class UnitRTCore : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint num_clients;
		uint max_rays;
		paddr_t node_base_addr;
		paddr_t sg_base_addr;

		UnitMemoryBase* cache;
		uint cache_port;
	};

private:
	enum class IssueType
	{
		NODE_FETCH,
		SG_FETCH,
		POP_CULL,
		HIT_RETURN,
		NUM_TYPES,
	};

	struct StackEntry
	{
		float t;
		rtm::WideBVH::Node::Data data;

		StackEntry() {}
	};

	struct StagingBuffer
	{
		paddr_t address;
		uint bytes_filled;
		uint type;

		union
		{
			uint8_t data[1];
			NT::Node node;
			struct
			{
				rtm::SphericalGaussian sgs[3];
				uint num_sgs;
				uint sg_id;
			};
		};

		StagingBuffer() {}
	};

	struct RayState
	{
		enum class Phase
		{
			RAY_FETCH,
			SCHEDULER,
			HIT_RETURN,
			NODE_FETCH,
			SG_FETCH,
			NODE_ISECT,
			TRI_ISECT,
			NUM_PHASES,
		}
		phase;

		rtm::Ray ray;
		rtm::vec3 inv_d;
		SGKernel::HitPacket hit_packet;
		float opacity;
		float opacity0;
		float ts[SGKernel::HitPacket::MAX_HITS];

		StackEntry stack[32 * NT::WIDTH];
		uint8_t stack_size;
		uint8_t current_entry;

		MemoryRequest::Flags flags;
		BitStack27 dst;

		StagingBuffer buffer;

		RayState() {};
	};

	struct FetchItem
	{
		paddr_t addr;
		uint8_t size;
		uint16_t ray_id;
	};

	//interconnects
	RequestCascade _request_network;
	ReturnCascade _return_network;
	UnitMemoryBase* _cache;
	uint _cache_port;

	//ray scheduling hardware
	std::queue<uint> _ray_scheduling_queue;
	std::queue<uint> _ray_return_queue;
	std::queue<FetchItem> _fetch_queue;

	std::set<uint> _free_ray_ids;
	std::vector<RayState> _ray_states;

	//node pipline
	std::queue<uint> _node_isect_queue;
	Pipline<uint> _box_pipline;
	uint _box_issue_count{0};

	//tri pipline
	std::queue<uint> _sg_isect_queue;
	Pipline<uint> _sg_pipline;
	uint _sg_issue_count{0};

	//meta data
	uint _max_rays;
	paddr_t _node_base_addr;
	paddr_t _sg_base_addr;
	uint _last_ray_id{0};

public:
	UnitRTCore(const Configuration& config);

	void clock_rise() override;

	void clock_fall() override;

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
	bool _try_queue_sgs(uint ray_id, uint tri_id, uint num_tris);

	void _read_requests();
	void _read_returns();
	void _schedule_ray();
	void _simualte_node_pipline();
	void _simualte_sg_pipline();

	void _issue_requests();
	void _issue_returns();

public:
	class Log
	{
	private:
		constexpr static uint NUM_COUNTERS = 16;

	public:
		union
		{
			struct
			{
				uint64_t rays;
				uint64_t nodes;
				uint64_t tris;
				uint64_t hits_returned;
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

		void print(uint num_units = 1)
		{
			const static std::string phase_names[] =
			{
				"RAY_FETCH",
				"SCHEDULER",
				"HIT_RETURN",
				"NODE_FETCH",
				"SG_FETCH",
				"NODE_ISECT",
				"TRI_ISECT",
				"NUM_PHASES",
			};

			const static std::string issue_names[] =
			{
				"NODE_FETCH",
				"SG_FETCH",
				"POP_CULL",
				"HIT_RETURN",
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
				if(_issue_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _issue_counter_pairs[i].first, _issue_counter_pairs[i].second / num_units, 100.0 * _issue_counter_pairs[i].second / total);

			printf("\nStall Cycles: %lld (%.2f%%)\n", stall_total / num_units, 100.0f * stall_total / total);
			for(uint i = 0; i < _data_stall_counter_pairs.size(); ++i)
				if(_data_stall_counter_pairs[i].second) printf("\t%s: %lld (%.2f%%)\n", _data_stall_counter_pairs[i].first, _data_stall_counter_pairs[i].second / num_units, 100.0 * _data_stall_counter_pairs[i].second / total);
		};
	}log;
};

}}}