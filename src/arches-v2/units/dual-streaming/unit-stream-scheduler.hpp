#pragma once 
#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-dram.hpp"
#include "units/unit-buffer.hpp"

#include "dual-streaming-kernel/include.hpp"
#include "unit-scene-buffer.hpp"

namespace Arches {
namespace Units {
namespace DualStreaming {

#define RAY_BUCKET_SIZE     (2048)
#define MAX_RAYS_PER_BUCKET ((RAY_BUCKET_SIZE - 16) / sizeof(BucketRay))

struct alignas(RAY_BUCKET_SIZE) RayBucket
{
	paddr_t next_bucket{0};
	uint segment_id{0};
	uint num_rays{0};
	BucketRay bucket_rays[MAX_RAYS_PER_BUCKET];

	bool is_full()
	{
		return num_rays == MAX_RAYS_PER_BUCKET;
	}

	void write_ray(const BucketRay& bray)
	{
		bucket_rays[num_rays++] = bray;
	}
};

enum class TraversalScheme
{
	BFS = 0,
	DFS
};

class UnitStreamScheduler : public UnitBase
{
public:
	struct Configuration
	{
		paddr_t  treelet_addr{0};
		paddr_t  heap_addr{0};
		rtm::PackedTreelet* cheat_treelets{nullptr};

		uint num_root_rays{0};
		uint num_tms{0};
		uint num_banks{1};

		uint traversal_scheme{0}; //0-bfs, 1-dfs
		uint weight_scheme{0}; //If weight scheme = 2, we use the default DFS order
		uint max_active_segments{1024 * 1024};

		UnitSceneBuffer* scene_buffer{nullptr};
		UnitMainMemoryBase* main_mem{nullptr};
		uint                main_mem_port_offset{0};
		uint                main_mem_port_stride{1};
	};

public:
	class StreamSchedulerRequestCrossbar : public CasscadedCrossBar<StreamSchedulerRequest>
	{
	public:
		StreamSchedulerRequestCrossbar(uint ports, uint banks) : CasscadedCrossBar<StreamSchedulerRequest>(ports, banks, banks) {}

		uint get_sink(const StreamSchedulerRequest& request) override
		{
			//segment zero is a special case it needs to be distrubted accross all banks for performance reasons
			if(request.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
			{
				if(request.swi.segment_id == 0)
					return request.port * num_sinks() / num_sources();

				return request.swi.segment_id % num_sinks();
			}
			else
			{
				//otherwise cascade
				return request.port * num_sinks() / num_sources();
			}
		}
	};

	struct Bank
	{
		std::queue<uint> bucket_flush_queue;
		std::map<uint, RayBucket> ray_coalescer{};
	};

	class MemoryManager
	{
	private:
		paddr_t next_bucket_addr;
		std::stack<paddr_t> free_buckets;

	public:
		paddr_t alloc_bucket()
		{
			if(!free_buckets.empty())
			{
				paddr_t bucket_address = free_buckets.top();
				free_buckets.pop();
				return bucket_address;
			}

			paddr_t bucket_address = next_bucket_addr;

			next_bucket_addr += RAY_BUCKET_SIZE;
			if((next_bucket_addr % ROW_BUFFER_SIZE) == 0)
				next_bucket_addr += (NUM_DRAM_CHANNELS - 1) * ROW_BUFFER_SIZE;

			return bucket_address;
		}

		void free_bucket(paddr_t bucket_address)
		{
			free_buckets.push(bucket_address);
		}

		MemoryManager(uint channel_index, paddr_t start_address)
		{
			next_bucket_addr = align_to(ROW_BUFFER_SIZE, start_address);
			while((next_bucket_addr / ROW_BUFFER_SIZE) % NUM_DRAM_CHANNELS != channel_index)
				next_bucket_addr += ROW_BUFFER_SIZE;
		}
	};

	struct SegmentState
	{
		std::queue<paddr_t> bucket_address_queue{};
		uint                next_channel{0};
		uint                total_buckets{0};
		uint                retired_buckets{0};
		bool                parent_finished{false};
		bool                prefetch_issued{false};
		bool                prefetch_complete{false};
		bool				child_order_generated{false};

		uint64_t			weight{0};
		uint64_t			num_rays{0};
		uint64_t			scheduled_weight{0};
		uint				depth{0};
	};

	struct Scheduler
	{
		std::queue<uint> bucket_allocated_queue;
		std::queue<uint> bucket_request_queue;
		std::queue<uint> bucket_complete_queue;
		Cascade<RayBucket> bucket_write_cascade;

		std::vector<uint> last_segment_on_tm;
		std::map<uint, SegmentState> segment_state_map;
		rtm::PackedTreelet* cheat_treelets;
		paddr_t treelet_addr;

		std::vector<MemoryManager> memory_managers;

		//the list of segments in the scene buffer or schduled to be in the scene buffer
		uint last_segment_activated;
		std::vector<uint> candidate_segments; //the set of segments that are ready to issue buckets
		std::stack<uint> traversal_stack; //for DFS
		std::queue<uint> traversal_queue; //for BFS

		std::queue<uint> prefetch_queue;
		std::queue<uint> retire_queue;

		uint root_rays_counter;
		uint num_root_rays;
		uint traversal_scheme;
		uint weight_scheme;
		uint max_active_segments;

		uint concurent_prefetches;
		uint active_segments;

		Scheduler(const Configuration& config) : bucket_write_cascade(config.num_banks, 1), last_segment_on_tm(config.num_tms, ~0u),
			root_rays_counter(0), num_root_rays(config.num_root_rays),
			traversal_scheme(config.traversal_scheme), weight_scheme(config.weight_scheme),
			max_active_segments(config.max_active_segments), concurent_prefetches(0), active_segments(0)
		{
			for(uint i = 0; i < NUM_DRAM_CHANNELS; ++i)
				memory_managers.emplace_back(i, config.heap_addr);

			SegmentState& segment_state = segment_state_map[0];
			segment_state.parent_finished = false;

			cheat_treelets = config.cheat_treelets;
			treelet_addr = config.treelet_addr;

			rtm::PackedTreelet::Header root_header = cheat_treelets[0].header;

			candidate_segments.push_back(0);
			last_segment_activated = 0;
		}

		bool is_complete()
		{
			return candidate_segments.size() == 0;
		}
	};

	struct Channel
	{
		struct WorkItem
		{
			enum Type
			{
				READ_BUCKET,
				WRITE_BUCKET,
				READ_SEGMENT,
			};

			Type type{READ_BUCKET};
			paddr_t address;

			union
			{
				uint      dst_tm;
				RayBucket bucket;
			};

			WorkItem() {};
		};

		//work queue
		std::queue<WorkItem> work_queue;

		//stream state
		uint bytes_requested{0};

		//forwarding
		MemoryReturn forward_return{};
		bool forward_return_valid{false};

		Channel() {};
	};

	UnitSceneBuffer* _scene_buffer;
	UnitMainMemoryBase* _main_mem;
	uint                _main_mem_port_offset;
	uint                _main_mem_port_stride;

	//request flow from _request_network -> bank -> scheduler -> channel
	StreamSchedulerRequestCrossbar _request_network;
	std::vector<Bank> _banks;
	Scheduler _scheduler;
	std::vector<Channel> _channels;
	UnitMemoryBase::ReturnCrossBar _return_network;

public:
	UnitStreamScheduler(const Configuration& config) :_request_network(config.num_tms, config.num_banks), _banks(config.num_banks), _scheduler(config), _channels(NUM_DRAM_CHANNELS), _return_network(config.num_tms, config.num_tms, NUM_DRAM_CHANNELS), _scene_buffer(config.scene_buffer)
	{
		_main_mem = config.main_mem;
		_main_mem_port_offset = config.main_mem_port_offset;
		_main_mem_port_stride = config.main_mem_port_stride;
	}

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index)
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const StreamSchedulerRequest& request, uint port_index)
	{
		_request_network.write(request, port_index);
	}

	bool return_port_read_valid(uint port_index)
	{
		return _return_network.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index)
	{
		return _return_network.peek(port_index);
	}

	const MemoryReturn read_return(uint port_index)
	{
		return _return_network.read(port_index);
	}

private:
	void _proccess_request(uint bank_index);
	void _proccess_return(uint channel_index);
	void _update_scheduler();
	void _issue_request(uint channel_index);
	void _issue_return(uint channel_index);

public:
	class Log
	{
	public:
		const static uint NUM_COUNTERS = 6;
		union
		{
			struct
			{
				uint64_t rays;
				uint64_t work_items;
				uint64_t buckets_launched;
				uint64_t buckets_generated;
				uint64_t segments_launched;
				uint64_t single_bucket_segments;
			};
			uint64_t counters[NUM_COUNTERS];
		};


	public:
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

		void print()
		{
			printf("Rays: %lld\n", rays);
			printf("Ray duplication: %.2f\n", (float)work_items / rays);
			printf("Rays per bucket: %.2f\n", (float)work_items / buckets_launched);
			printf("Buckets per segment: %.2f\n", (float)buckets_launched / segments_launched);
			printf("Total segments: %llu\n", segments_launched);
			printf("Single buckets segments: %lld (%.2f%%)\n", single_bucket_segments, 100.0f * single_bucket_segments / segments_launched);
		}
	}
	log;
};

}
}
}