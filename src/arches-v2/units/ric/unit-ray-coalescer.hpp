#pragma once 
#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-buffer.hpp"

#include "ric-kernel/include.hpp"

namespace Arches { namespace Units { namespace RIC {

#define RAY_BUCKET_SIZE 1024

struct alignas(RAY_BUCKET_SIZE) RayBucket
{
	struct Header
	{
		paddr_t next_bucket{0};
		uint32_t segment_id{0};
		uint32_t num_rays{0};
	};

	const static uint MAX_RAYS = (RAY_BUCKET_SIZE - sizeof(Header)) / sizeof(uint32_t);

	Header header;
	uint32_t ray_ids[MAX_RAYS];

	bool is_full()
	{
		return header.num_rays == MAX_RAYS;
	}

	void write_ray(const uint32_t& ray_id)
	{
		ray_ids[header.num_rays++] = ray_id;
	}
};

enum class TraversalScheme
{
	BFS = 0,
	DFS
};

class UnitRayCoalescer : public UnitBase
{
public:
	struct Configuration
	{
		paddr_t  treelet_addr{0};
		paddr_t  heap_addr{0};
		rtm::WideTreeletBVH::Treelet::Header* cheat_treelets{nullptr};

		uint num_root_rays{0};
		uint num_tms{0};
		uint num_banks{1};
		uint num_channels{1};
		uint64_t row_size{1};
		uint64_t block_size{1};

		uint traversal_scheme{0}; //0-bfs, 1-dfs
		uint weight_scheme{0}; //If weight scheme = 2, we use the default DFS order
		uint max_active_segments_size{1024 * 1024};

		UnitMemoryBase* l2_cache{nullptr};

		UnitMainMemoryBase* main_mem{nullptr};
		uint                main_mem_port_offset{0};
		uint                main_mem_port_stride{1};
	};

public:
	struct WorkItem
	{
		uint32_t ray_id;
		uint16_t segment_id;
		uint16_t order_hint;
	};

	struct Request
	{
		enum class Type : uint8_t
		{
			NA,
			STORE_WORKITEM,
			LOAD_BUCKET,
			BUCKET_COMPLETE,
		};

		Type     type{Type::NA};
		uint16_t port;

		union
		{
			WorkItem swi;
			struct
			{
				uint previous_segment_id;
			}lb;
			struct
			{
				uint segment_id;
			}bc;
		};

		Request() {};

		Request(const MemoryReturn& other)
		{
			*this = other;
		}

		Request& operator=(const Request& other)
		{
			type = other.type;
			port = other.port;
			if(type == Request::Type::STORE_WORKITEM)
			{
				swi = other.swi;
			}
			else if(type == Request::Type::LOAD_BUCKET)
			{
				lb = other.lb;
			}
			else if(type == Request::Type::BUCKET_COMPLETE)
			{
				bc = other.bc;
			}
		}
	};

	std::queue<uint> bucket_flush_queue;
	std::map<uint, RayBucket> ray_coalescer{};

	class MemoryManager
	{
	private:
		paddr_t _next_bucket_addr;
		uint _num_channels;
		uint64_t _row_size;
		std::stack<paddr_t> _free_buckets;

	public:
		paddr_t alloc_bucket()
		{
			if(!_free_buckets.empty())
			{
				paddr_t bucket_address = _free_buckets.top();
				_free_buckets.pop();
				return bucket_address;
			}

			paddr_t bucket_address = _next_bucket_addr;

			_next_bucket_addr += RAY_BUCKET_SIZE;
			if((_next_bucket_addr % _row_size) == 0)
				_next_bucket_addr += (_num_channels - 1) * _row_size;

			return bucket_address;
		}

		uint get_channel(paddr_t address) const
		{
			return (address / _row_size) % _num_channels;
		}

		void free_bucket(paddr_t bucket_address)
		{
			_free_buckets.push(bucket_address);
		}

		MemoryManager(uint channel_index, uint num_channels, uint64_t row_size, paddr_t start_address)
		{
			_num_channels = num_channels;
			_row_size = row_size;
			_next_bucket_addr = align_to(_row_size, start_address);
			while((_next_bucket_addr / _row_size) % _num_channels != channel_index)
				_next_bucket_addr += _row_size;
		}
	};

	struct SegmentState
	{
		std::queue<paddr_t> bucket_address_queue{};
		uint                next_channel{0};
		uint                total_buckets{0};
		uint                retired_buckets{0};
		bool                parent_finished{false};
		bool				children_scheduled{false};

		uint64_t            size{0};
		uint64_t			weight{0};
		uint64_t			num_rays{0};
		uint64_t			scheduled_weight{0};
		uint				depth{0};
		uint                num_tms{0};
	};

	struct Scheduler
	{
		std::queue<uint> bucket_allocated_queue;
		std::queue<uint> bucket_request_queue;
		std::set<uint>  bucket_request_set;
		std::queue<uint> bucket_complete_queue;
		Cascade<RayBucket> bucket_write_cascade;

		std::vector<uint> last_segment_on_tm;
		std::map<uint, SegmentState> segment_state_map;
		rtm::WideTreeletBVH::Treelet::Header* cheat_treelets;
		paddr_t treelet_addr;

		std::vector<MemoryManager> memory_managers;

		//the list of segments in the scene buffer or schduled to be in the scene buffer
		uint last_segment_activated;
		std::vector<uint> active_segments; //the set of segments that are ready to issue buckets
		std::stack<uint> traversal_stack; //for DFS
		std::queue<uint> traversal_queue; //for BFS

		UnitMemoryBase* l2_cache;
		std::vector<std::queue<paddr_t>> prefetch_queues;
		uint sector_size;

		uint root_rays_counter;
		uint num_root_rays;

		uint traversal_scheme;
		uint weight_scheme;

		uint max_active_segments_size{1024 << 20};
		uint active_segments_size{0};
		uint retired_segments{0};

		Scheduler(const Configuration& config) : bucket_write_cascade(config.num_banks, 1), last_segment_on_tm(config.num_tms, ~0u),
			root_rays_counter(0), num_root_rays(config.num_root_rays),
			traversal_scheme(config.traversal_scheme), weight_scheme(config.weight_scheme),
			max_active_segments_size(config.max_active_segments_size),
			sector_size(config.row_size), l2_cache(config.l2_cache)
		{
			for(uint i = 0; i < config.num_channels; ++i)
				memory_managers.emplace_back(i, config.num_channels, config.row_size, config.heap_addr);

			SegmentState& segment_state = segment_state_map[0];
			segment_state.parent_finished = false;

			cheat_treelets = config.cheat_treelets;
			treelet_addr = config.treelet_addr;

			rtm::WideTreeletBVH::Treelet::Header root_header = cheat_treelets[0];

			active_segments.push_back(0);
			last_segment_activated = 0;
			active_segments_size = cheat_treelets[0].bytes;

			prefetch_queues.resize(16);
		}

		bool is_complete()
		{
			return active_segments.size() == 0;
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

	uint64_t _block_size;
	UnitMainMemoryBase* _main_mem;
	uint                _main_mem_port_offset;
	uint                _main_mem_port_stride;

	//request flow from _request_network -> bank -> scheduler -> channel
	Cascade<Request> _request_network;
	Scheduler _scheduler;
	std::vector<Channel> _channels;
	UnitMemoryBase::ReturnCrossBar _return_network;

	bool prefetched_root = false;

public:
	UnitRayCoalescer(const Configuration& config) :
		_request_network(config.num_tms, config.num_banks), 
		_scheduler(config), _channels(config.num_channels), 
		_return_network(config.num_channels, config.num_tms), _block_size(config.block_size)
	{
		_main_mem = config.main_mem;
		_main_mem_port_offset = config.main_mem_port_offset;
		_main_mem_port_stride = config.main_mem_port_stride;

		_prefetch(0);
	}

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index)
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const Request& request, uint port_index)
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
	void _prefetch(uint segment);
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

}}}