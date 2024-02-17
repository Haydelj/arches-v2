#pragma once 
#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-dram.hpp"
#include "units/unit-buffer.hpp"

#include "dual-streaming-kernel/include.hpp"

namespace Arches {
namespace Units {
namespace DualStreaming {

#define SCENE_BUFFER_SIZE (4 * 1024 * 1024)

#define MAX_ACTIVE_SEGMENTS (SCENE_BUFFER_SIZE / sizeof(Treelet))

#define RAY_BUCKET_SIZE (2048)
#define MAX_RAYS_PER_BUCKET ((RAY_BUCKET_SIZE - 16) / sizeof(BucketRay))

struct alignas(RAY_BUCKET_SIZE) RayBucket
{
	paddr_t next_bucket{ 0 };
	uint segment{ 0 };
	uint num_rays{ 0 };
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

enum class TraversalScheme {
	BFS = 0,
	DFS
};

class UnitStreamSchedulerDFS : public UnitBase
{
public:
	struct Configuration
	{
		paddr_t  treelet_addr;
		paddr_t  heap_addr;
		Treelet* cheat_treelets{ nullptr };

		uint num_root_rays;
		uint num_tms;
		uint num_banks;

		uint traversal_scheme = 1; // 0-bfs, 1-dfs

		UnitMainMemoryBase* main_mem;
		uint                main_mem_port_offset{ 0 };
		uint                main_mem_port_stride{ 1 };
	};

private:
	class StreamSchedulerRequestCrossbar : public CasscadedCrossBar<StreamSchedulerRequest>
	{
	public:
		StreamSchedulerRequestCrossbar(uint ports, uint banks) : CasscadedCrossBar<StreamSchedulerRequest>(ports, banks, banks) {}

		uint get_sink(const StreamSchedulerRequest& request) override
		{
			if (request.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
			{
				//if this is a workitem write or bucket completed then distrbute across banks
				if (request.segment == 0) {
					current_sink = (current_sink + 1) % num_sinks();
					return current_sink; // distributes across all banks
				}
				
				return request.segment % num_sinks();
			}
			else
			{
				//otherwise cascade
				return request.port * num_sinks() / num_sources();
			}
		}
		int current_sink = 0;
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
			if (!free_buckets.empty())
			{
				paddr_t bucket_address = free_buckets.top();
				free_buckets.pop();
				return bucket_address;
			}

			uint bucket_address = next_bucket_addr;

			next_bucket_addr += RAY_BUCKET_SIZE;
			if ((next_bucket_addr % ROW_BUFFER_SIZE) == 0)
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
			while ((next_bucket_addr / ROW_BUFFER_SIZE) % NUM_DRAM_CHANNELS != channel_index)
				next_bucket_addr += ROW_BUFFER_SIZE;
		}
	};

	struct SegmentState
	{
		std::queue<paddr_t> bucket_address_queue{};
		uint                next_channel{ 0 };
		uint                total_buckets{ 0 };
		uint                active_buckets{ 0 };
		bool                parent_finished{ false };

		bool			    is_top_level{ true };
		uint64_t			weight;
		uint64_t			num_rays;
		uint64_t				average_ray_weight;
		uint				depth = 0;
		bool				child_order_generated{ false };
	};


	struct Scheduler
	{
		std::queue<uint> bucket_allocated_queue;

		std::queue<uint> bucket_request_queue;
		std::queue<uint> bucket_complete_queue;
		Casscade<RayBucket> bucket_write_cascade;

		std::vector<uint> last_segment_on_tm;
		std::map<uint, SegmentState> segment_state_map;
		Treelet* cheat_treelets;
		paddr_t treelet_addr;

		std::vector<MemoryManager> memory_managers;

		//the list of segments in the scene buffer or schduled to be in the scene buffer
		std::set<uint> active_segments;

		//the set of segments that are ready to issue buckets
		std::vector<uint> candidate_segments;
		
		std::stack<uint> traversal_stack; // for DFS
		std::queue<uint> traversal_queue; // for BFS

		int root_rays_counter = 0;
		int num_root_rays = 0;
		uint traversal_scheme = 0;

		Scheduler(const Configuration& config) : bucket_write_cascade(config.num_banks, 1), last_segment_on_tm(config.num_tms, ~0u), num_root_rays(config.num_root_rays), traversal_scheme(config.traversal_scheme)
		{
			for (uint i = 0; i < NUM_DRAM_CHANNELS; ++i)
				memory_managers.emplace_back(i, config.heap_addr);

			SegmentState& segment_state = segment_state_map[0];
			segment_state.parent_finished = false;

			cheat_treelets = config.cheat_treelets;
			treelet_addr = config.treelet_addr;

			active_segments.insert(0);
			Treelet::Header root_header = cheat_treelets[0].header;
			candidate_segments.push_back(0);

		}

		bool is_complete()
		{
			return active_segments.size() == 0 && candidate_segments.size() == 0;
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

			Type type{ READ_BUCKET };
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
		uint bytes_requested{ 0 };

		//forwarding
		MemoryReturn forward_return{};
		bool forward_return_valid{ false };

		Channel() {};
	};

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
	UnitStreamSchedulerDFS(const Configuration& config) :_request_network(config.num_tms, config.num_banks), _banks(config.num_banks), _scheduler(config), _channels(NUM_DRAM_CHANNELS), _return_network(config.num_tms, NUM_DRAM_CHANNELS, NUM_DRAM_CHANNELS)
	{
		_main_mem = config.main_mem;
		_main_mem_port_offset = config.main_mem_port_offset;
		_main_mem_port_stride = config.main_mem_port_stride;
		log.log_root_rays(config.num_root_rays);
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
		uint64_t num_rays = 0;
		uint64_t leaf_nodes_total_rays = 0; // For testing early termination
		std::map<int, int> leaf_node_rays_counter;
		std::map<int, uint64_t> leaf_node_weights;
		std::map<int, std::vector<int>> ray_info;
		std::vector<int> leaf_completed_order;
		
		uint64_t number_of_treelets_visited = 0;

		void log_rays()
		{
			number_of_treelets_visited++;
		}
		void log_root_rays(uint num)
		{
			num_rays = num;
		}
		void log_leaf_rays(uint node_id, uint ray_id)
		{
			leaf_node_rays_counter[node_id]++;
			leaf_nodes_total_rays++;
			ray_info[ray_id].push_back(node_id);
		}
		void log_complete(uint node_id, uint64_t weight)
		{
			leaf_completed_order.push_back(node_id);
			leaf_node_weights[node_id] = weight;
		}
		void print()
		{
			printf("\n\n-------------------------Stream Scheduler Stats:----------------------\n");
			printf("Total number of rays: %llu\n", num_rays);

			uint64_t leaf_nodes_num = leaf_node_rays_counter.size();
			printf("Total number of leaf nodes: %llu\n", leaf_nodes_num);

			printf("Early Termination Stats:\n");
			printf("Total rays in leaf nodes: %llu\n", leaf_nodes_total_rays);
			printf("Average rays per leaf: %.3lf\n", 1.0 * leaf_nodes_total_rays / leaf_nodes_num);
			printf("Average leafs per ray: %.3lf\n", 1.0 * leaf_nodes_total_rays / num_rays);
			printf("Average treelets per ray: %.3lf\n", 1.0 * number_of_treelets_visited / num_rays);


			printf("Leaf completed order : \n");
			for (auto node_id: leaf_completed_order)
			{
				printf("Leaf node id: %d, Number of rays: %d, Average ray weight: %.3lf\n", node_id, leaf_node_rays_counter[node_id], 1.0 * leaf_node_weights[node_id] / leaf_node_rays_counter[node_id]);
			}
			printf("\n");

			//printf("Ray distributed info : \n");
			//for (auto& [ray_id, node_list] : ray_info)
			//{
			//	printf("Ray id: %d\n", ray_id);
			//	printf("Visited node list: ");
			//	for (auto node_id : node_list) printf("%d ", node_id);
			//	printf("\n");
			//}
		}
	}log;
};

}
}
}