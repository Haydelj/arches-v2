#pragma once 
#include "../../stdafx.hpp"

#include "../unit-base.hpp"
#include "../unit-dram.hpp"
#include "../unit-buffer.hpp"

#include "../../../../dual-streaming-benchmark/src/include.hpp"

namespace Arches { namespace Units { namespace DualStreaming {

#define SCENE_BUFFER_SIZE (4 * 1024 * 1024)

#define MAX_ACTIVE_SEGMENTS (SCENE_BUFFER_SIZE / sizeof(Treelet))

#define RAY_BUCKET_SIZE 2048
#define MAX_RAYS_PER_BUCKET ((RAY_BUCKET_SIZE - 12) / sizeof(BucketRay))

struct RayBucket
{
	paddr_t next_bucket{0};
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

class UnitStreamScheduler : public UnitBase
{
public:
	struct Configuration
	{
		paddr_t scene_start;
		paddr_t bucket_start;

		uint num_tms;

		UnitMainMemoryBase* main_mem;
		uint                main_mem_port_offset{0};
		uint                main_mem_port_stride{1};
	};

private:
	class StreamSchedulerRequestCrossbar : public CasscadedCrossBar<StreamSchedulerRequest>
	{
	public:
		StreamSchedulerRequestCrossbar(uint ports, uint banks) : CasscadedCrossBar<StreamSchedulerRequest>(ports, banks, banks) {}

		uint get_sink(const StreamSchedulerRequest& request) override
		{
			if(request.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
			{
				//if this is a workitem write then distrbute across banks
				return request.work_item.segment % num_sinks();
			}
			else
			{
				//if this is a bcuket request then cascade
				request.port * num_sinks() / num_sources();
			}
		}
	};

	struct SegmentState
	{
		paddr_t   tail_bucket_address{0x0};
		RayBucket write_buffer{};

		//Once the parent is finished we are able to dispatch the last bucket for the segment
		bool      parent_finished{false};
	};

	struct Channel
	{
		enum StreamState
		{
			NA,
			READ_BUCKET,
			WRITE_BUCKET,
		};

		std::map<uint, SegmentState> segment_state_map;
		std::queue<uint> segment_dispatch_queue;

		std::queue<uint> bucket_write_queue;
		std::queue<uint> bucket_read_queue;

		StreamState stream_state{NA};

		struct
		{
			uint segment_index{0};
			uint lines_requested{0};
			uint dst_tm{0};
		}
		current_bucket;

		MemoryReturn forward_return;

		paddr_t next_bucket_addr;
		std::stack<paddr_t> free_buckets; //todo this should be a linked list in memory

		//allocates space for a new bucket
		paddr_t alloc_bucket()
		{
			if(!free_buckets.empty())
			{
				paddr_t bucket_address = free_buckets.top();
				free_buckets.pop();
				return bucket_address;
			}

			uint bucket_address = next_bucket_addr++;
			if((next_bucket_addr % ROW_BUFFER_SIZE) == 0)
				next_bucket_addr += (DRAM_CHANNELS - 1) * ROW_BUFFER_SIZE;

			return bucket_address;
		}

		//allocates space for a new bucket
		void free_bucket(paddr_t bucket_address)
		{
			free_buckets.push(bucket_address);
		}

		Channel(uint channel_index, paddr_t heap_address)
		{
			next_bucket_addr = align_to(ROW_BUFFER_SIZE, heap_address);
			while((next_bucket_addr / ROW_BUFFER_SIZE) % DRAM_CHANNELS != channel_index)
				next_bucket_addr += ROW_BUFFER_SIZE;
		}
	};

private:
	UnitMainMemoryBase* _main_mem;
	uint                _main_mem_port_offset;
	uint                _main_mem_port_stride;

	std::vector<Channel> _channels;
	StreamSchedulerRequestCrossbar _request_network;
	UnitMemoryBase::ReturnCrossBar _return_network;

	std::queue<uint> _tm_bucket_queue;
	uint             _root_rays_in_flight{0};

private:
	void proccess_request(uint channel_index);
	void proccess_return(uint channel_index);
	void issue_request(uint channel_index);
	void issue_return(uint channel_index);

public:
	UnitStreamScheduler(Configuration config) : _request_network(config.num_tms, DRAM_CHANNELS), _return_network(config.num_tms, DRAM_CHANNELS)
	{
		_main_mem = config.main_mem;
		_main_mem_port_offset = config.main_mem_port_offset;
		_main_mem_port_stride = config.main_mem_port_stride;

		for(uint i = 0; i < DRAM_CHANNELS; ++i)
			_channels.push_back({i, config.bucket_start});
	}

	void clock_rise() override
	{
		_request_network.clock();

		for(uint i = 0; i < _channels.size(); ++i)
		{
			proccess_request(i);
			proccess_return(i);
		}
	}

	void clock_fall() override
	{
		for(uint i = 0; i < _channels.size(); ++i)
		{
			issue_request(i);
			issue_return(i);
		}

		_return_network.clock();
	}

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
};

}}}