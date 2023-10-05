#pragma once 
#include "../../stdafx.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"
#include "../unit-main-memory-base.hpp"
#include "../unit-buffer.hpp"

#include "../../../../dual-streaming-benchmark/src/include.hpp"

namespace Arches { namespace Units { namespace DualStreaming {

#define SCENE_BUFFER_SIZE (4 * 1024 * 1024)

#define MAX_ACTIVE_SEGMENTS (SCENE_BUFFER_SIZE / sizeof(Treelet))

#define RAY_BUCKET_SIZE 2048
#define MAX_RAYS_PER_BUCKET (sizeof(BucketRay) / 2036)

struct RayBucket
{
	paddr_t next_bucket;
	uint num_rays{0};
	BucketRay bucket_rays[MAX_RAYS_PER_BUCKET];
};

class UnitStreamScheduler : UnitBase
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

	class StreamSchedulerCrossBar : public CasscadedCrossBar<StreamSchedulerRequest>
	{
	private:
		uint64_t mask;

	public:
		StreamSchedulerCrossBar(uint ports, uint banks, uint64_t bank_select_mask) : mask(bank_select_mask), CasscadedCrossBar<StreamSchedulerRequest>(ports, banks, banks) {}

		uint get_sink(const StreamSchedulerRequest& request) override
		{
			if(request.type == StreamSchedulerRequest::Type::LOAD_BUCKET)
			{
				return 0;
			}
			else if(request.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
			{
				uint bank = pext(request.work_item.segment, mask);
			}
		}
	};

	class Stream
	{
	public:
		paddr_t base_addr{0};
		uint num_lines{0};

		uint num_returned{0};
		uint num_requested{0};

		Stream()
		{

		}

		Stream(paddr_t base_addr, uint bytes, uint interface_width, uint dst)
		{
			this->base_addr = base_addr;
			this->num_lines = bytes / CACHE_BLOCK_SIZE;
		}

		bool fully_requested()
		{
			return num_requested == num_lines;
		}

		bool fully_returned()
		{
			return num_returned == num_lines;
		}

		void count_return()
		{
			num_returned++;
		}

		MemoryRequest get_next_request()
		{
			MemoryRequest req;
			req.type = MemoryRequest::Type::LOAD;
			req.size = CACHE_BLOCK_SIZE;
			req.paddr = base_addr + num_requested;
			num_requested++;

			return req;
		}
	};

	struct Bank
	{
		Stream open_stream;
		std::queue<Stream> buck_stream_queue; std::queue<WorkItem> ray_write_queue;
	};

	struct RayReadQueueEntry
	{
		uint port_index;
		uint last_segment;
	};

	struct BucketList
	{
		paddr_t head_bucket_address;

		paddr_t tail_bucket_address;
		uint tail_fill;
		uint tail_channel;
	};



private:
	UnitMainMemoryBase* _main_mem;
	uint                _main_mem_port_offset;
	uint                _main_mem_port_stride;

	uint _num_tms;
	std::vector<Bank> _banks;

	StreamSchedulerCrossBar _request_network;
	FIFOArray<MemoryReturn> _return_network;

	std::queue<uint> _ray_bucket_read_queue;

	paddr_t _scene_start, _bucket_start, _bucket_end;

	std::stack<paddr_t>        free_buckets; //todo this should be a linked list in memory
	std::map<uint, BucketList> bucket_lists;

	std::queue<uint> eligable_segments;
	uint active_segment_arb_index;
	uint num_active_segments;

private:
	void proccess_request(uint bank_index)
	{
		if(!_request_network.is_read_valid(bank_index)) return;
		const StreamSchedulerRequest& req = _request_network.peek(bank_index);
		Bank& bank = _banks[bank_index];

		if(req.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
		{
			bank.ray_write_queue.push(req.work_item);
			_request_network.read(bank_index);
		}
		else if(req.type == StreamSchedulerRequest::Type::LOAD_BUCKET)
		{
			//Insert the tm in the read queue
			_ray_bucket_read_queue.emplace(req.port, req.last_segment);
			_request_network.read(bank_index);
		}
	}

	void proccess_return(uint bank_index)
	{

	}

	void issue_request(uint bank_index)
	{

	}

	void issue_return(uint bank_index)
	{

	}

public:
	UnitStreamScheduler(Configuration config) : _request_network(config.num_tms, 1, 0b111), _return_network(config.num_tms)
	{
		_scene_start = config.scene_start;
		_bucket_start = config.bucket_start;
		_bucket_end = _bucket_start;

		_num_tms = config.num_tms;

		_main_mem = config.main_mem;
		_main_mem_port_offset = config.main_mem_port_offset;
		_main_mem_port_stride = config.main_mem_port_stride;

		_banks.resize(8);//TODO match num dram channels
	}

	void clock_rise() override
	{
		_request_network.clock();

		for(uint i = 0; i < _banks.size(); ++i)
		{
			proccess_request(i);
			proccess_return(i);
		}
	}

	void clock_fall() override
	{
		for(uint i = 0; i < _banks.size(); ++i)
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