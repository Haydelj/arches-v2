#pragma once 
#include "../../stdafx.hpp"

#include "../unit-base.hpp"
#include "../unit-memory-base.hpp"
#include "unit-stream-scheduler.hpp"

namespace Arches { namespace Units { namespace DualStreaming {


struct RayBucketBuffer
{
	union
	{
		RayBucket ray_bucket;
		uint8_t data_u8[RAY_BUCKET_SIZE];
	};

	bool requested{false};
	uint next_ray{0};
	uint bytes_returned{0};

	RayBucketBuffer() 
	{
		ray_bucket.num_rays = 0;
	}
};

class UnitRayStagingBuffer : public UnitMemoryBase
{
public:
	UnitRayStagingBuffer(uint num_tp, uint tm_index, UnitStreamScheduler* stream_scheduler) : UnitMemoryBase(),
		_request_network(num_tp, 1), _return_network(num_tp), tm_index(tm_index), _stream_scheduler(stream_scheduler)
	{
		front_buffer = &ray_buffer[0];
		back_buffer = &ray_buffer[1];
	}

	Casscade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;

	UnitStreamScheduler* _stream_scheduler;

	uint tm_index;

	RayBucketBuffer* front_buffer;
	RayBucketBuffer* back_buffer;
	RayBucketBuffer ray_buffer[2];

	bool request_valid{false};
	MemoryRequest request;

	void clock_rise() override
	{
		//requests
		_request_network.clock();

		if(_request_network.is_read_valid(0))
		{
			request = _request_network.read(0);
			request_valid = true;
		}

		//returns
		if(_stream_scheduler->return_port_read_valid(tm_index))
		{
			MemoryReturn ret = _stream_scheduler->read_return(tm_index);
			uint buffer_address = ret.paddr % RAY_BUCKET_SIZE;
			std::memcpy(&back_buffer->data_u8[buffer_address], ret.data, ret.size);
			back_buffer->bytes_returned += ret.size;
		}
	}

	void clock_fall() override
	{
		if(front_buffer->next_ray == front_buffer->ray_bucket.num_rays && back_buffer->bytes_returned == RAY_BUCKET_SIZE)
			std::swap(front_buffer, back_buffer);

		if(!back_buffer->requested && _stream_scheduler->request_port_write_valid(tm_index))
		{
			StreamSchedulerRequest req;
			req.type = StreamSchedulerRequest::Type::LOAD_BUCKET;
			req.port = tm_index;
			req.last_segment = 0;
			_stream_scheduler->write_request(req, req.port);
			back_buffer->requested = true;
			return;
		}

		if(!request_valid) return;

		if(request.type == MemoryRequest::Type::LOAD)
		{
			//if we have a filled bucket pop a ray from it otherwise stall
			if(front_buffer->next_ray < front_buffer->ray_bucket.num_rays)
			{
				MemoryReturn ret = request;
				std::memcpy(ret.data, &front_buffer->ray_bucket.bucket_rays[front_buffer->next_ray], ret.size);
				_return_network.write(ret, 0);
				front_buffer->next_ray++;
			}
		}
		else if(request.type == MemoryRequest::Type::STORE) //ray write or conditionally store hit
		{
			//forward to stream scheduler
			if(_stream_scheduler->request_port_write_valid(tm_index))
			{
				StreamSchedulerRequest req;
				req.type = StreamSchedulerRequest::Type::STORE_WORKITEM;
				req.port = tm_index;
				std::memcpy(&req.work_item, request.data, sizeof(WorkItem));
				_stream_scheduler->write_request(req, req.port);
				request_valid = false;
			}
		}

		_return_network.clock();
	}

	bool request_port_write_valid(uint port_index) override
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request, uint port_index) override
	{
		_request_network.write(request, port_index);
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
};

}}}