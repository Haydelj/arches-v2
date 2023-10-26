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
	uint bytes_returned{0};
	uint next_ray{0};
	uint rays_complete{0};

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
		buffer_executing_on_tp.resize(num_tp, nullptr);
	}

	Casscade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;

	UnitStreamScheduler* _stream_scheduler;

	uint tm_index;
	uint rgs_complete{0};

	RayBucketBuffer* front_buffer;
	RayBucketBuffer* back_buffer;
	RayBucketBuffer ray_buffer[2];
	std::vector<RayBucketBuffer*> buffer_executing_on_tp;

	std::queue<uint> completed_buckets;
	std::queue<uint> workitem_request_queue;

	bool request_valid{false};
	MemoryRequest request;

	void clock_rise() override
	{
		_request_network.clock();

		if(!request_valid && _request_network.is_read_valid(0))
		{
			request = _request_network.read(0);
			request_valid = true;
		}

		if(_stream_scheduler->return_port_read_valid(tm_index))
		{
			MemoryReturn ret = _stream_scheduler->read_return(tm_index);

			if(ret.size == 0)
			{
				//termination condition
				back_buffer->bytes_returned = RAY_BUCKET_SIZE;
				back_buffer->ray_bucket.segment = ~0u;
				back_buffer->ray_bucket.num_rays = 32;
			}
			else
			{
				uint buffer_address = ret.paddr % RAY_BUCKET_SIZE;
				std::memcpy(&back_buffer->data_u8[buffer_address], ret.data, ret.size);
				back_buffer->bytes_returned += ret.size;
			}
		}
	}

	void issue_requests()
	{
		//back buffer is full and front is empty swap buffers
		if(back_buffer->bytes_returned == RAY_BUCKET_SIZE && front_buffer->next_ray >= front_buffer->ray_bucket.num_rays)
		{
			std::swap(front_buffer, back_buffer);

			//reset back buffer to empty state
			back_buffer->bytes_returned = 0;
			back_buffer->requested = false;
		}

		//back buffer is complete request a fill from the stream scheduler
		if(!back_buffer->requested && back_buffer->rays_complete == back_buffer->ray_bucket.num_rays && _stream_scheduler->request_port_write_valid(tm_index))
		{
			StreamSchedulerRequest req;
			req.type = StreamSchedulerRequest::Type::LOAD_BUCKET;
			req.port = tm_index;
			req.segment = ~0u; //TODO send segment hint
			_stream_scheduler->write_request(req, req.port);

			//reset ray return state as completed
			back_buffer->rays_complete = 0;
			back_buffer->next_ray = 0;

			back_buffer->requested = true;
			return;
		}

		if(request_valid)
		{
			//a store request is pending service it
			if(request.type == MemoryRequest::Type::STORE) //ray write or conditionally store hit
			{
				//forward to stream scheduler
				if(_stream_scheduler->request_port_write_valid(tm_index))
				{
					StreamSchedulerRequest req;
					req.type = StreamSchedulerRequest::Type::STORE_WORKITEM;
					req.port = tm_index;

					WorkItem wi;
					std::memcpy(&wi, request.data, request.size);
					req.bray = wi.bray;
					req.segment = wi.segment;

					_stream_scheduler->write_request(req, req.port);
					request_valid = false;
				}
			}
		}
		else
		{
			//we can't send these until draining the pending stores

			//ray generation complete
			if(rgs_complete == 32 && _stream_scheduler->request_port_write_valid(tm_index))
			{
				StreamSchedulerRequest req;
				req.type = StreamSchedulerRequest::Type::BUCKET_COMPLETE;
				req.port = tm_index;
				req.segment = 0;
				_stream_scheduler->write_request(req, req.port);

				rgs_complete = ~0u;
				return;
			}

			//tell the stream scheduler that a bucket completed
			if(!completed_buckets.empty() && _stream_scheduler->request_port_write_valid(tm_index))
			{
				StreamSchedulerRequest req;
				req.type = StreamSchedulerRequest::Type::BUCKET_COMPLETE;
				req.port = tm_index;
				req.segment = completed_buckets.front();
				_stream_scheduler->write_request(req, req.port);

				completed_buckets.pop();
				return;
			}
		}
	}

	void issue_returns()
	{
		//a load request is pending put it in the request queue
		if(request_valid && request.type == MemoryRequest::Type::LOAD)
		{
			workitem_request_queue.push(request.port);
			request_valid = false;

			//mark previous ray as complete
			if(buffer_executing_on_tp[request.port])
			{
				RayBucketBuffer* buffer = buffer_executing_on_tp[request.port];
				buffer->rays_complete++;
				if(buffer->rays_complete == buffer->ray_bucket.num_rays)
					completed_buckets.push(buffer->ray_bucket.segment);
			}
			else
			{
				rgs_complete++;
			}
		}

		//if we have a filled bucket pop a ray from it
		if(!workitem_request_queue.empty() && front_buffer->next_ray < front_buffer->ray_bucket.num_rays && _return_network.is_write_valid(request.port))
		{
			MemoryReturn ret;

			ISA::RISCV::RegAddr reg_addr;
			reg_addr.reg = 0;
			reg_addr.reg_type = ISA::RISCV::RegType::FLOAT;
			reg_addr.sign_ext = 0;
			ret.dst = reg_addr.u8;

			ret.size = sizeof(WorkItem);
			ret.port = workitem_request_queue.front();

			WorkItem wi;
			wi.bray = front_buffer->ray_bucket.bucket_rays[front_buffer->next_ray];
			wi.segment = front_buffer->ray_bucket.segment;
			std::memcpy(ret.data, &wi, ret.size);
			_return_network.write(ret, ret.port);

			workitem_request_queue.pop();
			front_buffer->next_ray++;
			buffer_executing_on_tp[ret.port] = front_buffer;
		}
	}

	void clock_fall() override
	{
		issue_requests();
		issue_returns();
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