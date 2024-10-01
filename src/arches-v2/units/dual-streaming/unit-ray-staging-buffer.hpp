#pragma once 
#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-memory-base.hpp"
#include "unit-stream-scheduler.hpp"
#include "unit-hit-record-updater.hpp"

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

	RayBucketBuffer()
	{
		ray_bucket.header.num_rays = 0;
	}
};

class UnitRayStagingBuffer : public UnitMemoryBase
{
public:
	UnitRayStagingBuffer(uint num_tp, uint tm_index, UnitStreamScheduler* stream_scheduler, UnitHitRecordUpdater* hit_record_updater) : UnitMemoryBase(),
		_request_network(num_tp, 1), _return_network(num_tp), num_tp(num_tp), tm_index(tm_index), _stream_scheduler(stream_scheduler), _hit_record_updater(hit_record_updater)
	{
		returned_hit.paddr = ~0;
	}

private:
	Cascade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;

	UnitStreamScheduler*  _stream_scheduler;
	UnitHitRecordUpdater* _hit_record_updater;

	uint tm_index;
	uint num_tp;
	uint rgs_complete{0};

#define BUFFER_NUMBER 2
	RayBucketBuffer ray_buffer[BUFFER_NUMBER];
	uint front_buffer_id = 0;
	uint filling_buffer_id = 1;

	struct SegmentState
	{
		uint active_rays{0};
		uint active_buckets{0};
	};

	std::map<uint, SegmentState> segment_state_map;
	std::map<std::pair<uint, uint>, uint> segment_executing_on_thread;
	std::queue<uint> completed_buckets;

	std::queue<uint> workitem_request_queue;
	std::queue<std::pair<uint, uint>> thread_workitem_request_queue;

	MemoryReturn returned_hit;
	std::map<paddr_t, std::queue<std::pair<uint16_t, uint16_t>>> tp_load_hit_request;

	bool request_valid{false};
	MemoryRequest request{};

	void clock_rise() override
	{
		_request_network.clock();

		if(!request_valid && _request_network.is_read_valid(0))
		{
			request = _request_network.read(0);
			request_valid = true;
		}

		if(_hit_record_updater->return_port_read_valid(tm_index) && returned_hit.paddr == ~0)
		{
			returned_hit = _hit_record_updater->read_return(tm_index);
			_assert(returned_hit.size == sizeof(rtm::Hit));
		}

		if(_stream_scheduler->return_port_read_valid(tm_index))
		{
			MemoryReturn ret = _stream_scheduler->read_return(tm_index);

			if(ret.size == 0)
			{
				//termination condition
				ray_buffer[filling_buffer_id].bytes_returned = RAY_BUCKET_SIZE;
				ray_buffer[filling_buffer_id].ray_bucket.header.segment_id = INVALID_SEGMENT_ID;
				ray_buffer[filling_buffer_id].ray_bucket.header.num_rays = num_tp;
			}
			else
			{
				uint buffer_address = ret.paddr % RAY_BUCKET_SIZE;
				std::memcpy(&ray_buffer[filling_buffer_id].data_u8[buffer_address], ret.data, ret.size);
				ray_buffer[filling_buffer_id].bytes_returned += ret.size;
			}
		}
	}

	void issue_requests()
	{
		if(ray_buffer[(front_buffer_id + 1) % BUFFER_NUMBER].bytes_returned == RAY_BUCKET_SIZE && ray_buffer[front_buffer_id].next_ray >= ray_buffer[front_buffer_id].ray_bucket.header.num_rays)
		{
			//reset back buffer to empty state
			ray_buffer[front_buffer_id].bytes_returned = 0;
			ray_buffer[front_buffer_id].requested = false;
			front_buffer_id = (front_buffer_id + 1) % BUFFER_NUMBER;
		}

		if(ray_buffer[filling_buffer_id].bytes_returned == RAY_BUCKET_SIZE && ray_buffer[(filling_buffer_id + 1) % BUFFER_NUMBER].bytes_returned == 0)
		{
			filling_buffer_id = (filling_buffer_id + 1) % BUFFER_NUMBER;
		}

		if(_stream_scheduler->request_port_write_valid(tm_index))
		{
			if(!ray_buffer[filling_buffer_id].requested)
			{
				//back buffer is drained request a fill from the stream scheduler
				StreamSchedulerRequest req;
				int dis = (filling_buffer_id - front_buffer_id + BUFFER_NUMBER) % BUFFER_NUMBER;
				req.type = StreamSchedulerRequest::Type::LOAD_BUCKET;
				req.port = tm_index;
				req.lb.previous_segment_id = dis;
				_stream_scheduler->write_request(req, req.port);

				//reset ray return state as completed
				ray_buffer[filling_buffer_id].next_ray = 0;
				ray_buffer[filling_buffer_id].requested = true;
				return;
			}
			else if(request_valid && request.size == sizeof(WorkItem) && request.type == MemoryRequest::Type::STORE)
			{
				
				//a store is pending forward to stream scheduler
				StreamSchedulerRequest req;
				req.type = StreamSchedulerRequest::Type::STORE_WORKITEM;
				req.port = tm_index;

				WorkItem wi;
				std::memcpy(&wi, request.data, request.size);
				req.swi = wi;

				_stream_scheduler->write_request(req, req.port);
				request_valid = false;
				return;
			}
			else if(!completed_buckets.empty() && _stream_scheduler->request_port_write_valid(tm_index))
			{
				//a bucket completed
				StreamSchedulerRequest req;
				req.type = StreamSchedulerRequest::Type::BUCKET_COMPLETE;
				req.port = tm_index;
				req.bc.segment_id = completed_buckets.front();
				_stream_scheduler->write_request(req, req.port);

				completed_buckets.pop();
				return;
			}
		}

		if(request_valid && request.size == sizeof(rtm::Hit) &&
			_hit_record_updater->request_port_write_valid(tm_index))
		{
			HitRecordUpdaterRequest req;
			req.port = tm_index;
			req.hit_info.hit_address = request.paddr;
			req.hit_info.hit.t = T_MAX;

			if(request.type == MemoryRequest::Type::STORE)
			{
				std::memcpy(&req.hit_info.hit, request.data, request.size);
				req.type = HitRecordUpdaterRequest::TYPE::STORE;
			}
			else
			{
				req.type = HitRecordUpdaterRequest::TYPE::LOAD;
				tp_load_hit_request[request.paddr].push({request.port, request.dst});
			}

			_hit_record_updater->write_request(req, req.port);
			request_valid = false;
			return;
		}
	}

	void issue_returns()
	{
		if(returned_hit.paddr != ~0)
		{
			_assert(returned_hit.size == sizeof(rtm::Hit));
			_assert(tp_load_hit_request.count(returned_hit.paddr));

			auto& queue = tp_load_hit_request[returned_hit.paddr];
			const auto& tp_request = queue.front();

			rtm::Hit hit;
			std::memcpy(&hit, returned_hit.data, returned_hit.size);
			if(_return_network.is_write_valid(tp_request.first))
			{
				returned_hit.port = tp_request.first;
				returned_hit.dst = tp_request.second;

				_return_network.write(returned_hit, returned_hit.port);

				queue.pop();
				if(queue.empty())
					tp_load_hit_request.erase(returned_hit.paddr);
			}

			returned_hit.paddr = ~0;
		}

		//a load request is pending put it in the request queue
		if(request_valid && request.size == sizeof(WorkItem) && request.type == MemoryRequest::Type::LOAD)
		{
			//workitem_request_queue.push(request.port);
			thread_workitem_request_queue.push({request.port, request.dst});
			//mark previous ray as complete
			if(segment_executing_on_thread.count({request.port, request.dst}))
			{
				uint segment_index = segment_executing_on_thread[{request.port, request.dst}];
				SegmentState& segment_state = segment_state_map[segment_index];

				segment_state.active_rays--;
				if(segment_state.active_rays == 0)
				{
					for(uint i = 0; i < segment_state.active_buckets; ++i)
						completed_buckets.push(segment_index);
					segment_state_map.erase(segment_index);
				}
			}

			request_valid = false;
		}

		//if we have a filled bucket pop a ray from it
		if(!thread_workitem_request_queue.empty() && ray_buffer[front_buffer_id].next_ray < ray_buffer[front_buffer_id].ray_bucket.header.num_rays && _return_network.is_write_valid(request.port))
		{
			auto [port, dst] = thread_workitem_request_queue.front();
			thread_workitem_request_queue.pop();

			WorkItem wi;
			wi.bray = ray_buffer[front_buffer_id].ray_bucket.bucket_rays[ray_buffer[front_buffer_id].next_ray];
			wi.segment_id = ray_buffer[front_buffer_id].ray_bucket.header.segment_id;
			wi.use_scene_buffer = ray_buffer[front_buffer_id].ray_bucket.header.use_scene_buffer;

			MemoryReturn ret;
			ret.size = sizeof(WorkItem);
			ret.port = port;
			ret.dst = dst;
			std::memcpy(ret.data, &wi, ret.size);

			_return_network.write(ret, ret.port);

			if(ray_buffer[front_buffer_id].next_ray == 0)
			{
				segment_state_map[wi.segment_id].active_buckets++;
				segment_state_map[wi.segment_id].active_rays += ray_buffer[front_buffer_id].ray_bucket.header.num_rays;
			}

			//segment_executing_on_tp[ret.port] = wi.segment;
			segment_executing_on_thread[{ret.port, ret.dst}] = wi.segment_id;

			//workitem_request_queue.pop();
			ray_buffer[front_buffer_id].next_ray++;
		}
	}

public:
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

	const std::string unit_name = "Ray Staging Buffer";
};

}}}