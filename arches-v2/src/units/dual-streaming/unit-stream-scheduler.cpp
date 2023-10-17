#pragma once 
#include "unit-stream-scheduler.hpp"


namespace Arches { namespace Units { namespace DualStreaming {

void UnitStreamScheduler::proccess_request(uint channel_index)
{
	if(!_request_network.is_read_valid(channel_index)) return;
	const StreamSchedulerRequest& req = _request_network.peek(channel_index);
	Channel& channel = _channels[channel_index];

	if(req.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
	{
		uint segment_index = req.work_item.segment;
		SegmentState& segment_state = channel.segment_state_map[segment_index];

		if(segment_state.write_buffer.is_full()) return;

		segment_state.write_buffer.write_ray(req.work_item.bray);
		_request_network.read(channel_index);

		if(req.work_item.segment == 0)
			if(++_root_rays_in_flight == 1024 * 1024)
				channel.segment_state_map[0].parent_finished = true;

		//We just filled the write buffer queue it up for streaming
		if(segment_state.write_buffer.is_full())
		{
			channel.bucket_write_queue.push(segment_index);
			//TODO: allocate a new write buffer for this segment instead of stalling while we wait to stream
		}
	}
	else if(req.type == StreamSchedulerRequest::Type::LOAD_BUCKET)
	{
		//Insert the tm in the read queue
		_tm_bucket_queue.emplace(req.port);
		_request_network.read(channel_index);
	}
}

void UnitStreamScheduler::proccess_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint mem_higher_port_index = channel_index * _main_mem_port_stride + _main_mem_port_offset;
	if(!_main_mem->return_port_read_valid(mem_higher_port_index) || channel.forward_return.type != MemoryReturn::Type::NA) return;

	channel.forward_return = _main_mem->read_return(mem_higher_port_index);
}

void UnitStreamScheduler::issue_request(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint mem_higher_port_index = channel_index * _main_mem_port_stride + _main_mem_port_offset;
	if(!_main_mem->request_port_write_valid(mem_higher_port_index)) return;

	if(channel.stream_state == Channel::StreamState::NA)
	{
		if(!channel.bucket_write_queue.empty())
		{ 
			uint segment_index = channel.bucket_write_queue.front();
			SegmentState& segment_state = channel.segment_state_map[segment_index];

			channel.current_bucket.segment_index = segment_index;
			channel.current_bucket.lines_requested = 0;

			//allocate a new bucket to write into
			if(segment_state.tail_bucket_address == 0)
				channel.bucket_read_queue.push(segment_index);

			segment_state.write_buffer.next_bucket = segment_state.tail_bucket_address;
			segment_state.tail_bucket_address = channel.alloc_bucket();
		}
		else if(!channel.bucket_read_queue.empty() && !_tm_bucket_queue.empty())
		{
			uint segment_index = channel.bucket_read_queue.front();
			SegmentState& segment_state = channel.segment_state_map[segment_index];

			//initilize bucket read stream
			channel.stream_state = Channel::StreamState::READ_BUCKET;
			channel.current_bucket.segment_index = segment_index;
			channel.current_bucket.lines_requested = 0;
			channel.current_bucket.dst_tm = _tm_bucket_queue.front();
			_tm_bucket_queue.pop();
		}
	}

	if(channel.stream_state == Channel::StreamState::WRITE_BUCKET)
	{
		SegmentState& segment_state = channel.segment_state_map[channel.current_bucket.segment_index];
		paddr_t bucket_address = segment_state.tail_bucket_address;

		MemoryRequest req;
		req.type = MemoryRequest::Type::STORE;
		req.size = CACHE_BLOCK_SIZE;
		req.port = mem_higher_port_index;
		req.write_mask = generate_nbit_mask(CACHE_BLOCK_SIZE);
		req.paddr = bucket_address + CACHE_BLOCK_SIZE * channel.current_bucket.lines_requested;
		std::memcpy(req.data, ((uint8_t*)&segment_state.write_buffer) + CACHE_BLOCK_SIZE * channel.current_bucket.lines_requested, CACHE_BLOCK_SIZE);
		_main_mem->write_request(req, req.port);

		channel.current_bucket.lines_requested++;
		if(channel.current_bucket.lines_requested == (sizeof(RayBucket) / CACHE_BLOCK_SIZE))
		{
			channel.stream_state = Channel::StreamState::NA;
			segment_state.write_buffer.num_rays = 0;
		}
	}
	else if(channel.stream_state == Channel::StreamState::READ_BUCKET)
	{
		SegmentState& segment_state = channel.segment_state_map[channel.current_bucket.segment_index];
		paddr_t bucket_address = segment_state.tail_bucket_address;

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = CACHE_BLOCK_SIZE;
		req.port = mem_higher_port_index;
		req.dst = channel.current_bucket.dst_tm;
		req.paddr = bucket_address + CACHE_BLOCK_SIZE * channel.current_bucket.lines_requested;
		_main_mem->write_request(req, req.port);

		channel.current_bucket.lines_requested++;
		if(channel.current_bucket.lines_requested == (sizeof(RayBucket) / CACHE_BLOCK_SIZE))
		{
			channel.stream_state = Channel::StreamState::NA;

			//update tail pointer
			paddr_t next_bucket_addr;
			_main_mem->direct_read(&next_bucket_addr, sizeof(paddr_t), segment_state.tail_bucket_address);
			segment_state.tail_bucket_address = next_bucket_addr;

			//free the bucket for resuse 
			channel.free_bucket(bucket_address);
		}
	}
}

void UnitStreamScheduler::issue_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	if(channel.forward_return.type == MemoryReturn::Type::NA || !_return_network.is_write_valid(channel_index)) return;

	channel.forward_return.port = channel.forward_return.dst;
	_return_network.write(channel.forward_return, channel_index);
	channel.forward_return.type = MemoryReturn::Type::NA;
}

}}}