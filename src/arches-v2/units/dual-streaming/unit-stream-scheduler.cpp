#pragma once 
#include "unit-stream-scheduler.hpp"


namespace Arches { namespace Units { namespace DualStreaming {

void UnitStreamScheduler::clock_rise()
{
	_request_network.clock();

	for(uint i = 0; i < _banks.size(); ++i)
		_proccess_request(i);

	for(uint i = 0; i < _channels.size(); ++i)
		_proccess_return(i);

	_update_scheduler();
}

void UnitStreamScheduler::clock_fall()
{
	for(uint i = 0; i < _channels.size(); ++i)
	{
		if(_scheduler.is_complete() && _return_network.is_write_valid(0))
		{
			uint tm_index = _scheduler.bucket_request_queue.front();
			_scheduler.bucket_request_queue.pop();

			MemoryReturn ret;
			ret.size = 0;
			ret.port = tm_index;
			_return_network.write(ret, 0);

			continue;
		}

		_issue_request(i);
		_issue_return(i);
	}

	_return_network.clock();
}

void UnitStreamScheduler::_proccess_request(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	//try to flush a bucket from the cache
	if(!bank.bucket_flush_queue.empty())
	{
		uint flush_segment_index = bank.bucket_flush_queue.front();
		if(bank.ray_coalescer.count(flush_segment_index) > 0)
		{
			if(_scheduler.bucket_write_cascade.is_write_valid(bank_index))
			{
				_scheduler.bucket_write_cascade.write(bank.ray_coalescer[flush_segment_index], bank_index);
				bank.ray_coalescer.erase(flush_segment_index);
				bank.bucket_flush_queue.pop();
			}
		}
		else
		{
			bank.bucket_flush_queue.pop();
		}

		return;
	}

	if(!_request_network.is_read_valid(bank_index)) return;
	const StreamSchedulerRequest& req = _request_network.peek(bank_index);

	if(req.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
	{
		uint segment_index = req.segment;

		//if this segment is not in the coalescer add an entry
		if(bank.ray_coalescer.count(segment_index) == 0)
		{
			_scheduler.bucket_allocated_queue.push(segment_index);
			bank.ray_coalescer[segment_index].segment = segment_index;
		}

		RayBucket& write_buffer = bank.ray_coalescer[segment_index];

		if(!write_buffer.is_full())
		{
			write_buffer.write_ray(req.bray);
			_request_network.read(bank_index);
		}

		if(write_buffer.is_full())
		{
			//We just filled the write buffer queue it up for streaming and remove from cache
			if(write_buffer.is_full() && _scheduler.bucket_write_cascade.is_write_valid(bank_index))
			{
				_scheduler.bucket_write_cascade.write(write_buffer, bank_index);
				bank.ray_coalescer.erase(segment_index);
			}
		}
	}
	else if(req.type == StreamSchedulerRequest::Type::BUCKET_COMPLETE)
	{
		//forward to stream scheduler
		_scheduler.bucket_complete_queue.push(req.segment);
		_request_network.read(bank_index);
	}
	else if(req.type == StreamSchedulerRequest::Type::LOAD_BUCKET)
	{
		//forward to stream scheduler
		_scheduler.bucket_request_queue.push(req.port);
		_request_network.read(bank_index);
	}
	else assert(false);
}

void UnitStreamScheduler::_proccess_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint mem_higher_port_index = channel_index * _main_mem_port_stride + _main_mem_port_offset;
	if(!_main_mem->return_port_read_valid(mem_higher_port_index) || channel.forward_return_valid) return;

	channel.forward_return = _main_mem->read_return(mem_higher_port_index);
	channel.forward_return_valid = true;
}

void UnitStreamScheduler::_update_scheduler()
{
	//update the segment states to include new buckets
	while(!_scheduler.bucket_allocated_queue.empty())
	{
		uint segment_index = _scheduler.bucket_allocated_queue.front();
		_scheduler.bucket_allocated_queue.pop();
		SegmentState& state = _scheduler.segment_state_map[segment_index];

		//if there is no state entry initilize it
		if(state.total_buckets == 0)
			_scheduler.segment_state_map[segment_index].next_channel = segment_index % NUM_DRAM_CHANNELS;

		//increment total buckets
		state.total_buckets++;
	}

	//proccess completed buckets
	while(!_scheduler.bucket_complete_queue.empty())
	{
		uint segment_index = _scheduler.bucket_complete_queue.front();
		SegmentState& segment_state = _scheduler.segment_state_map[segment_index];

		segment_state.total_buckets--;
		segment_state.active_buckets--;

		//all remaining buckets are active
		if(segment_state.parent_finished && segment_state.total_buckets == 0)
		{
			//segment is complete

			//remove from the active segments
			_scheduler.active_segments.erase(segment_index);
			printf("Segment %d retired\n", segment_index);

			//free the segment state
			_scheduler.segment_state_map.erase(segment_index);

			//for all children segments
			Treelet::Header header = _scheduler.cheat_treelets[segment_index].header;
			for(uint i = 0; i < header.num_children; ++i)
			{
				//mark the child as parent finsihed
				uint child_segment_index = header.first_child + i;
				SegmentState& child_segment_state = _scheduler.segment_state_map[child_segment_index];
				child_segment_state.parent_finished = true;

				//flush the child from the coalescer
				uint child_bank_index = child_segment_index % _banks.size();
				Bank& child_bank = _banks[child_bank_index];
				child_bank.bucket_flush_queue.push(child_segment_index);
			}

			break; // Why break here?
		}

		_scheduler.bucket_complete_queue.pop();
	}

	//try to pick a new segment to prefetch
	if((_scheduler.active_segments.size()) < MAX_ACTIVE_SEGMENTS && !_scheduler.traversal_queue.empty())
	{
		//we have room in the working set and a segment in the traversal queue try to expand working set
		do
		{
			uint next_segment = _scheduler.traversal_queue.front();
			_scheduler.traversal_queue.pop();
			SegmentState& state = _scheduler.segment_state_map[next_segment];

			//if this segment is active or the parent is still traversing
			if(state.total_buckets > 0 || !state.parent_finished)
			{
				// prefecth the segment
				//paddr_t address = _scheduler.treelet_addr + sizeof(Treelet) * next_segment;
				//for(uint i = 0; i < sizeof(Treelet); i += ROW_BUFFER_SIZE)
				//{
				//	Channel::WorkItem channel_work_item;
				//	channel_work_item.type = Channel::WorkItem::Type::READ_SEGMENT;
				//	channel_work_item.address = address + i;
				//
				//	uint channel_index = calcDramAddr(address).channel;
				//	Channel& channel = _channels[channel_index];
				//	channel.work_queue.push(channel_work_item);
				//}

				//TODO: wait for prefetch to complete before adding to candidate set
				_scheduler.candidate_segments.push_back(next_segment);

				//Add the segment to the active set
				_scheduler.active_segments.insert(next_segment);
				printf("Segment %d scheduled\n", next_segment);
				break; // Why do you need to break here?
			}
		} 
		while(!_scheduler.traversal_queue.empty());
	}

	//schedule bucket read requests
	if(!_scheduler.bucket_request_queue.empty())
	{
		uint tm_index = _scheduler.bucket_request_queue.front();
		uint last_segment = _scheduler.last_segment_on_tm[tm_index];

		//find highest priority segment that has rays ready
		uint current_segment = ~0u;
		for(uint i = 0; i < _scheduler.candidate_segments.size(); ++i)
		{
			uint candidate_segment = _scheduler.candidate_segments[i];
			SegmentState& state = _scheduler.segment_state_map[candidate_segment];

			if(state.active_buckets != state.total_buckets)
			{
				//last segment match or first match
				if(!state.bucket_address_queue.empty() && (current_segment == ~0u || candidate_segment == last_segment))
				{
					current_segment = candidate_segment;
				}
			}
			else if(state.parent_finished)
			{
				//No more rays to traverse. Erase the segment from the candidate set
				_scheduler.candidate_segments.erase(_scheduler.candidate_segments.begin() + i--);

				if(state.total_buckets == 0)
				{
					//If no ray were launched erase the candidate from the active set as well 
					_scheduler.active_segments.erase(candidate_segment); 
				}
				else
				{
					//Otherwise queue up the children segments. The segment will be removed from active set when all rays complete
					
					//for all children segments
					Treelet::Header header = _scheduler.cheat_treelets[candidate_segment].header;
					for(uint i = 0; i < header.num_children; ++i)
					{
						//add child to the traversal qeueue
						uint child_segment_index = header.first_child + i;
						SegmentState& child_segment_state = _scheduler.segment_state_map[child_segment_index];
						_scheduler.traversal_queue.emplace(child_segment_index);
					}
				}
			}
		}

		//try to insert the tm into the read queue of one of the channels
		if(current_segment != ~0u)
		{
			printf("Segment %d launched\n", current_segment);
			_scheduler.bucket_request_queue.pop();
			_scheduler.last_segment_on_tm[tm_index] = current_segment;

			SegmentState& state = _scheduler.segment_state_map[current_segment];
			paddr_t bucket_adddress = state.bucket_address_queue.front();
			state.bucket_address_queue.pop();

			uint channel_index = calcDramAddr(bucket_adddress).channel;
			MemoryManager& memory_manager = _scheduler.memory_managers[channel_index];
			memory_manager.free_bucket(bucket_adddress);

			Channel::WorkItem channel_work_item;
			channel_work_item.type = Channel::WorkItem::Type::READ_BUCKET;
			channel_work_item.address = bucket_adddress;
			channel_work_item.dst_tm = tm_index;

			Channel& channel = _channels[channel_index];
			channel.work_queue.push(channel_work_item);

			state.active_buckets++;
		}
	}

	//schduel bucket write requests
	_scheduler.bucket_write_cascade.clock();
	if(_scheduler.bucket_write_cascade.is_read_valid(0))
	{
		const RayBucket& bucket = _scheduler.bucket_write_cascade.peek(0);
		uint segment_index = bucket.segment;
		SegmentState& state = _scheduler.segment_state_map[segment_index];

		uint channel_index = state.next_channel;
		MemoryManager& memory_manager = _scheduler.memory_managers[channel_index];
		paddr_t bucket_adddress = memory_manager.alloc_bucket();
		state.bucket_address_queue.push(bucket_adddress);

		Channel::WorkItem channel_work_item;
		channel_work_item.type = Channel::WorkItem::Type::WRITE_BUCKET;
		channel_work_item.address = bucket_adddress;
		channel_work_item.bucket = bucket;
		_scheduler.bucket_write_cascade.read(0);

		Channel& channel = _channels[channel_index];
		channel.work_queue.push(channel_work_item);

		if(++state.next_channel >= NUM_DRAM_CHANNELS)
			state.next_channel = 0;
	}
}

void UnitStreamScheduler::_issue_request(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint mem_higher_port_index = channel_index * _main_mem_port_stride + _main_mem_port_offset;
	if(!_main_mem->request_port_write_valid(mem_higher_port_index)) return;

	if(channel.work_queue.empty()) return;


	if(channel.work_queue.front().type == Channel::WorkItem::Type::READ_BUCKET)
	{
		uint dst_tm = channel.work_queue.front().dst_tm;

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = CACHE_BLOCK_SIZE;
		req.port = mem_higher_port_index;
		req.dst = dst_tm;
		req.paddr = channel.work_queue.front().address + channel.bytes_requested;
		_main_mem->write_request(req, req.port);

		channel.bytes_requested += CACHE_BLOCK_SIZE;
		if(channel.bytes_requested == sizeof(RayBucket))
		{
			channel.bytes_requested = 0;
			channel.work_queue.pop();
		}
	}
	else if(channel.work_queue.front().type == Channel::WorkItem::Type::WRITE_BUCKET)
	{
		RayBucket& bucket = channel.work_queue.front().bucket;

		MemoryRequest req;
		req.type = MemoryRequest::Type::STORE;
		req.size = CACHE_BLOCK_SIZE;
		req.port = mem_higher_port_index;
		req.write_mask = generate_nbit_mask(CACHE_BLOCK_SIZE);
		req.paddr = channel.work_queue.front().address + channel.bytes_requested;
		std::memcpy(req.data, ((uint8_t*)&bucket) + channel.bytes_requested, CACHE_BLOCK_SIZE);
		_main_mem->write_request(req, req.port);

		channel.bytes_requested += CACHE_BLOCK_SIZE;
		if(channel.bytes_requested == sizeof(RayBucket))
		{
			channel.bytes_requested = 0;
			channel.work_queue.pop();
		}
	}
	else if(channel.work_queue.front().type == Channel::WorkItem::Type::READ_SEGMENT)
	{
		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = CACHE_BLOCK_SIZE;
		req.port = mem_higher_port_index;
		req.dst = _return_network.num_sinks();
		req.paddr = channel.work_queue.front().address + channel.bytes_requested;
		_main_mem->write_request(req, req.port);

		channel.bytes_requested += CACHE_BLOCK_SIZE;
		if(channel.bytes_requested == ROW_BUFFER_SIZE)
		{
			channel.bytes_requested = 0;
			channel.work_queue.pop();
		}
	}
}

void UnitStreamScheduler::_issue_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	if(!_return_network.is_write_valid(channel_index)) return;

	if(channel.forward_return_valid)
	{
		if(channel.forward_return.dst < _return_network.num_sinks())
		{
			//forward to ray buffer
			channel.forward_return.port = channel.forward_return.dst;
			_return_network.write(channel.forward_return, channel_index);
			channel.forward_return_valid = false;
		}
		else
		{
			//forward to scene buffer
			channel.forward_return_valid = false;
		}
	}
}

}}}