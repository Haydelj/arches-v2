#pragma once

#include "unit-stream-scheduler.hpp"
#include <numeric>

namespace Arches { namespace Units { namespace DualStreaming {

#define STREAM_SCHEDULER_DEBUG_PRINTS false

/*!
* \brief In this function, we deal with the traversal logic and decide the next ray bucket to load from DRAM
*/
void UnitStreamScheduler::_update_scheduler()
{
	if(_scheduler.root_rays_counter == _scheduler.num_root_rays && !_scheduler.segment_state_map[0].parent_finished)
	{
		SegmentState& segment_state = _scheduler.segment_state_map[0];
		segment_state.parent_finished = true;
		// flush all banks
		for(auto& bank : _banks) 
			bank.bucket_flush_queue.push(0);
	}

	// update the segment states to include new buckets
	while(!_scheduler.bucket_allocated_queue.empty())
	{
		uint segment_index = _scheduler.bucket_allocated_queue.front();
		_scheduler.bucket_allocated_queue.pop();
		SegmentState& state = _scheduler.segment_state_map[segment_index];

		//if there is no state entry initilize it
		if(state.total_buckets == 0)
			_scheduler.segment_state_map[segment_index].next_channel = segment_index % _channels.size();

		//increment total buckets
		state.total_buckets++;
	}

	//proccess completed buckets
	while(!_scheduler.bucket_complete_queue.empty())
	{
		uint segment_index = _scheduler.bucket_complete_queue.front();
		_scheduler.bucket_complete_queue.pop();

		SegmentState& state = _scheduler.segment_state_map[segment_index];
		state.retired_buckets++;
		//printf("complete segment %d, total bucket %d, active bucket %d\n", segment_index, segment_state.total_buckets, segment_state.active_buckets);
	}

	// update prefetched segment
	if(_scene_buffer && _scene_buffer->prefetch_complete_sideband.is_read_valid())
	{
		uint segment_index = _scene_buffer->prefetch_complete_sideband.read();
		SegmentState& state = _scheduler.segment_state_map[segment_index];
		state.prefetch_complete = true;
		_scheduler.concurent_prefetches--;
	}

	//prefetch new segments
	for(uint i = 0; i < _scheduler.candidate_segments.size(); ++i)
	{
		uint candidate_segment = _scheduler.candidate_segments[i];
		SegmentState& state = _scheduler.segment_state_map[candidate_segment];
		if(!state.prefetch_issued
			&& state.total_buckets > 0
			&& _scheduler.active_segments < _scheduler.max_active_segments
			&& _scheduler.concurent_prefetches < 4)
		{
			state.prefetch_issued = true;
			_scheduler.active_segments++;

			if(_scene_buffer)
			{
				_scheduler.concurent_prefetches++;
				_scheduler.scene_buffer_command_queue.push({UnitSceneBuffer::Command::Type::PREFETCH, candidate_segment});
			}
			else
			{
				state.prefetch_complete = true;

				if(_l2_cache)
				{
					//while(!_l2_cache_prefetch_queue.empty()) _l2_cache_prefetch_queue.pop();

					paddr_t base_addr = _scheduler.treelet_addr + candidate_segment * rtm::PackedTreelet::SIZE;
					uint rays = candidate_segment == 0 ? _scheduler.num_root_rays : state.num_rays;
					//printf("Prefetching %d to l2 with %d rays:", candidate_segment, rays);
					for(uint i = 0; i < 8; ++i)
					{
						float median_sah = _scheduler.cheat_treelets[candidate_segment].header.median_page_sah[i];
						float num_accesses = rays * median_sah * 0.5f;
						float first_access_chance = rtm::min(1.0, num_accesses);

						float dram_stream_cost = 16;
						float dram_random_cost = 64;
						float cost_diff = dram_stream_cost - first_access_chance * dram_random_cost;
						//printf("%f, ", cost_diff);

						if(cost_diff < 0.0f)
							for(uint j = 0; j < rtm::PackedTreelet::PREFETCH_BLOCK_SIZE; j += _block_size)
								_l2_cache_prefetch_queue.push(base_addr + j);
					}
					//printf("\n");
				}
			}

			log.segments_launched++;
			break;
		}
	}

	//retire segments
	for(uint i = 0; i < _scheduler.candidate_segments.size(); ++i)
	{
		uint candidate_segment = _scheduler.candidate_segments[i];
		SegmentState& state = _scheduler.segment_state_map[candidate_segment];
		if(state.parent_finished
			&& state.child_order_generated
			&& state.retired_buckets == state.total_buckets)
		{
			// remove segment from candidate set
			_scheduler.candidate_segments.erase(_scheduler.candidate_segments.begin() + i--);

			//for all children segments
			rtm::PackedTreelet::Header header = _scheduler.cheat_treelets[candidate_segment].header;
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

			if(state.prefetch_issued)
			{
				if(_scene_buffer)
					_scheduler.scene_buffer_command_queue.push({UnitSceneBuffer::Command::Type::RETIRE, candidate_segment});

				_scheduler.active_segments--;
				if(STREAM_SCHEDULER_DEBUG_PRINTS)
					printf("Segment %d retired after %d buckets\n", candidate_segment, state.total_buckets);
				if(state.total_buckets == 1)
					log.single_bucket_segments++;
			}
			else
			{
				if(STREAM_SCHEDULER_DEBUG_PRINTS)
					printf("Segment %d culled\n", candidate_segment);
			}

			//free the segment state
			_scheduler.segment_state_map.erase(candidate_segment);

			break;
		}
	}

	uint buckets_ready = 0;
	for(uint segment : _scheduler.candidate_segments)
	{
		SegmentState& state = _scheduler.segment_state_map[segment];
		buckets_ready += state.bucket_address_queue.size();
	}

	//schedule new segments
	if(_scheduler.traversal_scheme == 0) //BFS
	{
		if(buckets_ready < 16
			&& _scheduler.root_rays_counter == _scheduler.num_root_rays
			&& _scheduler.candidate_segments.size() < _scheduler.max_active_segments)
		{
			SegmentState& last_segment_state = _scheduler.segment_state_map[_scheduler.last_segment_activated];
			if(!last_segment_state.child_order_generated)
			{
				if(!last_segment_state.parent_finished || last_segment_state.total_buckets > 0)
				{
					rtm::PackedTreelet::Header header = _scheduler.cheat_treelets[_scheduler.last_segment_activated].header;
					for(uint i = 0; i < header.num_children; ++i)
					{
						uint child_id = header.first_child + i;
						SegmentState& child_state = _scheduler.segment_state_map[child_id];
						child_state.depth = last_segment_state.depth + 1;
						_scheduler.traversal_queue.push(child_id);
					}
				}
				last_segment_state.child_order_generated = true;
			}

			if(!_scheduler.traversal_queue.empty())
			{
				uint next_segment = _scheduler.traversal_queue.front();
				_scheduler.traversal_queue.pop();

				SegmentState& next_segment_state = _scheduler.segment_state_map[next_segment];
				_scheduler.candidate_segments.push_back(next_segment);
				_scheduler.last_segment_activated = next_segment;
				if(STREAM_SCHEDULER_DEBUG_PRINTS)
					printf("Segment %d scheduled\n", next_segment);
			}
		}
	}
	else if(_scheduler.traversal_scheme == 1) //DFS
	{
		//if we fall bellow the low water mark try to expand the candidate set
		if(buckets_ready < 16
			&& _scheduler.root_rays_counter == _scheduler.num_root_rays
			//&& _scheduler.active_segments < _scheduler.max_active_segments)
			&& _scheduler.candidate_segments.size() < _scheduler.max_active_segments)
		{
			SegmentState& last_segment_state = _scheduler.segment_state_map[_scheduler.last_segment_activated];
			if(!last_segment_state.child_order_generated)
			{
				rtm::PackedTreelet::Header header = _scheduler.cheat_treelets[_scheduler.last_segment_activated].header;
				std::vector<uint64_t> child_weights(header.num_children);
				std::vector<uint> child_offsets(header.num_children);
				std::iota(child_offsets.begin(), child_offsets.end(), 0);
				for(uint i = 0; i < header.num_children; ++i)
				{
					uint child_id = header.first_child + i;
					SegmentState& child_state = _scheduler.segment_state_map[child_id];
					child_state.depth = last_segment_state.depth + 1;
					if(_scheduler.weight_scheme == 0)      child_weights[i] = child_state.weight; // based on total weight
					else if(_scheduler.weight_scheme == 1) child_weights[i] = child_state.weight / std::max(1ull, child_state.num_rays); // based on average ray weight
					else                                   child_weights[i] = 0.0f; //falls back to order in memory
					child_state.scheduled_weight = child_weights[i];
				}

				std::sort(child_offsets.begin(), child_offsets.end(), [&](const uint& x, const uint& y)
				{
					return child_weights[x] < child_weights[y];
				});

				for(const uint& child_offset : child_offsets)
				{
					uint child_id = header.first_child + child_offset;
					_scheduler.traversal_stack.push(child_id);
				}

				last_segment_state.child_order_generated = true;
			}

			//try to expand working set
			if(!_scheduler.traversal_stack.empty())
			{
				uint next_segment = _scheduler.traversal_stack.top();
				_scheduler.traversal_stack.pop();

				SegmentState& next_segment_state = _scheduler.segment_state_map[next_segment];
				_scheduler.candidate_segments.push_back(next_segment);
				_scheduler.last_segment_activated = next_segment;
				if(STREAM_SCHEDULER_DEBUG_PRINTS)
					printf("Segment %d scheduled, weight %llu\n", next_segment, next_segment_state.scheduled_weight);
			}
		}
	}

	//schedule bucket read requests
	if(!_scheduler.bucket_request_queue.empty())
	{
		uint tm_index = _scheduler.bucket_request_queue.front();
		uint last_segment = _scheduler.last_segment_on_tm[tm_index];

		//find highest priority segment that has rays ready
		uint current_segment = ~0u;
		uint depth = 0;
		for(uint i = 0; i < _scheduler.candidate_segments.size(); ++i)
		{
			uint candidate_segment = _scheduler.candidate_segments[i];
			SegmentState& state = _scheduler.segment_state_map[candidate_segment];

			//last segment match or first match
			// We can only issue rays from prefetched segments
			if(state.prefetch_complete && !state.bucket_address_queue.empty() && (current_segment == ~0u || candidate_segment == last_segment))
			{
				current_segment = candidate_segment;
				depth = state.depth;
			}
		}

		//try to insert the tm into the read queue of one of the channels
		if(current_segment != ~0u)
		{
			SegmentState& state = _scheduler.segment_state_map[current_segment];
			//printf("Segment %d launched, total bucket %d, activated bucket %d \n", current_segment, state.total_buckets, state.active_buckets);
			_scheduler.bucket_request_queue.pop();
			_scheduler.last_segment_on_tm[tm_index] = current_segment;

			paddr_t bucket_adddress = state.bucket_address_queue.front();
			state.bucket_address_queue.pop();

			uint channel_index = _scheduler.memory_managers[0].get_channel(bucket_adddress);
			MemoryManager& memory_manager = _scheduler.memory_managers[channel_index];
			memory_manager.free_bucket(bucket_adddress);

			Channel::WorkItem channel_work_item;
			channel_work_item.type = Channel::WorkItem::Type::READ_BUCKET;
			channel_work_item.address = bucket_adddress;
			channel_work_item.dst_tm = tm_index;

			Channel& channel = _channels[channel_index];
			channel.work_queue.push(channel_work_item);
			log.buckets_launched++;
		}
	}

	//schduel bucket write requests
	_scheduler.bucket_write_cascade.clock();
	if(_scheduler.bucket_write_cascade.is_read_valid(0))
	{
		const RayBucket& bucket = _scheduler.bucket_write_cascade.peek(0);
		uint segment_index = bucket.header.segment_id;
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

		if(++state.next_channel >= _channels.size())
			state.next_channel = 0;

		log.buckets_generated++;
	}
}



/*
* The following part is almost the same with traditional stream scheduler!
*/

void UnitStreamScheduler::clock_rise()
{
	_request_network.clock();

	for(uint i = 0; i < _banks.size(); ++i)
		_proccess_request(i);

	for(uint i = 0; i < _channels.size(); ++i)
		_proccess_return(i);

	_update_scheduler(); // In this process, we deal with traversal logic and decide the next ray bucket to load from DRAM
}

void UnitStreamScheduler::clock_fall()
{
	for(uint i = 0; i < _channels.size(); ++i)
	{
		if(_scheduler.is_complete() && _return_network.is_write_valid(0) && !_scheduler.bucket_request_queue.empty())
		{
			uint tm_index = _scheduler.bucket_request_queue.front();
			_scheduler.bucket_request_queue.pop();

			MemoryReturn ret;
			ret.size = 0;
			ret.port = tm_index;
			_return_network.write(ret, 0); // ray generation
			continue;
		}

		_issue_request(i);
		_issue_return(i);
	}

	if(!_scheduler.scene_buffer_command_queue.empty()
		&& _scene_buffer && _scene_buffer->command_sideband.is_write_valid()
)
	{
		_scene_buffer->command_sideband.write(_scheduler.scene_buffer_command_queue.front());
		_scheduler.scene_buffer_command_queue.pop();
	}

	if(!_l2_cache_prefetch_queue.empty()
		&& _l2_cache && _l2_cache->request_port_write_valid(_l2_cache_port))
	{
		MemoryRequest request;
		request.type = MemoryRequest::Type::PREFETCH;
		request.paddr = _l2_cache_prefetch_queue.front();
		request.port = _l2_cache_port;
		request.size = _block_size;
		request.unit_name = unit_name;

		_l2_cache->write_request(request);
		_l2_cache_prefetch_queue.pop();
	}

	_return_network.clock();
}

void UnitStreamScheduler::_proccess_request(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	//try to flush a bucket from the cache
	while(!bank.bucket_flush_queue.empty())
	{
		uint flush_segment_index = bank.bucket_flush_queue.front();
		if(bank.ray_coalescer.count(flush_segment_index) > 0)
		{
			if(_scheduler.bucket_write_cascade.is_write_valid(bank_index))
			{
				_scheduler.bucket_write_cascade.write(bank.ray_coalescer[flush_segment_index], bank_index);
				bank.ray_coalescer.erase(flush_segment_index);
				bank.bucket_flush_queue.pop();
				return;
			}
		}
		else
		{
			bank.bucket_flush_queue.pop();
		}
	}

	if(!_request_network.is_read_valid(bank_index)) return;

	const StreamSchedulerRequest& req = _request_network.peek(bank_index);
	if(req.type == StreamSchedulerRequest::Type::STORE_WORKITEM)
	{
		uint segment_index = req.swi.segment_id;
		uint weight = 1 << (15 - (std::min(req.swi.order_hint, 15u)));

		//if this segment is not in the coalescer add an entry
		if(bank.ray_coalescer.count(segment_index) == 0 || !bank.ray_coalescer[segment_index].is_full())
		{
			if(req.swi.bray.ray.t_min != req.swi.bray.ray.t_max)
			{
				if(bank.ray_coalescer.count(segment_index) == 0)
				{
					_scheduler.bucket_allocated_queue.push(segment_index);
					bank.ray_coalescer[segment_index].header.segment_id = segment_index;
				}

				bank.ray_coalescer[segment_index].write_ray(req.swi.bray);

				SegmentState& state = _scheduler.segment_state_map[segment_index];
				rtm::PackedTreelet::Header header = _scheduler.cheat_treelets[segment_index].header;
				state.weight += weight;
				state.num_rays++;

				if(segment_index == 0) 
					log.rays++;
			}

			if(segment_index == 0)
				_scheduler.root_rays_counter++;

			log.work_items++;
			_request_network.read(bank_index);
		}

		if(bank.ray_coalescer.count(segment_index) != 0 && bank.ray_coalescer[segment_index].is_full() && _scheduler.bucket_write_cascade.is_write_valid(bank_index))
		{
			//We just filled the write buffer queue it up for streaming and remove from cache
			_scheduler.bucket_write_cascade.write(bank.ray_coalescer[segment_index], bank_index); // Whenever this bucket is full, we send it to dram and create a new bucket next cycle
			bank.ray_coalescer.erase(segment_index);
		}
	}
	else if(req.type == StreamSchedulerRequest::Type::BUCKET_COMPLETE)
	{
		//forward to stream scheduler
		_scheduler.bucket_complete_queue.push(req.bc.segment_id);
		_request_network.read(bank_index);
	}
	else if(req.type == StreamSchedulerRequest::Type::LOAD_BUCKET)
	{
		//forward to stream scheduler
		_scheduler.bucket_request_queue.push(req.port);
		int tm_index = req.port;
		_request_network.read(bank_index);
	}
	else _assert(false);
}

void UnitStreamScheduler::_proccess_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint mem_higher_port_index = channel_index * _main_mem_port_stride + _main_mem_port_offset;
	if(!_main_mem->return_port_read_valid(mem_higher_port_index) || channel.forward_return_valid) return;

	channel.forward_return = _main_mem->read_return(mem_higher_port_index);
	channel.forward_return_valid = true;
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
		req.size = _block_size;
		req.port = mem_higher_port_index;
		req.dst = dst_tm;
		req.paddr = channel.work_queue.front().address + channel.bytes_requested;

		req.unit_name = unit_name;

		_main_mem->write_request(req);

		channel.bytes_requested += _block_size;
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
		req.size = _block_size;
		req.port = mem_higher_port_index;
		req.paddr = channel.work_queue.front().address + channel.bytes_requested;
		std::memcpy(req.data, ((uint8_t*)&bucket) + channel.bytes_requested, _block_size);

		req.unit_name = unit_name;

		_main_mem->write_request(req);

		channel.bytes_requested += _block_size;
		if(channel.bytes_requested == sizeof(RayBucket))
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