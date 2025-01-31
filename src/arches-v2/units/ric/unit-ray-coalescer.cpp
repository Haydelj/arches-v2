#pragma once

#include "units/ric/unit-ray-coalescer.hpp"
#include <numeric>

namespace Arches { namespace Units { namespace RIC {

constexpr bool DEBUG_PRINTS = false;

void UnitRayCoalescer::_update_scheduler()
{
	if(!prefetched_root)
	{
		_prefetch(0);
		prefetched_root = true;
	}

	//mark root as finished
	if(_scheduler.root_rays_counter == _scheduler.num_root_rays && !_scheduler.segment_state_map[0].parent_finished)
	{
		_scheduler.segment_state_map[0].parent_finished = true;
		bucket_flush_queue.push(0);
	}

	// update the segment states to include new buckets
	while(!_scheduler.bucket_allocated_queue.empty())
	{
		uint segment_index = _scheduler.bucket_allocated_queue.front();
		_scheduler.bucket_allocated_queue.pop();
		SegmentState& state = _scheduler.segment_state_map[segment_index];

		//if there is no state entry initilize it
		if(state.total_buckets == 0)
			state.next_channel = segment_index % _channels.size();

		state.total_buckets++;
	}

	//retire buckets
	while(!_scheduler.bucket_complete_queue.empty())
	{
		uint segment_index = _scheduler.bucket_complete_queue.front();
		_scheduler.bucket_complete_queue.pop();

		SegmentState& state = _scheduler.segment_state_map[segment_index];
		state.retired_buckets++;
		if(DEBUG_PRINTS)
			printf("completed bucket %d, total buckets %d, retired buckets %d\n", segment_index, state.total_buckets, state.retired_buckets);
	}

	//retire segments
	for(uint i = 0; i < _scheduler.active_segments.size(); ++i)
	{
		uint candidate_segment = _scheduler.active_segments[i];
		SegmentState& state = _scheduler.segment_state_map[candidate_segment];
		if(state.parent_finished && state.children_scheduled && state.retired_buckets == state.total_buckets)
		{
			_scheduler.active_segments.erase(_scheduler.active_segments.begin() + i--);

			rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[candidate_segment].header;
			for(uint i = 0; i < header.num_children; ++i)
			{
				//mark the child as parent finsihed
				uint child_segment_index = header.first_child + i;
				SegmentState& child_segment_state = _scheduler.segment_state_map[child_segment_index];
				child_segment_state.parent_finished = true;

				//flush the child from the coalescer
				bucket_flush_queue.push(child_segment_index);
			}

			_scheduler.active_segments_size -= header.bytes;
			_scheduler.retired_segments++;

			if(DEBUG_PRINTS)
				printf("Segment %d retired after %d buckets\n", candidate_segment, state.total_buckets);
			
			if(state.total_buckets == 1)
				log.single_bucket_segments++;

			//free the segment state
			_scheduler.segment_state_map.erase(candidate_segment);

			break;
		}
	}

	//schedule new segments
	if(_scheduler.root_rays_counter == _scheduler.num_root_rays)
	if(_scheduler.traversal_scheme == 0) //BFS
	{
		uint buckets_ready = 0;
		for(uint segment : _scheduler.active_segments)
		{
			SegmentState& state = _scheduler.segment_state_map[segment];
			buckets_ready += state.bucket_address_queue.size();
		}

		//If we expand the active set make sure to schedule the children of the previous segment added
		SegmentState& last_segment_state = _scheduler.segment_state_map[_scheduler.last_segment_activated];
		if(!last_segment_state.children_scheduled)
		{
			if(!last_segment_state.parent_finished || last_segment_state.total_buckets > 0)
			{
				rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[_scheduler.last_segment_activated].header;
				for(uint i = 0; i < header.num_children; ++i)
				{
					uint child_id = header.first_child + i;
					SegmentState& child_state = _scheduler.segment_state_map[child_id];
					child_state.depth = last_segment_state.depth + 1;
					_scheduler.traversal_queue.push(child_id);
				}
			}
			last_segment_state.children_scheduled = true;
		}

		if(last_segment_state.children_scheduled && !_scheduler.traversal_queue.empty())
		{
			uint next_segment = _scheduler.traversal_queue.front();
			rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[next_segment].header;
			if((header.bytes + _scheduler.active_segments_size) <= _scheduler.max_active_segments_size || (buckets_ready < 1))
			{
				_scheduler.active_segments_size += header.bytes;
				_scheduler.traversal_queue.pop();

				SegmentState& next_segment_state = _scheduler.segment_state_map[next_segment];
				_scheduler.active_segments.push_back(next_segment);
				_scheduler.last_segment_activated = next_segment;
				if(DEBUG_PRINTS)
					printf("Segment %d scheduled\n", next_segment);

				//queue prefetch
				_prefetch(next_segment);
			}
		}
	}
	else if(_scheduler.traversal_scheme == 1) //DFS
	{
		uint buckets_ready = 0;
		for(uint segment : _scheduler.active_segments)
		{
			SegmentState& state = _scheduler.segment_state_map[segment];
			buckets_ready += state.bucket_address_queue.size();
		}

		//if we fall bellow the low watermark try to expand the candidate set
		SegmentState& last_segment_state = _scheduler.segment_state_map[_scheduler.last_segment_activated];
		if(!last_segment_state.children_scheduled)
		{
			rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[_scheduler.last_segment_activated].header;
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
				else if(_scheduler.weight_scheme == 2) child_weights[i] = 1.0 / child_state.num_rays;
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

			last_segment_state.children_scheduled = true;
		}

		//try to expand working set
		if(last_segment_state.children_scheduled && !_scheduler.traversal_stack.empty())
		{
			uint next_segment = _scheduler.traversal_stack.top();
			rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[next_segment].header;
			if((header.bytes + _scheduler.active_segments_size) <= _scheduler.max_active_segments_size || (buckets_ready < 1))
			{
				_scheduler.active_segments_size += header.bytes;
				_scheduler.traversal_stack.pop();

				SegmentState& next_segment_state = _scheduler.segment_state_map[next_segment];
				_scheduler.active_segments.push_back(next_segment);
				_scheduler.last_segment_activated = next_segment;
				if(header.num_children == 0) last_segment_state.children_scheduled = true;
				if(DEBUG_PRINTS)
					printf("Segment %d scheduled, weight %llu\n", next_segment, next_segment_state.scheduled_weight);

				_prefetch(next_segment);
			}
		}
	}

	//schedule bucket read requests
	if(!_scheduler.bucket_request_queue.empty())
	{
		uint tm_index = _scheduler.bucket_request_queue.front();
		uint last_segment = _scheduler.last_segment_on_tm[tm_index];

		//find highest priority segment that has rays ready
		uint current_depth = ~0;
		uint current_segment = ~0u;
		float  current_score = INFINITY;
		for(uint i = 0; i < _scheduler.active_segments.size(); ++i)
		{
			uint candidate_segment = _scheduler.active_segments[i];
			SegmentState& state = _scheduler.segment_state_map[candidate_segment];
			if(state.bucket_address_queue.empty()) continue;

			if(state.num_tms == 0)
			{
				current_segment = candidate_segment;
				break;
			}

			if(candidate_segment == last_segment)
			{
				current_segment = candidate_segment;
				break;
			}

			float score = state.num_tms / (float)state.bucket_address_queue.size();
			//score *= (1 << state.depth);
			if(score < current_score)
			{
				current_depth = state.depth;
				current_score = score;
				current_segment = candidate_segment;
			}
		}

		//try to insert the tm into the read queue of one of the channels
		if(current_segment != ~0u)
		{
			SegmentState& state = _scheduler.segment_state_map[current_segment];
			//printf("Segment %d launched, total bucket %d, activated bucket %d \n", current_segment, state.total_buckets, state.active_buckets);
			_scheduler.bucket_request_queue.pop();
			_scheduler.bucket_request_set.erase(tm_index);

			std::set<uint> active_segemnts;
			if(_scheduler.last_segment_on_tm[tm_index] != current_segment)
			{
				if(_scheduler.last_segment_on_tm[tm_index] != ~0u)
					_scheduler.segment_state_map[_scheduler.last_segment_on_tm[tm_index]].num_tms--;

				_scheduler.last_segment_on_tm[tm_index] = current_segment;
				state.num_tms++;
			}

			for(uint i = 0; i < _scheduler.last_segment_on_tm.size(); ++i)
			{
				if(i % 16 == 0) printf("\n");
				printf("\033[3%dm", (rtm::RNG::hash(_scheduler.last_segment_on_tm[i] + 1) % 8));
				printf("%04d", _scheduler.last_segment_on_tm[i]);
				printf("\033[0m ");
				if(active_segemnts.count(_scheduler.last_segment_on_tm[i]) == 0)
					active_segemnts.insert(_scheduler.last_segment_on_tm[i]);
			}
			printf("\n");
			printf("%03d, %03d(%2.2fMB), %03d", _scheduler.retired_segments, _scheduler.active_segments.size(), _scheduler.active_segments_size / ((float)(1 << 20)), active_segemnts.size());
			printf("\r\033[9A");


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

	//scheduel bucket write requests
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

	for(uint i = 0; i < _scheduler.prefetch_queues.size(); ++i)
	{
		uint port = 128 + i * 4;
		if(!_scheduler.prefetch_queues[i].empty() && _scheduler.l2_cache->request_port_write_valid(port))
		{
			MemoryRequest req;
			req.type = MemoryRequest::Type::PREFECTH;
			req.size = _block_size;
			req.port = port;
			req.paddr = _scheduler.prefetch_queues[i].front();
			_scheduler.prefetch_queues[i].pop();
			_scheduler.l2_cache->write_request(req);
		}
	}
}

void UnitRayCoalescer::_prefetch(uint segment)
{
	rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[segment].header;
	paddr_t start = _scheduler.treelet_addr + segment * sizeof(rtm::CompressedWideTreeletBVH::Treelet);
	paddr_t end = align_to(_block_size, start + header.bytes);
	start += sizeof(rtm::CompressedWideTreeletBVH::Treelet::Header);
	for(paddr_t addr = start; addr < end; addr += _block_size)
	{
		uint qi = (addr / _scheduler.sector_size) % _scheduler.prefetch_queues.size();
		_scheduler.prefetch_queues[qi].push(addr);
	} 
}

/*
* The following part is almost the same with traditional stream scheduler!
*/

void UnitRayCoalescer::clock_rise()
{
	_request_network.clock();

	for(uint i = 0; i < _request_network.num_sinks(); ++i)
		_proccess_request(i);

	for(uint i = 0; i < _channels.size(); ++i)
		_proccess_return(i);

	_update_scheduler(); // In this process, we deal with traversal logic and decide the next ray bucket to load from DRAM
}

void UnitRayCoalescer::clock_fall()
{
	for(uint i = 0; i < _channels.size(); ++i)
	{
		if(_scheduler.is_complete() && _return_network.is_write_valid(0) && !_scheduler.bucket_request_queue.empty())
		{
			uint tm_index = _scheduler.bucket_request_queue.front();
			_scheduler.bucket_request_queue.pop();
			_scheduler.bucket_request_set.erase(tm_index);

			MemoryReturn ret;
			ret.size = 0;
			ret.port = tm_index;
			_return_network.write(ret, 0); // ray generation
			continue;
		}

		_issue_request(i);
		_issue_return(i);
	}

	_return_network.clock();
}

void UnitRayCoalescer::_proccess_request(uint bank_index)
{
	//try to flush a bucket from the cache
	while(!bucket_flush_queue.empty())
	{
		uint flush_segment_index = bucket_flush_queue.front();
		if(ray_coalescer.count(flush_segment_index) > 0)
		{
			if(_scheduler.bucket_write_cascade.is_write_valid(bank_index))
			{
				_scheduler.bucket_write_cascade.write(ray_coalescer[flush_segment_index], bank_index);
				ray_coalescer.erase(flush_segment_index);
				bucket_flush_queue.pop();
			}
			return;
		}
		else
		{
			bucket_flush_queue.pop();
		}
	}

	if(!_request_network.is_read_valid(bank_index)) return;

	const Request& req = _request_network.peek(bank_index);
	if(req.type == Request::Type::STORE_WORKITEM)
	{
		uint segment_index = req.swi.segment_id;
		uint weight = 1 << (15 - (std::min((uint)req.swi.order_hint, 15u)));

		//if this segment is not in the coalescer add an entry
		if(ray_coalescer.count(segment_index) == 0 || !ray_coalescer[segment_index].is_full())
		{
			if(req.swi.ray_id != ~0ull)
			{
				if(ray_coalescer.count(segment_index) == 0)
				{
					_scheduler.bucket_allocated_queue.push(segment_index);
					ray_coalescer[segment_index].header.segment_id = segment_index;
				}

				ray_coalescer[segment_index].write_ray(req.swi.ray_id);

				SegmentState& state = _scheduler.segment_state_map[segment_index];
				rtm::WideTreeletBVH::Treelet::Header header = _scheduler.cheat_treelets[segment_index].header;
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

		if(ray_coalescer.count(segment_index) != 0 && ray_coalescer[segment_index].is_full() && _scheduler.bucket_write_cascade.is_write_valid(bank_index))
		{
			//We just filled the write buffer queue it up for streaming and remove from cache
			_scheduler.bucket_write_cascade.write(ray_coalescer[segment_index], bank_index); // Whenever this bucket is full, we send it to dram and create a new bucket next cycle
			ray_coalescer.erase(segment_index);
		}
	}
	else if(req.type == Request::Type::BUCKET_COMPLETE)
	{
		//forward to stream scheduler
		_scheduler.bucket_complete_queue.push(req.bc.segment_id);
		_request_network.read(bank_index);
	}
	else if(req.type == Request::Type::LOAD_BUCKET)
	{
		//forward to stream scheduler
		_scheduler.bucket_request_queue.push(req.port);
		_scheduler.bucket_request_set.insert(req.port);
		int tm_index = req.port;
		_request_network.read(bank_index);
	}
	else _assert(false);
}

void UnitRayCoalescer::_proccess_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint mem_higher_port_index = channel_index * _main_mem_port_stride + _main_mem_port_offset;
	if(!_main_mem->return_port_read_valid(mem_higher_port_index) || channel.forward_return_valid) return;

	channel.forward_return = _main_mem->read_return(mem_higher_port_index);
	channel.forward_return_valid = true;
}

void UnitRayCoalescer::_issue_request(uint channel_index)
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
		req.dst.push(dst_tm, 8);
		req.paddr = channel.work_queue.front().address + channel.bytes_requested;
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
		_main_mem->write_request(req);

		channel.bytes_requested += _block_size;
		if(channel.bytes_requested == sizeof(RayBucket))
		{
			channel.bytes_requested = 0;
			channel.work_queue.pop();
		}
	}
}

void UnitRayCoalescer::_issue_return(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	if(!_return_network.is_write_valid(channel_index)) return;

	if(channel.forward_return_valid)
	{
		channel.forward_return.port = channel.forward_return.dst.pop(8);
		_return_network.write(channel.forward_return, channel_index);
		channel.forward_return_valid = false;
	}
}

}}}