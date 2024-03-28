#include "unit-scene-buffer.hpp"

namespace Arches {
namespace Units {
namespace DualStreaming {


void UnitSceneBuffer::process_finish()
{
	if (!segment_finished.empty())
	{
		// note: if a segment is finished, we then reject all following returns of this segment
		uint segment_index = segment_finished.front();
		finished[segment_index] = true;
		segment_finished.pop();
		//printf("Scene Buffer Finish Segment %d\n", segment_index);

		if (!segment_offset.count(segment_index)) return;
		SegmentState& segment = segments[segment_offset[segment_index]];
		segment.state = SegmentState::State::EMPTY;
		segment.segment_index = ~0u;
		segment.byte_requested = 0;
		segment.byte_returned = 0;
		segment_offset.erase(segment_index);
	}
}

uint UnitSceneBuffer::find_segment_slot()
{
	for (uint i = 0; i < segments.size(); i++)
	{
		if (segments[i].state == SegmentState::State::EMPTY)
		{
			assert(segments[i].segment_index == ~0u);
			return i;
		}
	}
	return ~0u;
}

void UnitSceneBuffer::process_prefetch()
{
	if (!prefetch_request.empty())
	{
		uint segment_index = prefetch_request.front();
		// need to find a slot for this segment
		assert(!segment_offset.count(segment_index));

		if (finished.count(segment_index))
		{
			prefetch_request.pop();
			return;
		}
		uint new_segment_slot = find_segment_slot();
		if (new_segment_slot != ~0u)
		{
			//std::cout << "new segment slot for segment " << segment_index << ": " << new_segment_slot << '\n';
			prefetch_request.pop();

			segment_offset[segment_index] = new_segment_slot;
			// distribute the prefetch request to multiple bank
			paddr_t segment_address = treelet_start + 1ull * segment_index * treelet_size;
			uint start_channel = calcDramAddr(segment_address).channel;

			uint num_channels_to_across = treelet_size / dram_row_size;
			for (uint i = 0; i < num_channels_to_across; i++)
			{
				uint channel_index = (start_channel + i) % NUM_DRAM_CHANNELS;
				
				channels[channel_index].prefetch_queue.push({ segment_address});
				segment_address += dram_row_size;
			}

			SegmentState& segment = segments[new_segment_slot];
			segment.state = SegmentState::State::PREFETCHING;
			segment.segment_index = segment_index;
		}
	}
}

void UnitSceneBuffer::process_internal_return(uint bank_index)
{
	Bank& bank = banks[bank_index];
	if (internal_return_crossbar.is_read_valid(bank_index))
	{
		MemoryReturn ret = internal_return_crossbar.read(bank_index);
		assert(ret.size == CACHE_BLOCK_SIZE);
		uint segment_index = calculate_segment(ret.paddr);

		if (!segment_offset.count(segment_index))
		{
			return;
		}
		SegmentState& segment = segments[segment_offset[segment_index]];
		uint treelet_offset = offset_in_treelet(ret.paddr);
		
		assert(segment.state == SegmentState::State::PREFETCHING);

		segment.returned_blocks.insert(ret.paddr);
		std::memcpy(segment.data_u8 + treelet_offset, ret.data, ret.size);
		segment.byte_returned += ret.size;
		
		assert(segment.byte_returned <= treelet_size);

		if (segment.byte_returned >= treelet_size * 0.8 && allow_wait && !segment_signal_returned[segment_index])
		{ 
			segment_signal_returned[segment_index] = true;
			bank.segment_prefetched = segment_index;
		}
		if (segment.byte_returned == treelet_size)
		{
			segment.state = SegmentState::State::LOADED;
			if (!segment_signal_returned[segment_index])
			{
				bank.segment_prefetched = segment_index;
				segment_signal_returned[segment_index] = true;
			}
		}
	}
}

void UnitSceneBuffer::process_requests(uint bank_index)
{
	Bank& bank = banks[bank_index];
	if (request_network.is_read_valid(bank_index) && !bank.has_return)
	{
		// TO DO: add pipeline
		// Since the data must be here, we simply return data here
		const SceneBufferLoadRequest& req = request_network.peek(bank_index);
		assert(get_bank(req.paddr) == bank_index);

		uint segment_index = calculate_segment(req.paddr);
		assert(segment_offset.count(segment_index));

		SegmentState& segment = segments[segment_offset[segment_index]];
		if(!allow_wait) assert(segment.state == SegmentState::State::LOADED);

		uint data_offset = offset_in_treelet(req.paddr);

		assert(data_offset + req.size - 1 < treelet_size);
		assert(get_bank(req.paddr + req.size - 1) == bank_index); // requested data should be all in this bank

		// Check whether the corresponding data has been returned
		auto& returned_set = segment.returned_blocks;
		assert(!returned_set.empty()); // at least some data has been returned
		auto it = returned_set.upper_bound(req.paddr);
		if (it == returned_set.begin()) return;
		it--;
		if (*it + CACHE_BLOCK_SIZE <= req.paddr) return;
		// We are sure the needed data is returned

		//if (return_network.is_write_valid(req.port))
		//{
			bank.has_return = true;
			request_network.read(bank_index);
			MemoryReturn ret;
			ret.port = req.port;
			ret.size = req.size;
			assert(ret.size == CACHE_BLOCK_SIZE);
			std::memcpy(ret.data, segment.data_u8 + data_offset, req.size);
			ret.dst = req.dst;
			ret.paddr = req.paddr;
			bank.ret = ret;
			//return_network.write(ret, ret.port);
		//}
	}
}

void UnitSceneBuffer::process_returns(uint channel_index)
{
	// process data return from DRAM
	// Once the whole treelet has been loaded, we send a signal to Stream Scheduler
	// Then the stream scheduler can start loading rays in that segment
	uint port_in_dram = main_mem_port_stride * channel_index + main_mem_port_offset;
	if (main_memory->return_port_read_valid(port_in_dram))
	{
		// write to the buffer directly
		// TO DO: maybe need to implement latency here (e.g. add an internal return crossbar)

		MemoryReturn ret = main_memory->peek_return(port_in_dram);
		assert(ret.size == CACHE_BLOCK_SIZE);
		uint segment_index = calculate_segment(ret.paddr);
		if (!segment_offset.count(segment_index))
		{
			main_memory->read_return(port_in_dram);
			return;
		}
		ret.port = get_bank(ret.paddr);
		uint data_offset = offset_in_treelet(ret.paddr);
		
		assert(segment_offset.count(segment_index));
		assert(data_offset + ret.size - 1 < treelet_size);
		SegmentState& segment = segments[segment_offset[segment_index]];
		assert(segment.state == SegmentState::State::PREFETCHING);

		assert(get_bank(ret.paddr + ret.size - 1) == get_bank(ret.paddr));
		uint bank_index = get_bank(ret.paddr);
		if (internal_return_crossbar.is_write_valid(channel_index))
		{
			internal_return_crossbar.write(ret, channel_index);
			main_memory->read_return(port_in_dram);
		}
	}

}

void UnitSceneBuffer::issue_requests(uint channel_index)
{
	uint port_in_dram = main_mem_port_stride * channel_index + main_mem_port_offset;
	Channel& channel = channels[channel_index];
	if (!channel.prefetch_queue.empty() && main_memory->request_port_write_valid(port_in_dram))
	{
		paddr_t start_address = channel.prefetch_queue.front();
		uint segment_index = calculate_segment(start_address);
		if (!segment_offset.count(segment_index))
		{
			channel.byte_requested = 0;
			channel.prefetch_queue.pop();
			return;
		}
		MemoryRequest req;
		req.size = CACHE_BLOCK_SIZE;
		req.paddr = start_address + channel.byte_requested;
		req.type = MemoryRequest::Type::LOAD;
		req.port = port_in_dram;
		main_memory->write_request(req);
		channel.byte_requested += req.size;
		if (channel.byte_requested == dram_row_size)
		{
			channel.byte_requested = 0;
			channel.prefetch_queue.pop();
		}
		
		
		SegmentState& segment = segments[segment_offset[segment_index]];
		assert(segment.state == SegmentState::State::PREFETCHING);
		segment.byte_requested += req.size;

		assert(segment.byte_requested <= treelet_size);
	}
}

void UnitSceneBuffer::issue_returns(uint bank_index)
{
	Bank& bank = banks[bank_index];
	if (bank.has_return && return_network.is_write_valid(bank.ret.port))
	{
		bank.has_return = false;
		return_network.write(bank.ret, bank.ret.port);
	}
	if (bank.segment_prefetched != ~0u)
	{
		segment_prefetched.push(bank.segment_prefetched);
		bank.segment_prefetched = ~0u;
	}
	// Technically, we need to send signal to Stream Scheduler if data is loaded
	// This process will be done in stream scheduler
	
}

void UnitSceneBuffer::clock_rise()
{
	request_network.clock();
	process_prefetch();
	process_finish();
	
	for (int i = 0; i < banks.size(); i++)
	{
		process_internal_return(i);
		process_requests(i);
	}
	for (int i = 0; i < NUM_DRAM_CHANNELS; i++)
	{
		process_returns(i);
	}
}

void UnitSceneBuffer::clock_fall()
{
	for (int i = 0; i < NUM_DRAM_CHANNELS; i++)
	{
		issue_requests(i);
	}
	for (int i = 0; i < banks.size(); i++)
	{
		issue_returns(i);
	}
	return_network.clock();
	internal_return_crossbar.clock();
}

}
}
}