#include "unit-scene-buffer.hpp"

namespace Arches {
namespace Units {
namespace DualStreaming {


void UnitSceneBuffer::process_finish()
{
	if (!segment_finished.empty())
	{
		uint segment_index = segment_finished.front();
		segment_finished.pop();

		assert(segment_offset.count(segment_index));
		SegmentState& segment = segments[segment_offset[segment_index]];
		segment.state = SegmentState::State::EMPTY;
		segment.segment_index = ~0u;
		segment.byte_requested = 0;
		segment.byte_returned = 0;
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

		uint new_segment_slot = find_segment_slot();
		std::cout << "new segment slot for segment " << segment_index << ": " << new_segment_slot << '\n';
		if (new_segment_slot != ~0u)
		{
			prefetch_request.pop();

			segment_offset[segment_index] = new_segment_slot;
			// distribute the prefetch request to multiple bank
			uint start_bank = (new_segment_slot * treelet_size / row_size) % num_bank;
			uint num_banks_to_across = treelet_size / row_size;
			uint treelet_offset = 0;
			for (uint i = 0; i < num_banks_to_across; i++)
			{
				uint bank_index = (start_bank + i) % num_bank;
				
				banks[bank_index].prefetch_queue.push({ segment_index, treelet_offset });
				treelet_offset += row_size;
			}

			SegmentState& segment = segments[new_segment_slot];
			segment.state = SegmentState::State::PREFETCHING;
			segment.segment_index = segment_index;
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
		assert(segment.state == SegmentState::State::LOADED);

		uint data_offset = offset_in_treelet(req.paddr);

		assert(data_offset + req.size - 1 < treelet_size);
		assert(get_bank(req.paddr + req.size - 1) == bank_index); // requested data should be all in this bank
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
	else if (!bank.prefetch_queue.empty())
	{
		auto [segment_index, treelet_offset] = bank.prefetch_queue.front();
		assert(segment_offset.count(segment_index));
		//printf("Bank %d prefetching segment %d, already requested %d, row_size: %d\n", bank_index, segment_index, bank.byte_requested, row_size);
		SegmentState& segment = segments[segment_offset[segment_index]];
		assert(segment.state == SegmentState::State::PREFETCHING);
		if (internal_crossbar.is_write_valid(bank_index))
		{
			MemoryRequest req;
			req.paddr = calculate_treelet_address(segment_index) + treelet_offset + bank.byte_requested;
			req.size = CACHE_BLOCK_SIZE;
			req.type = MemoryRequest::Type::LOAD;
			internal_crossbar.write(req, bank_index);
			//printf("req size: %d\n", req.size);
			bank.byte_requested += req.size;
			segment.byte_requested += req.size;
			assert(segment.byte_requested <= treelet_size);
			if (bank.byte_requested == row_size)
			{
				bank.byte_requested = 0;
				bank.prefetch_queue.pop();
			}
		}
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
		const MemoryReturn& ret = main_memory->read_return(port_in_dram);
		assert(ret.size == CACHE_BLOCK_SIZE);
		uint segment_index = calculate_segment(ret.paddr);
		uint data_offset = offset_in_treelet(ret.paddr);

		assert(segment_offset.count(segment_index));
		assert(data_offset + ret.size - 1 < treelet_size);

		assert(get_bank(ret.paddr + ret.size - 1) == get_bank(ret.paddr));

		SegmentState& segment = segments[segment_offset[segment_index]];
		assert(segment.state == SegmentState::State::PREFETCHING);

		std::memcpy(segment.data_u8 + data_offset, &ret.data, ret.size);
		segment.byte_returned += ret.size;

		assert(segment.byte_returned <= treelet_size);
		if (segment.byte_returned == treelet_size)
		{
			segment.state = SegmentState::State::LOADED;
			segment_prefetched.push(segment_index);
		}
		//printf("Scene Buffer Loading Segment: %d, Data loaded percentage: %.2f\%\n", segment_index, 100.0 * segment.byte_returned / treelet_size);
	}

}

void UnitSceneBuffer::issue_requests(uint channel_index)
{
	uint port_in_dram = main_mem_port_stride * channel_index + main_mem_port_offset;
	if (main_memory->request_port_write_valid(port_in_dram) && internal_crossbar.is_read_valid(channel_index))
	{
		//printf("Scene Buffer issuing requests from channel %d\n", channel_index);
		MemoryRequest req = internal_crossbar.read(channel_index);

		uint segment_index = calculate_segment(req.paddr);
		SegmentState& segment = segments[segment_offset[segment_index]];
		assert(segment.state == SegmentState::State::PREFETCHING);
		req.port = port_in_dram;
		main_memory->write_request(req);
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

	// Technically, we need to send signal to Stream Scheduler if data is loaded
	// This process will be done in stream scheduler
	
}

void UnitSceneBuffer::clock_rise()
{
	request_network.clock();

	process_finish();
	process_prefetch();
	for (int i = 0; i < banks.size(); i++)
	{
		process_requests(i);
	}
	for (int i = 0; i < NUM_DRAM_CHANNELS; i++)
	{
		process_returns(i);
	}
}

void UnitSceneBuffer::clock_fall()
{
	internal_crossbar.clock();
	for (int i = 0; i < NUM_DRAM_CHANNELS; i++)
	{
		issue_requests(i);
	}
	for (int i = 0; i < banks.size(); i++)
	{
		issue_returns(i);
	}
	return_network.clock();
}

}
}
}