#include "unit-scene-buffer.hpp"

namespace Arches { namespace Units { namespace DualStreaming {

void UnitSceneBuffer::process_finish()
{
	if(retire_sideband.is_read_valid())
	{
		uint segment_id = retire_sideband.read();

		if (_dynamic_prefetch && _address_translator.is_mapped(segment_id))
		{
			// we need to reset the block status
			paddr_t buffer_start = _address_translator.translate(_address_translator.get_segment_address(segment_id));
			uint block_id = buffer_start / CACHE_BLOCK_SIZE;
			uint num_blocks = _address_translator.segment_size / CACHE_BLOCK_SIZE;
			for (uint i = 0; i < num_blocks; ++i)
				_block_status[block_id + i] = 0; // In reality, this would be super fast
		}
		_address_translator.unmap(segment_id);
		_segment_states.erase(segment_id);
	}
}

void UnitSceneBuffer::process_prefetch()
{
	if (prefetch_sideband.is_read_valid() && _address_translator.num_free_slots())
	{
		auto [segment_id, ratio] = prefetch_sideband.read();
		uint slot = _address_translator.map(segment_id);
		paddr_t segment_addr = _address_translator.get_segment_address(segment_id);
		_segment_states[segment_id] = SegmentState();

		if (_dynamic_prefetch)
		{
			uint buffer_addr = _address_translator.translate(segment_addr);
			uint block_id = buffer_addr / CACHE_BLOCK_SIZE;
			// Start prefetching from 0
			assert(!_block_status[block_id]);

			// Based on the ray ratio
			uint bytes_to_prefetch = ratio * _address_translator.segment_size;
			uint num_blocks = bytes_to_prefetch / CACHE_BLOCK_SIZE;
			uint block_per_channel = ROW_BUFFER_SIZE / CACHE_BLOCK_SIZE;
			paddr_t start_addr = segment_addr;
			while (num_blocks > 0)
			{
				uint block_to_fetch = std::min(num_blocks, block_per_channel);
				uint channel_index = (start_addr / ROW_BUFFER_SIZE) % NUM_DRAM_CHANNELS;
				_channels[channel_index].dynamic_prefetch_queue.push(std::make_pair(start_addr, block_to_fetch));
				num_blocks -= block_to_fetch;
				start_addr += block_to_fetch * CACHE_BLOCK_SIZE;
			}
			_prefetch_complete_queue.push(segment_id); // SS can start issuing rays
		}
		else 
		{
			for (uint i = 0; i < _address_translator.segment_size; i += ROW_BUFFER_SIZE)
			{
				uint channel_index = ((segment_addr + i) / ROW_BUFFER_SIZE) % NUM_DRAM_CHANNELS;
				_channels[channel_index].prefetch_queue.emplace(segment_addr + i);
			}
		}
	}
}

void UnitSceneBuffer::process_requests(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	if (_dynamic_prefetch)
	{
		if (_request_network.is_read_valid(bank_index))
		{
			const MemoryRequest req = _request_network.peek(bank_index);
			assert(req.size == CACHE_BLOCK_SIZE);
			paddr_t buffer_addr = _address_translator.translate(req.paddr);
			uint block_id = buffer_addr / CACHE_BLOCK_SIZE;
			if (_block_status[block_id] == 2)
			{
				_request_network.read(bank_index);
				bank.data_array_pipline.write(MemoryReturn(req, &_data_u8[buffer_addr]));
				log.loads++;
			}
			else if(_block_status[block_id] == 1)
			{
				// half misses
			}
			else
			{
				// misses
				log.read_misses++;
				// prefetch again
				uint channel_index = (req.paddr / ROW_BUFFER_SIZE) % NUM_DRAM_CHANNELS;
				uint blocks_to_next_channel = (ROW_BUFFER_SIZE - req.paddr % ROW_BUFFER_SIZE) / CACHE_BLOCK_SIZE;
				_channels[channel_index].dynamic_prefetch_queue.push(std::make_pair(req.paddr, std::min(_prefetch_block, blocks_to_next_channel)));
				_block_status[block_id] = 1;
			}
		}
	}
	else if (bank.data_array_pipline.is_write_valid() && _request_network.is_read_valid(bank_index))
	{
		const MemoryRequest req = _request_network.read(bank_index);
		paddr_t buffer_addr = _address_translator.translate(req.paddr);
		bank.data_array_pipline.write(MemoryReturn(req, &_data_u8[buffer_addr]));
		log.loads++;
	}
	bank.data_array_pipline.clock();
}

void UnitSceneBuffer::process_returns(uint channel_index)
{
	uint port_in_dram = _main_mem_port_stride * channel_index + _main_mem_port_offset;
	if(_main_memory->return_port_read_valid(port_in_dram))
	{
		const MemoryReturn ret = _main_memory->read_return(port_in_dram);
		uint segment_id = _address_translator.get_segment_id(ret.paddr);
		if(_address_translator.is_mapped(segment_id)) //the segment can be unmapped before it finishes prefetching in wich case we can disgard the returned data
		{
			paddr_t buffer_addr = _address_translator.translate(ret.paddr);
			std::memcpy(&_data_u8[buffer_addr], ret.data, ret.size);

			_segment_states[segment_id].bytes_returned += ret.size;
			
			if(!_dynamic_prefetch && _segment_states[segment_id].bytes_returned == _address_translator.segment_size)
				_prefetch_complete_queue.push(segment_id);

			log.stores++;
			log.bytes_written += ret.size;

			assert(ret.size == CACHE_BLOCK_SIZE);
			uint block_id = buffer_addr / CACHE_BLOCK_SIZE;
			_block_status[block_id] = 2;
		}
	}
}

void UnitSceneBuffer::issue_requests(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint port_in_dram = _main_mem_port_stride * channel_index + _main_mem_port_offset;
	if (_dynamic_prefetch)
	{
		if (!channel.dynamic_prefetch_queue.empty() && _main_memory->request_port_write_valid(port_in_dram))
		{
			paddr_t segment_address = channel.dynamic_prefetch_queue.front().first;
			uint num_blocks = channel.dynamic_prefetch_queue.front().second;
			MemoryRequest req;
			req.size = CACHE_BLOCK_SIZE;
			req.paddr = segment_address + channel.byte_requested;
			req.type = MemoryRequest::Type::LOAD;
			req.port = port_in_dram;
			_main_memory->write_request(req);
			
			assert((req.paddr / ROW_BUFFER_SIZE) % NUM_DRAM_CHANNELS == channel_index);
			channel.byte_requested += req.size;
			if (channel.byte_requested == num_blocks * CACHE_BLOCK_SIZE)
			{
				channel.dynamic_prefetch_queue.pop();
				channel.byte_requested = 0;
			}

		}	
	}
	else if (!channel.prefetch_queue.empty() && _main_memory->request_port_write_valid(port_in_dram))
	{
		paddr_t base_address = channel.prefetch_queue.front();

		MemoryRequest req;
		req.size = CACHE_BLOCK_SIZE;
		req.paddr = base_address + channel.byte_requested;
		req.type = MemoryRequest::Type::LOAD;
		req.port = port_in_dram;
		_main_memory->write_request(req);
		channel.byte_requested += req.size;

		if (channel.byte_requested == ROW_BUFFER_SIZE)
		{
			channel.byte_requested = 0;
			channel.prefetch_queue.pop();
		}
	}
}

void UnitSceneBuffer::issue_returns(uint bank_index)
{
	Bank& bank = _banks[bank_index];
	if(bank.data_array_pipline.is_read_valid() && _return_network.is_write_valid(bank_index))
	{
		log.bytes_read += bank.data_array_pipline.peek().size;
		_return_network.write(bank.data_array_pipline.read(), bank_index);
	}
}

void UnitSceneBuffer::clock_rise()
{
	_request_network.clock();

	process_prefetch();
	process_finish();
	
	for (uint i = 0; i < _banks.size(); ++i)
		process_requests(i);
	
	for(uint i = 0; i < _channels.size(); ++i)
		process_returns(i);
}

void UnitSceneBuffer::clock_fall()
{
	for (uint i = 0; i < _channels.size(); ++i)
		issue_requests(i);

	for (uint i = 0; i < _banks.size(); ++i)
		issue_returns(i);

	if(!_prefetch_complete_queue.empty() && prefetch_complete_sideband.is_write_valid())
	{
		prefetch_complete_sideband.write(_prefetch_complete_queue.front());
		_prefetch_complete_queue.pop();
	}

	_return_network.clock();
}

}}}