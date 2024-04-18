#include "unit-scene-buffer.hpp"

namespace Arches { namespace Units { namespace DualStreaming {

void UnitSceneBuffer::process_commands()
{
	if(command_sideband.is_read_valid())
	{
		const Command& command = command_sideband.peek();
		if(command.type == Command::Type::PREFETCH)
		{
			_assert(_address_translator.num_free_slots() > 0);

			uint slot = _address_translator.map(command.segment_id);
			paddr_t segment_addr = _address_translator.get_segment_address(command.segment_id);
			_segment_states[command.segment_id] = SegmentState();

			for(uint i = 0; i < _address_translator.segment_size; i += _row_size)
			{
				uint channel_index = ((segment_addr + i) / _row_size) % _channels.size();
				_channels[channel_index].prefetch_queue.emplace(segment_addr + i);
			}

			command_sideband.read();
		}
		else if(command.type == Command::Type::RETIRE)
		{
			_address_translator.unmap(command.segment_id);
			_segment_states.erase(command.segment_id);
			command_sideband.read();
		}
	}
}

void UnitSceneBuffer::process_requests(uint bank_index)
{
	Bank& bank = _banks[bank_index];
	if (bank.data_array_pipline.is_write_valid() && _request_network.is_read_valid(bank_index))
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
			if(_segment_states[segment_id].bytes_returned == _address_translator.segment_size)
				_prefetch_complete_queue.push(segment_id);

			log.stores++;
			log.bytes_written += ret.size;
		}
	}
}

void UnitSceneBuffer::issue_requests(uint channel_index)
{
	Channel& channel = _channels[channel_index];
	uint port_in_dram = _main_mem_port_stride * channel_index + _main_mem_port_offset;
	if (!channel.prefetch_queue.empty() && _main_memory->request_port_write_valid(port_in_dram))
	{
		paddr_t base_address = channel.prefetch_queue.front();

		MemoryRequest req;
		req.size = _block_size;
		req.paddr = base_address + channel.byte_requested;
		req.type = MemoryRequest::Type::LOAD;
		req.port = port_in_dram;
		_main_memory->write_request(req);
		channel.byte_requested += req.size;

		if (channel.byte_requested == _row_size)
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

	process_commands();
	
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