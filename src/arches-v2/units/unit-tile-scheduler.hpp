#pragma once 
#include "stdafx.hpp"

#include "unit-base.hpp"
#include "unit-memory-base.hpp"
#include "unit-atomic-reg-file.hpp"
#include "util/bit-manipulation.hpp"

namespace Arches {
namespace Units {

class UnitThreadScheduler : public UnitMemoryBase
{
private:
	UnitAtomicRegfile* _atomic_regs;

	Cascade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;

	bool _current_request_valid{ false };
	MemoryRequest _current_request;

	bool _stalled_for_atomic_reg{ false };

	uint _num_tp;
	uint _tm_index;

	uint _block_size;
	uint _current_block;
	uint _current_offset;

public:
	UnitThreadScheduler(uint num_tp, uint tm_index, UnitAtomicRegfile* atomic_regs, uint block_size) : UnitMemoryBase(),
		_block_size(block_size), _request_network(num_tp, 1), _return_network(num_tp), _num_tp(num_tp), _tm_index(tm_index), _atomic_regs(atomic_regs)
	{
		_current_offset = _block_size;
	}

	void clock_rise() override
	{
		_request_network.clock();

		if (_stalled_for_atomic_reg)
		{
			if (_atomic_regs->return_port_read_valid(_tm_index))
			{
				const MemoryReturn ret = _atomic_regs->read_return(_tm_index);
				_current_block = ret.data_u32;
				_current_offset = 0;

				_stalled_for_atomic_reg = false;
			}
		}
		else if (_request_network.is_read_valid(0) && !_current_request_valid)
		{
			_current_request = _request_network.read(0);
			_current_request_valid = true;
		}
	}

	void clock_fall() override
	{
		if (!_stalled_for_atomic_reg && _current_request_valid)
		{
			if (_current_offset == _block_size)
			{
				if (_atomic_regs->request_port_write_valid(_tm_index))
				{
					MemoryRequest request;
					request.type = MemoryRequest::Type::AMO_ADD;
					request.size = 4;
					request.port = _tm_index;
					request.paddr = 0x0ull;
					request.data_u32 = _block_size;
					_atomic_regs->write_request(request);
					_stalled_for_atomic_reg = true;
				}
			}
			else if(_return_network.is_write_valid(_current_request.port))
			{
				//uint tile_x = pext(_current_offset, 0x5555);
				//uint tile_y = pext(_current_offset, 0xaaaa);
				//uint tile_x = (_current_offset % _tile_width);
				//uint tile_y = (_current_offset / _tile_width);
				//uint x = (_current_tile % (_width / _tile_width)) * _tile_width + tile_x;
				//uint y = (_current_tile / (_width / _tile_width)) * _tile_height + tile_y;
				uint32_t index = _current_block + _current_offset;
				MemoryReturn ret(_current_request, &index);

				if(index == 0)
				{
					//__debugbreak();
				}

				_return_network.write(ret, ret.port);

				_current_offset++;
				_current_request_valid = false;
			}
		}

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
};

}
}