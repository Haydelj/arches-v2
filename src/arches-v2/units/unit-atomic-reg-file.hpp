#pragma once 
#include "stdafx.hpp"

#include "unit-base.hpp"
#include "unit-memory-base.hpp"

#include "util/arbitration.hpp"

namespace Arches {
namespace Units {

class UnitAtomicRegfile : public UnitMemoryBase
{
public:
	uint32_t iregs[32];

private:
	bool _current_request_valid{ false };
	MemoryRequest _current_request;

	Cascade<MemoryRequest, RoundRobinArbiter<uint128_t>> _request_network;
	ReturnCascade _return_network;

public:
	UnitAtomicRegfile(uint num_clients) : UnitMemoryBase(),
		_request_network(num_clients, 1), _return_network(1, num_clients)
	{
		for (uint i = 0; i < 32; ++i)
			iregs[i] = 0;
	}

	void clock_rise() override
	{
		_request_network.clock();

		if (_request_network.is_read_valid(0))
		{
			_current_request = _request_network.read(0);
			_current_request_valid = true;
		}
	}

	void clock_fall() override
	{
		if (_current_request_valid)
		{
			if (_current_request.type != MemoryRequest::Type::STORE && !_return_network.is_write_valid(0)) return;

			uint32_t reg_index = (_current_request.paddr >> 2) & 0b1'1111;
			uint32_t request_data = _current_request.data_u32;
			uint32_t ret_val = iregs[reg_index];

			switch (_current_request.type)
			{
			case MemoryRequest::Type::STORE:
				iregs[reg_index] = request_data;
				break;

			case MemoryRequest::Type::LOAD:
				break;

			case MemoryRequest::Type::AMO_ADD:
				iregs[reg_index] += request_data;
				break;

			case MemoryRequest::Type::AMO_AND:
				iregs[reg_index] &= request_data;
				break;

			case MemoryRequest::Type::AMO_OR:
				iregs[reg_index] |= request_data;
				break;

			case MemoryRequest::Type::AMO_XOR:
				iregs[reg_index] ^= request_data;
				break;

			case MemoryRequest::Type::AMO_MIN:
				iregs[reg_index] = std::min((int32_t)request_data, (int32_t)iregs[reg_index]);
				break;

			case MemoryRequest::Type::AMO_MAX:
				iregs[reg_index] = std::max((int32_t)request_data, (int32_t)iregs[reg_index]);
				break;

			case MemoryRequest::Type::AMO_MINU:
				iregs[reg_index] = std::min(request_data, iregs[reg_index]);
				break;

			case MemoryRequest::Type::AMO_MAXU:
				iregs[reg_index] = std::max(request_data, iregs[reg_index]);
				break;
			}

			if (_current_request.type != MemoryRequest::Type::STORE)
			{
				MemoryReturn ret(_current_request, &ret_val);
				_return_network.write(ret, 0);
			}

			_current_request_valid = false;
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