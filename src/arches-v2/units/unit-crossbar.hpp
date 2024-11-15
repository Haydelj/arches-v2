#pragma once 
#include "stdafx.hpp"

#include "util/bit-manipulation.hpp"
#include "unit-base.hpp"
#include "unit-main-memory-base.hpp"

namespace Arches { namespace Units {

class UnitCrossbar : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint num_clients{1};
		uint width{1};
		uint latency{1};
		uint64_t mem_higher_select_mask{0};
		std::vector<UnitMemoryBase*> mem_highers;
	};

private:
	uint64_t _select_mask;
	RequestCrossBar _request_network;
	ReturnCrossBar _return_network;
	std::vector<MemoryRequest> _request_regs;
	std::vector<MemoryReturn> _return_regs;
	std::vector<UnitMemoryBase*> _mem_highers;

public:
	UnitCrossbar(Configuration config) : UnitMemoryBase(),
		_select_mask(config.mem_higher_select_mask),
		_request_network(config.num_clients, config.mem_highers.size(), config.mem_higher_select_mask, config.width),
		_return_network(config.mem_highers.size(), config.num_clients, config.width),
		_request_regs(config.mem_highers.size()),
		_return_regs(config.mem_highers.size()),
		_mem_highers(config.mem_highers)
	{
		for(uint i = 0; i < _request_regs.size(); ++i)
			_request_regs[i].paddr = ~0x0ull;
		for(uint i = 0; i < _return_regs.size(); ++i)
			_return_regs[i].paddr = ~0x0ull;
	}

	void clock_rise() override
	{
		_request_network.clock();

		//select next request and issue to pipline
		for(uint i = 0; i < _request_network.num_sinks(); ++i)
		{
			if(!_request_network.is_read_valid(i) || _request_regs[i].paddr != ~0x0ull) continue;
			_request_regs[i] = _request_network.read(i);
		}

		for(uint i = 0; i < _mem_highers.size(); ++i)
		{
			if(!_mem_highers[i]->return_port_read_valid(0) || _return_regs[i].paddr != ~0x0ull) continue;
			_return_regs[i] = _mem_highers[i]->read_return(0);
		}
	}

	void clock_fall() override
	{
		for(uint i = 0; i < _request_regs.size(); ++i)
		{
			if(_request_regs[i].paddr == ~0x0ull || !_mem_highers[i]->request_port_write_valid(0)) continue;

			//_request_regs[i].paddr = _remove_select_bits(_request_regs[i].paddr);
			//_request_regs[i].dst = _request_regs[i].port;
			//_request_regs[i].port = 0;

			_mem_highers[i]->write_request(_request_regs[i]);
			_request_regs[i].paddr = ~0x0ull;
		}

		for(uint i = 0; i < _return_regs.size(); ++i)
		{
			if(_return_regs[i].paddr == ~0x0ull || !_return_network.is_write_valid(i)) continue;

			//_return_regs[i].paddr = _insert_select_bits(_return_regs[i].paddr, i);
			//_return_regs[i].port = _return_regs[i].dst;
			//_return_regs[i].dst = 0;

			_return_network.write(_return_regs[i], i);
			_return_regs[i].paddr = ~0x0ull;
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

private:
	paddr_t _remove_select_bits(paddr_t addr)
	{
		return pext(addr, ~_select_mask);
	}

	paddr_t _insert_select_bits(paddr_t addr, paddr_t index)
	{
		addr = pdep(addr, ~_select_mask);
		addr |= pdep(index, _select_mask);
		return addr;
	}
};

}}