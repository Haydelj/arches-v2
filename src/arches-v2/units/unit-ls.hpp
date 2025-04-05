#pragma once 
#include "stdafx.hpp"

#include "util/bit-manipulation.hpp"
#include "unit-base.hpp"
#include "unit-main-memory-base.hpp"

namespace Arches { namespace Units {

class UnitLoadStore : public UnitMemoryBase, public Cascade<MemoryRequest>, Cascade<MemoryReturn>
{
public:
	struct Configuration
	{
		uint num_tps{1};
		uint num_ports{1};
		UnitMemoryBase* cache;
	};

private:
	uint _num_partitions{1};
	uint _partition_stride{1};
	uint _num_slices{1};
	uint _slice_stride{1};

	std::vector<MemoryRequest> _request_regs;
	std::vector<MemoryReturn> _return_regs;
	UnitMemoryBase* _cache;

public:
	UnitLoadStore(Configuration config) : UnitMemoryBase(), 
		Cascade<MemoryRequest>(config.num_tps, config.num_ports, 64, 64),
		Cascade<MemoryReturn>(config.num_tps, config.num_ports, 64, 64),
		_cache(config.cache)
	{
		for(uint i = 0; i < _request_regs.size(); ++i)
			_request_regs[i].paddr = ~0x0ull;

		for(uint i = 0; i < _return_regs.size(); ++i)
			_return_regs[i].paddr = ~0x0ull;
	}

	void clock_rise() override
	{
		for(uint i = 0; i < Cascade<MemoryRequest>::num_sinks(); ++i)
		{
			for(uint i = 0; i < 2; ++i) Cascade<MemoryRequest>::clock();

			//select next request and issue to piplinev
			for(uint i = 0; i < _request_regs.size(); ++i)
			{
				if(!Cascade<MemoryRequest>::is_read_valid(i) || _request_regs[i].paddr != ~0x0ull) continue;
				_request_regs[i] = Cascade<MemoryRequest>::read(i);
			}

			for(uint i = 0; i < _return_regs.size(); ++i)
			{
				uint partition = i / _num_slices;
				uint slice = i % _num_slices;
				if(!_cache->return_port_read_valid(i) || _return_regs[i].paddr != ~0x0ull) continue;
				_return_regs[i] = _cache->read_return(i);
			}

		}
	}

	void clock_fall() override
	{
		for(uint i = 0; i < _request_regs.size(); ++i)
		{
			if(_request_regs[i].paddr == ~0x0ull || !_cache->request_port_write_valid(i)) continue;

			_request_regs[i].dst.push(_request_regs[i].port, 7);
			_request_regs[i].port = i;

			_cache->write_request(_request_regs[i]);
			_request_regs[i].paddr = ~0x0ull;
		}

		for(uint i = 0; i < _return_regs.size(); ++i)
		{
			if(_return_regs[i].paddr == ~0x0ull || !Cascade<MemoryReturn>::is_write_valid(i)) continue;

			_return_regs[i].port = _return_regs[i].dst.pop(7);

			Cascade<MemoryReturn>::write(_return_regs[i], i);
			_return_regs[i].paddr = ~0x0ull;
		}

		for(uint i = 0; i < 2; ++i) Cascade<MemoryReturn>::clock();
	}

	bool request_port_write_valid(uint port_index) override
	{
		return Cascade<MemoryRequest>::is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request) override
	{
		Cascade<MemoryRequest>::write(request, request.port);
	}

	bool return_port_read_valid(uint port_index) override
	{
		return Cascade<MemoryReturn>::is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index) override
	{
		return Cascade<MemoryReturn>::peek(port_index);
	}

	const MemoryReturn read_return(uint port_index) override
	{
		return Cascade<MemoryReturn>::read(port_index);
	}
};

}}