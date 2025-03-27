#pragma once 
#include "stdafx.hpp"

#include "util/bit-manipulation.hpp"
#include "unit-base.hpp"
#include "unit-main-memory-base.hpp"

namespace Arches { namespace Units {

class UnitCrossbar : public UnitMemoryBase, public CasscadedCrossBar<MemoryRequest>
{
public:
	struct Configuration
	{
		uint num_clients{1};
		uint width{1};
		uint latency{1};
		uint num_partitions{1};
		uint partition_stride{1};
		uint num_slices{1};
		uint slice_stride{1};
		std::vector<UnitMemoryBase*> mem_highers;
	};

private:
	uint _num_partitions{1};
	uint _partition_stride{1};
	uint _num_slices{1};
	uint _slice_stride{1};

	ReturnCrossBar _return_network;
	std::vector<MemoryRequest> _request_regs;
	std::vector<MemoryReturn> _return_regs;
	std::vector<UnitMemoryBase*> _mem_highers;

public:
	UnitCrossbar(Configuration config) : UnitMemoryBase(), CasscadedCrossBar<MemoryRequest>(config.num_clients, config.num_partitions * config.num_slices, config.width),
		_return_network(config.num_partitions * config.num_slices, config.num_clients, config.width),
		_num_partitions(config.num_partitions), _partition_stride(config.partition_stride), 
		_num_slices(config.num_slices), _slice_stride(config.slice_stride),
		_request_regs(_num_partitions * _num_slices),
		_return_regs(_num_partitions * _num_slices),
		_mem_highers(config.mem_highers)
	{
		_assert(_mem_highers.size() == _num_partitions);

		for(uint i = 0; i < _request_regs.size(); ++i)
			_request_regs[i].paddr = ~0x0ull;
		for(uint i = 0; i < _return_regs.size(); ++i)
			_return_regs[i].paddr = ~0x0ull;
	}

	uint get_sink(const MemoryRequest& request) override
	{
		paddr_t paddr = request.paddr;
		uint partition = paddr / _partition_stride % _num_partitions;
		paddr = strip_partition(paddr);
		uint slice = paddr / _slice_stride % _num_slices;
		return partition * _num_slices + slice;
	}

	paddr_t strip_partition(paddr_t paddr)
	{
		return (paddr / _partition_stride / _num_partitions) * _partition_stride + (paddr % _partition_stride);
	}

	paddr_t unstrip_partition(paddr_t paddr, uint partition)
	{
		return (paddr / _partition_stride * _num_partitions + partition) * _partition_stride + (paddr % _partition_stride);
	}

	void clock_rise() override
	{
		for(uint i = 0; i < 1; ++i) clock();

		//select next request and issue to pipline
		for(uint i = 0; i < num_sinks(); ++i)
		{
			if(!is_read_valid(i) || _request_regs[i].paddr != ~0x0ull) continue;
			_request_regs[i] = read(i);
		}

		for(uint i = 0; i < num_sinks(); ++i)
		{
			uint partition = i / _num_slices;
			uint slice = i % _num_slices;
			if(!_mem_highers[partition]->return_port_read_valid(slice) || _return_regs[i].paddr != ~0x0ull) continue;
			_return_regs[i] = _mem_highers[partition]->read_return(slice);
		}
	}

	void clock_fall() override
	{
		for(uint i = 0; i < _request_regs.size(); ++i)
		{
			uint partition = i / _num_slices;
			uint slice = i % _num_slices;
			if(_request_regs[i].paddr == ~0x0ull || !_mem_highers[partition]->request_port_write_valid(slice)) continue;

			_request_regs[i].paddr = strip_partition(_request_regs[i].paddr);
			_request_regs[i].dst.push(_request_regs[i].port, 7);
			_request_regs[i].port = slice;

			_mem_highers[partition]->write_request(_request_regs[i]);
			_request_regs[i].paddr = ~0x0ull;
		}

		for(uint i = 0; i < _return_regs.size(); ++i)
		{
			uint partition = i / _num_slices;
			if(_return_regs[i].paddr == ~0x0ull || !_return_network.is_write_valid(i)) continue;

			_return_regs[i].paddr = unstrip_partition(_return_regs[i].paddr, partition);
			_return_regs[i].port = _return_regs[i].dst.pop(7);

			_return_network.write(_return_regs[i], i);
			_return_regs[i].paddr = ~0x0ull;
		}

		for(uint i = 0; i < 1; ++i) _return_network.clock();
	}

	bool request_port_write_valid(uint port_index) override
	{
		return is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request) override
	{
		write(request, request.port);
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

}}