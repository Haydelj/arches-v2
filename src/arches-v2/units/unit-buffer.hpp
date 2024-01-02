#pragma once 
#include "stdafx.hpp"

#include "util/bit-manipulation.hpp"
#include "unit-base.hpp"
#include "unit-memory-base.hpp"

namespace Arches { namespace Units {

class UnitBuffer : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint64_t size{1024};
		uint num_ports{1};
		uint num_banks{1};
		uint64_t bank_select_mask{0};
		uint latency{1};
	};

private:
	struct Bank
	{
		Pipline<MemoryRequest> data_pipline;
		Bank(uint latency) : data_pipline(latency, 1) {}
	};

	uint8_t* _data_u8;
	uint64_t _buffer_address_mask;

	std::vector<Bank> _banks;
	RequestCrossBar _request_network;
	ReturnCrossBar _return_network;

public:
	UnitBuffer(Configuration config) : UnitMemoryBase(),
		_request_network(config.num_ports, config.num_banks, config.bank_select_mask), _return_network(config.num_ports, config.num_banks), _banks(config.num_banks, config.latency)
	{
		_data_u8 = (uint8_t*)malloc(config.size);
		_buffer_address_mask = generate_nbit_mask(log2i(config.size));
	}

	~UnitBuffer()
	{
		free(_data_u8);
	}

	void clock_rise() override
	{
		_request_network.clock();

		//select next request and issue to pipline
		for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
		{
			Bank& bank = _banks[bank_index];
			bank.data_pipline.clock();
			if(!bank.data_pipline.is_write_valid() || !_request_network.is_read_valid(bank_index)) continue;
			bank.data_pipline.write(_request_network.read(bank_index));
		}
	}

	void clock_fall() override
	{
		for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
		{
			Bank& bank = _banks[bank_index];
			if(!bank.data_pipline.is_read_valid()) continue;

			const MemoryRequest& req = bank.data_pipline.peek();
			paddr_t buffer_addr = _get_buffer_addr(req.paddr);
			if(req.type == MemoryRequest::Type::LOAD)
			{
				if(!_return_network.is_write_valid(bank_index)) continue;

				MemoryReturn ret(req, &_data_u8[buffer_addr]);
				_return_network.write(ret, bank_index);
				bank.data_pipline.read();
				log.log_read();
			}
			else if(req.type == MemoryRequest::Type::STORE)
			{
				std::memcpy(&_data_u8[buffer_addr], req.data, req.size);
				bank.data_pipline.read();
				log.log_write();
			}
			else assert(false);
		}

		_return_network.clock();
	}

	bool request_port_write_valid(uint port_index) override
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request, uint port_index) override
	{
		assert(request.port == port_index);
		_request_network.write(request, port_index);
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
	paddr_t _get_buffer_addr(paddr_t paddr) { return paddr & _buffer_address_mask; }

public:
	class Log
	{
	public:
		uint64_t _reads;
		uint64_t _writes;

		Log() { reset(); }

		void reset()
		{
			_reads = 0;
			_writes = 0;
		}

		void accumulate(const Log& other)
		{
			_reads += other._reads;
			_writes += other._writes;
		}

		void log_read(uint n = 1) { _reads += n; } //TODO hit under miss logging
		void log_write(uint n = 1) { _writes += n; }

		uint64_t get_total() { return _reads + _writes; }

		void print_log(FILE* stream = stdout, uint units = 1)
		{
			uint64_t total = get_total();
			float ft = total / 100.0f;

			fprintf(stream, "Total: %lld\n", total / units);
			fprintf(stream, "Reads: %lld(%.2f%%)\n", _reads / units, _reads / ft);
			fprintf(stream, "Writes: %lld(%.2f%%)\n", _writes / units, _writes / ft);
		}
	}log;
};

}}