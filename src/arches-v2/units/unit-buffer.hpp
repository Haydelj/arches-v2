#pragma once 
#include "stdafx.hpp"

#include "util/bit-manipulation.hpp"
#include "unit-base.hpp"
#include "unit-main-memory-base.hpp"

namespace Arches { namespace Units {

class UnitBuffer : public UnitMainMemoryBase
{
public:
	struct Configuration
	{
		uint64_t size{1024};
		uint num_ports{1};
		uint num_banks{1};
		uint latency{1};
	};

private:
	struct Bank
	{
		LatencyFIFO<MemoryRequest> data_pipline;
		Bank(uint latency) : data_pipline(latency) {}
	};

	bool exec = false;

	uint64_t _buffer_address_mask;

	std::vector<Bank> _banks;
	RequestCascade _request_network;
	ReturnCascade _return_network;

public:
	UnitBuffer(Configuration config) : UnitMainMemoryBase(config.size),
		_request_network(config.num_ports, config.num_banks), 
		_return_network(config.num_banks, config.num_ports),
		_banks(config.num_banks, config.latency)
	{
		_buffer_address_mask = generate_nbit_mask(log2i(config.size));
	}

	void clock_rise() override
	{
		_request_network.clock();

		//select next request and issue to pipline
		for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
		{
			Bank& bank = _banks[bank_index];

			if(!_request_network.is_read_valid(bank_index) || !bank.data_pipline.is_write_valid()) continue;

			bank.data_pipline.write(_request_network.read(bank_index));
		}
	}

	void clock_fall() override
	{
		for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
		{
			Bank& bank = _banks[bank_index];
			bank.data_pipline.clock();

			if(!bank.data_pipline.is_read_valid()) continue;

			const MemoryRequest& req = bank.data_pipline.peek();
			paddr_t buffer_addr = _get_buffer_addr(req.paddr);
			if(req.type == MemoryRequest::Type::LOAD)
			{
				if(!_return_network.is_write_valid(bank_index)) continue;

				_assert(req.paddr < 0x1ull << 32);

				log.loads++;
				log.bytes_read += req.size;

				MemoryReturn ret(req, &_data_u8[buffer_addr]);
				_return_network.write(ret, bank_index);
				bank.data_pipline.read();
			}
			else if(req.type == MemoryRequest::Type::STORE)
			{
				log.stores++;
				log.bytes_written += req.size;

				//Masked write
				//for(uint i = 0; i < req.size; ++i)
				//	if((req.write_mask >> i) & 0x1)
				//		_data_u8[buffer_addr + i] = req.data[i];

				std::memcpy(&_data_u8[buffer_addr], req.data, req.size);

				bank.data_pipline.read();
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

private:
	paddr_t _get_buffer_addr(paddr_t paddr) { return paddr & _buffer_address_mask; }

public:
	class Log
	{
	public:
		union
		{
			struct
			{
				uint64_t loads;
				uint64_t stores;
				uint64_t bytes_read;
				uint64_t bytes_written;
			};
			uint64_t counters[8];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < 8; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < 8; ++i)
				counters[i] += other.counters[i];
		}

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = loads + stores;
			float ft = total / 100.0f;

			printf("Read Bandwidth: %.2f bytes/cycle\n", (double)bytes_read / units / cycles);
			printf("Write Bandwidth: %.2f bytes/cycle\n", (double)bytes_written / units / cycles);
			printf("\n");

			printf("Total: %lld\n", total / units);
			printf("Loads: %lld\n", loads / units);
			printf("Stores: %lld\n", stores / units);
			printf("\n");
		}
	}
	log;
};

}}