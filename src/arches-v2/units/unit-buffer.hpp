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
	RequestCrossBar _request_cross_bar;
	ReturnCrossBar _return_cross_bar;

public:
	UnitBuffer(Configuration config) : UnitMemoryBase(),
		_request_cross_bar(config.num_ports, config.num_banks, config.bank_select_mask), _return_cross_bar(config.num_ports, config.num_banks), _banks(config.num_banks, config.latency)
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
		_request_cross_bar.clock();

		//select next request and issue to pipline
		for(uint bank_index = 0; bank_index < _banks.size(); ++bank_index)
		{
			Bank& bank = _banks[bank_index];
			bank.data_pipline.clock();
			if(!bank.data_pipline.is_write_valid() || !_request_cross_bar.is_read_valid(bank_index)) continue;
			bank.data_pipline.write(_request_cross_bar.read(bank_index));
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
				if(!_request_cross_bar.is_write_valid(bank_index)) continue;

				MemoryReturn ret(req, &_data_u8[buffer_addr]);
				_return_cross_bar.write(ret, bank_index);
				bank.data_pipline.read();
			}
			else if(req.type == MemoryRequest::Type::STORE)
			{
				std::memcpy(&_data_u8[buffer_addr], req.data, req.size);
				bank.data_pipline.read();
			}
		}

		_return_cross_bar.clock();
	}

	bool request_port_write_valid(uint port_index) override
	{
		return _request_cross_bar.is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request, uint port_index) override
	{
		assert(request.port == port_index);
		_request_cross_bar.write(request, port_index);
	}

	bool return_port_read_valid(uint port_index) override
	{
		return _return_cross_bar.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index) override
	{
		return _return_cross_bar.peek(port_index);
	}

	const MemoryReturn read_return(uint port_index) override
	{
		return _return_cross_bar.read(port_index);
	}

private:
	paddr_t _get_buffer_addr(paddr_t paddr) { return paddr & _buffer_address_mask; }
};

}}