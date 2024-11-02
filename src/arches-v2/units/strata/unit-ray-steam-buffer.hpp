#pragma once 
#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-main-memory-base.hpp"
#include "strata-kernel/ray-data.hpp"

namespace Arches { namespace Units { namespace STRaTA {

struct RayBuffer
{
	struct Header
	{
		paddr_t next_buffer{0};
		uint32_t treelet_id{0};
	};
	Header header;
	std::vector<RayData> rays;

	void write_ray(const RayData& ray)
	{
		rays.push_back(ray);
	}
	void pop_ray()
	{
		rays.pop_back();
	}
	uint32_t size()
	{
		return rays.size();
	}
};

class UnitRayStreamBuffer : public UnitMainMemoryBase
{
public:
	struct Configuration
	{
		uint64_t size{1024};
		uint num_tm{1};
		uint num_banks{1};
		uint latency{1};
	};
	uint64_t _max_size{1024};
	UnitRayStreamBuffer(const Configuration& config) : UnitMainMemoryBase(config.size), _max_size(config.size),
		_request_network(config.num_tm, config.num_banks), _return_network(config.num_banks, config.num_tm), _banks(config.num_banks, config.latency)
	{
		_buffer_address_mask = generate_nbit_mask(log2i(config.size));
		_tm_buffer_table.resize(config.num_tm, ~0u);
		_idle_ray_buffer.clear();
	}

	struct Bank
	{
		Pipline<MemoryRequest> data_pipline;
		Bank(uint latency) : data_pipline(latency) {}
	};

	std::map<uint32_t, RayBuffer> _ray_buffers{};	// map treelet index to ray buffer
	uint64_t _ray_buffers_size{0};
	
private:
	RequestCascade _request_network;
	ReturnCascade _return_network;
	std::vector<Bank> _banks;
	uint64_t _buffer_address_mask;
	std::vector<uint32_t> _tm_buffer_table;		// <tm_id, treelet_id> pair
	std::set<uint32_t> _idle_ray_buffer;

	void issue_returns();
	void _proccess_request(uint bank_index);

public:
	void clock_rise() override;
	void clock_fall() override;

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

	MemoryReturn allocate_ray_buffer(uint bank_index, const MemoryRequest& req);

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

}}}