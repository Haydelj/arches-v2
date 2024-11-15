#pragma once 
#include "stdafx.hpp"

#include "unit-main-memory-base.hpp"
#include "util/arbitration.hpp"


#include <ramulator2/src/base/base.h>
#include <ramulator2/src/base/request.h>
#include <ramulator2/src/base/config.h>
#include <ramulator2/src/frontend/frontend.h>
#include <ramulator2/src/memory_system/memory_system.h>
#include <ramulator2/src/frontend/impl/external_wrapper/gem5_frontend.cpp>
#include <ramulator2/src/addr_mapper/impl/linear_mappers.cpp>
#include <ramulator2/src/dram_controller/impl/rowpolicy/basic_rowpolicies.cpp>
#include <ramulator2/src/memory_system/impl/generic_DRAM_system.cpp>
#include "ramulator/unit-generic-dram-controller.cpp"
#include <ramulator2/src/dram_controller/impl/scheduler/generic_scheduler.cpp>
#include <ramulator2/src/dram_controller/impl/refresh/all_bank_refresh.cpp>

#ifdef uint
#undef uint
#endif
typedef unsigned int uint;

namespace Arches { namespace Units {

class UnitDRAMRamulator : public UnitMainMemoryBase
{
public:
	struct Configuration
	{
		std::string config_path;
		uint64_t size{1ull << 30};
		uint num_ports;
		uint num_controllers;
		uint64_t partition_mask;
	};

private:
	struct RamulatorReturn
	{
		cycles_t return_cycle;
		uint return_id;

		friend bool operator<(const RamulatorReturn& l, const RamulatorReturn& r)
		{
			return l.return_cycle > r.return_cycle;
		}
	};

	struct MemoryController
	{
		Ramulator::IFrontEnd* ramulator2_frontend;
		Ramulator::IMemorySystem* ramulator2_memorysystem;
		std::priority_queue<RamulatorReturn> return_queue;
	};

	int _clock_ratio;
	uint _pending_requests = 0;
	bool _busy{false};

	paddr_t _partition_mask{0x0ull};

	std::vector<MemoryController> _controllers;
	RequestCascade _request_network;
	ReturnCascade _return_network;
	cycles_t _current_cycle{ 0 };

	std::vector<MemoryReturn> _returns;
	std::stack<uint> _free_return_ids;

public:
	UnitDRAMRamulator(Configuration config);
	virtual ~UnitDRAMRamulator() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

	void clock_rise() override;
	void clock_fall() override;

	void print_stats(uint32_t const word_size, cycles_t cycle_count);
	float total_power();

	class Log
	{
	public:
		const static uint NUM_COUNTERS = 4;
		union
		{
			struct
			{
				uint64_t loads;
				uint64_t stores;
				uint64_t bytes_read;
				uint64_t bytes_written;
			};
			uint64_t counters[NUM_COUNTERS];
		};

		Log() { reset(); }

		void reset()
		{
			for (uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for (uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];
		}

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = loads + stores;

			printf("Read Bandwidth: %.1f bytes/cycle\n", (double)bytes_read / units / cycles);
			printf("Write Bandwidth: %.1f bytes/cycle\n", (double)bytes_written / units / cycles);
			printf("\n");
			printf("Total: %lld\n", total / units);
			printf("Loads: %lld\n", loads / units);
			printf("Stores: %lld\n", stores / units);
		}
	}
	log;

private:
	bool _load(const MemoryRequest& request_item, uint channel_index);
	bool _store(const MemoryRequest& request_item, uint channel_index);
	paddr_t _convert_address(paddr_t address)
	{
		address &= ~generate_nbit_mask(log2i(CACHE_BLOCK_SIZE));
		return pext(address, ~_partition_mask);
	}
};

}}