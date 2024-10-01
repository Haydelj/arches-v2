#pragma once 
#include "stdafx.hpp"

#include "ramulator/base/base.h"
#include "ramulator/base/request.h"
#include "ramulator/base/config.h"
#include "ramulator/frontend/frontend.h"
#include "ramulator/memory_system/memory_system.h"

#include "unit-base.hpp"
#include "unit-main-memory-base.hpp"
#include "util/arbitration.hpp"

namespace Arches { namespace Units {

#define NUM_DRAM_CHANNELS (8)

class UnitDRAMRamulator : public UnitMainMemoryBase
{
private:
	std::string config_path;
	Ramulator::IFrontEnd* ramulator2_frontend;
	Ramulator::IMemorySystem* ramulator2_memorysystem;
	int clock_ratio;

	//float memory_tCK = ramulator2_memorysystem->get_tCK();

	struct RamulatorReturn
	{
		cycles_t return_cycle;
		uint return_id;

		friend bool operator<(const RamulatorReturn& l, const RamulatorReturn& r)
		{
			return l.return_cycle > r.return_cycle;
		}
	};

	struct Channel
	{
		std::priority_queue<RamulatorReturn> return_queue;
	};

	bool _busy{false};

	std::vector<Channel> _channels;
	RequestCascade _request_network;
	ReturnCascade _return_network;
	cycles_t _current_cycle{ 0 };

	std::vector<MemoryReturn> returns;
	std::stack<uint> free_return_ids;

	unsigned int log_base2(unsigned int new_value)
	{
		int i;
		for (i = 0; i < 32; i++)
		{
			new_value >>= 1;
			if (new_value == 0)
				break;
		}
		return i;
	}

public:
	UnitDRAMRamulator(uint num_clients, uint64_t size);
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
		std::map<std::pair<std::string, std::string>, uint64_t> request_logs;

		Log() { reset(); }

		void reset()
		{
			for (uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;

			request_logs.clear();
		}

		void accumulate(const Log& other)
		{
			for (uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];
			
			for (auto& a : other.request_logs)
			{
				request_logs[a.first] += a.second;
			}
		}

		void print_request_logs(cycles_t cycles, uint units = 1)
		{
			printf("\n====================== Detailed Bandwidth Utilization: ======================\n\n");
			for (auto& a : request_logs)
			{
				auto [unit_name, request_label] = a.first;
				uint64_t bytes = a.second;
				printf("Unit name: %s, Request label: %s, Bandwidth Utilization: %.1f bytes/cycle\n", unit_name.c_str(), request_label.c_str(), (double)bytes / units / cycles);
			}
			printf("\n=============================================================================\n\n");
		}

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = loads + stores;

			print_request_logs(cycles);

			printf("Read Bandwidth: %.1f bytes/cycle\n", (double)bytes_read / units / cycles);
			printf("Write Bandwidth: %.1f bytes/cycle\n", (double)bytes_written / units / cycles);

			printf("\n");
			printf("Total: %lld\n", total / units);
			printf("Loads: %lld\n", loads / units);
			printf("Stores: %lld\n", stores / units);
		}
	}
	log;
	const std::string unit_name = "DRAM";

private:
	bool _load(const MemoryRequest& request_item, uint channel_index);
	bool _store(const MemoryRequest& request_item, uint channel_index);
};

}}