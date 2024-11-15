#pragma once 
#include "stdafx.hpp"

#include "util/arbitration.hpp"
#include "unit-cache-base.hpp"
#include "units/dual-streaming/unit-scene-buffer.hpp"

namespace Arches {
namespace Units {

class UnitCache : public UnitCacheBase
{
public:
	struct Configuration
	{
		bool in_order{false};

		uint size{1024};
		uint block_size{CACHE_BLOCK_SIZE};
		uint associativity{1};

		uint rob_size{1};
		uint num_mshr{1};
		uint input_latency{1};
		uint output_latency{1};

		uint num_ports{1};
		uint crossbar_width{1};
		uint num_partitions{1};
		uint num_banks{1};
		uint64_t partition_select_mask{0x0};
		uint64_t bank_select_mask{0x0};

		std::vector<UnitMemoryBase*> mem_highers{nullptr};
		uint                         mem_higher_port{0};
		uint                         mem_higher_port_stride{1};
	};

	struct PowerConfig
	{
		//Energy is joules, power in watts
		float tag_energy{0.0f};
		float read_energy{0.0f};
		float write_energy{0.0f};
		float leakage_power{0.0f};
	};

	UnitCache(Configuration config);
	virtual ~UnitCache();

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

protected:
	struct Bank
	{
		//request path (per bank)

		//in order logic
		std::vector<bool> rob_filled;
		uint rob_head{0};

		//out of order logic
		std::set<uint> rob_free_set;

		uint rob_size{0};
		std::vector<MemoryReturn> rob;
		std::queue<uint16_t> return_queue;
		Pipline<MemoryRequest> request_pipline;
		Pipline<MemoryReturn> return_pipline;

		Bank(Configuration config);
	};

	struct Miss
	{
		paddr_t block_addr;
		uint request_buffer_index;
	};

	struct MSHR //Miss Status Handling Register
	{
		std::queue<uint16_t> sub_entries; //linked list in a real hardware
		MSHR() = default;
	};

	struct Partition
	{
		std::vector<Bank> banks;

		//miss path (per partition)
		Cascade<Miss> miss_queue;
		uint num_mshr;
		std::map<paddr_t, MSHR> mshrs;
		uint mem_higher_port;
		std::queue<MemoryRequest> mem_higher_request_queue;
		bool recived_return;

		Partition(Configuration config);
	};

	std::vector<UnitMemoryBase*> _mem_highers;
	RequestCrossBar _request_network;
	std::vector<Partition> _partitions;
	ReturnCrossBar _return_network;

	bool _in_order;

	uint64_t _bank_select_mask{0};
	uint _get_bank(paddr_t addr)
	{
		return pext(addr, _bank_select_mask);
	}

	void _recive_return();
	void _recive_request();

	void _send_request();
	void _send_return();

	virtual UnitMemoryBase* _get_mem_higher(paddr_t addr) { return _mem_highers[0]; }

public:
	class Log
	{
	public:
		const static uint NUM_COUNTERS = 10;
		union
		{
			struct
			{
				uint64_t hits;
				uint64_t misses;
				uint64_t half_misses;
				uint64_t uncached_writes;
				uint64_t rob_stalls;
				uint64_t mshr_stalls;
				uint64_t bytes_read;
				uint64_t tag_array_access;
				uint64_t data_array_reads;
				uint64_t data_array_writes;
			};
			uint64_t counters[NUM_COUNTERS];
		};
		std::map<paddr_t, uint64_t> profile_counters;

	public:
		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;

			profile_counters.clear();
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];

			for(auto& a : other.profile_counters)
				profile_counters[a.first] += a.second;
		}

		uint64_t get_total() { return hits + half_misses + misses; }
		uint64_t get_total_data_array_accesses() { return data_array_reads + data_array_writes; }

		void print(cycles_t cycles, uint units = 1, PowerConfig power_config = PowerConfig())
		{
			uint64_t total = get_total();
			printf("Read Bandwidth: %.1f bytes/cycle\n", (float)bytes_read / units / cycles);
			printf("\n");
			printf("Total: %lld\n", total / units);
			printf("Hits: %lld (%.2f%%)\n", hits / units, 100.0f * hits / total);
			printf("Misses: %lld (%.2f%%)\n", misses / units, 100.0f * misses / total);
			printf("Half Misses: %lld (%.2f%%)\n", half_misses / units, 100.0f * half_misses / total);
			printf("\n");
			printf("Uncached writes: %lld\n", uncached_writes / units);
			printf("\n");
			printf("RB Stalls: %lld\n", rob_stalls / units);
			printf("MSHR Stalls: %lld\n", mshr_stalls / units);
			printf("\n");
			printf("Tag Array Access: %lld\n", tag_array_access / units);
			printf("Data Array Reads: %lld\n", data_array_reads / units);
			printf("Data Array Writes: %lld\n", data_array_writes / units);
		}

		float print_power(PowerConfig power_config, float time_delta, uint units = 1)
		{
			float read_energy = data_array_reads * power_config.read_energy / units;
			float write_energy = data_array_writes * power_config.write_energy / units;
			float tag_energy = tag_array_access * power_config.tag_energy / units;
			float leakage_energy = time_delta * power_config.leakage_power / units;

			float total_energy = read_energy + write_energy + tag_energy + leakage_energy;
			float total_power = total_energy / time_delta;

			printf("\n");
			printf("Total Energy: %.2f mJ\n", total_energy * 1000.0f);
			printf("Total Power: %.2f W\n", total_power);

			return total_power;
		}
	}
	log;
};

}
}