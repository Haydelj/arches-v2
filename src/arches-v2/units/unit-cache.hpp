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
		uint level{0};
		
		bool miss_alloc{false};
		bool block_prefetch{false};
		UnitCacheBase::Policy policy{UnitCacheBase::Policy::LRU};

		uint size{1024};
		uint block_size{CACHE_BLOCK_SIZE};
		uint sector_size{CACHE_SECTOR_SIZE};
		uint associativity{1};

		uint num_mshr{1};
		uint num_subentries{1};
		uint return_queue_size{32};
		uint latency{1};

		uint num_ports{1};
		uint crossbar_width{1};
		uint num_slices{1};
		uint num_banks{1};


		uint64_t slice_select_mask;
		uint64_t bank_select_mask;

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
		FIFO<MemoryReturn> return_queue;
		LatencyFIFO<MemoryRequest> request_pipline;
		LatencyFIFO<MemoryReturn> return_pipline;
		Bank(Configuration config);
	};

	struct MSHR //Miss Status Handling Register
	{
		std::queue<MemoryRequest> subentries;
		MSHR() = default;
	};

	struct Slice
	{
		std::vector<Bank> banks;

		//miss path (per partition)
		Cascade<MemoryRequest> miss_network;
		std::map<paddr_t, MSHR> mshrs;

		std::queue<MemoryRequest> mem_higher_request_queue;
		uint mem_higher_port;

		Slice(Configuration config);
	};

	std::vector<UnitMemoryBase*> _mem_highers;
	RequestCrossBar _request_network;

	std::vector<Slice> _slices;
	ReturnCrossBar _return_network;

	uint _level;
	uint _num_mshr;
	uint _num_subentries;
	bool _block_prefetch;
	bool _miss_alloc;

	uint _get_bank(paddr_t addr)
	{
		return (addr / _block_size) % _slices[0].banks.size();
	}

	void _recive_return();
	void _recive_request();
	void _send_request();

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
				uint64_t uncached_requests;
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

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = get_total();
			printf("Total: %lld\n", total / units);
			printf("Hits: %lld (%.2f%%)\n", hits / units, 100.0f * hits / total);
			printf("Half Misses: %lld (%.2f%%)\n", half_misses / units, 100.0f * half_misses / total);
			printf("Misses: %lld (%.2f%%)\n", misses / units, 100.0f * misses / total);
			printf("\n");
			printf("Uncached Requests: %lld\n", uncached_requests / units);
			printf("\n");
			printf("MSHR Stalls: %lld\n", mshr_stalls / units);
			printf("\n");
			printf("Tag Array Access: %lld\n", tag_array_access / units);
			printf("Data Array Reads: %lld\n", data_array_reads / units);
			printf("Data Array Writes: %lld\n", data_array_writes / units);
		}

		void print_short(cycles_t cycles, uint units = 1)
		{
			uint64_t total = get_total();
			printf("Hits/Half/Misses: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * hits / total, 100.0 * half_misses / total, 100.0 * total);
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