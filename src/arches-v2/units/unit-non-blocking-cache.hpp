#pragma once 
#include "stdafx.hpp"

#include "util/arbitration.hpp"
#include "unit-cache-base.hpp"
#include "units/dual-streaming/unit-scene-buffer.hpp"

namespace Arches {
namespace Units {

class UnitNonBlockingCache : public UnitCacheBase
{
public:
	struct Configuration
	{
		uint size{1024};
		uint block_size{CACHE_BLOCK_SIZE};
		uint associativity{1};

		uint latency{1};
		uint cycle_time{1};

		uint num_ports{1};
		uint num_banks{1};
		uint64_t bank_select_mask{0};

		uint num_mshr{1};
		bool use_lfb{false};

		std::vector<UnitMemoryBase*> mem_highers{nullptr};
		uint                         mem_higher_port_offset{0};
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

	UnitNonBlockingCache(Configuration config);
	virtual ~UnitNonBlockingCache();

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

protected:
	struct MSHR //Miss Status Handling Register
	{
		struct SubEntry
		{
			uint64_t  dst;
			uint16_t  port;
			uint8_t   size;
			uint8_t   offset;
		};

		enum class Type : uint8_t
		{
			READ,
			WRITE, //TODO: support write through. This will need to write the store to the buffer then load the background data and write it back to the tag array
			//we can reuse some of read logic to do this. It is basically a read that always needs to be commited at the end (hit or miss).
			//in the furture we might also want to support cache coherency
			WRITE_COMBINING, //only valid in LFB mode
		};

		enum class State : uint8_t
		{
			INVALID,
			EMPTY,
			DATA_ARRAY,
			MISSED,
			FILLED,
			RETIRED,
		};

		addr_t block_addr{~0ull};
		std::queue<SubEntry> sub_entries;
		Type type{Type::READ};
		State state{State::INVALID};

		uint8_t lru{0u};  //This is used for LFB (Line Fill Buffer) mode
		uint128_t write_mask{0x0}; //This is used for LFB mode
		uint8_t block_data[128]; //This is used for LFB mode

		MSHR() = default;

		bool operator==(const MSHR& other) const
		{
			return block_addr == other.block_addr && type == other.type;
		}
	};

	struct Bank
	{
		std::vector<MSHR> mshrs;
		std::queue<uint> mshr_request_queue;
		std::queue<uint> mshr_return_queue;
		std::queue<MemoryRequest> uncached_write_queue;
		Pipline<uint> data_array_pipline;
		Bank(uint num_lfb, uint latency) : mshrs(num_lfb), data_array_pipline(latency - 1) {}
	};

	bool _use_lfb;
	std::vector<Bank> _banks;
	RequestCrossBar _request_cross_bar;
	ReturnCrossBar _return_cross_bar;

	std::vector<UnitMemoryBase*> _mem_highers;
	uint _mem_higher_port_offset;
	uint _mem_higher_port_stride;

	void _push_request(MSHR& lfb, const MemoryRequest& request);
	MemoryRequest _pop_request(MSHR& lfb);

	uint _fetch_lfb(uint bank_index, MSHR& lfb);
	uint _allocate_lfb(uint bank_index, MSHR& lfb);
	uint _fetch_or_allocate_mshr(uint bank_index, uint64_t block_addr, MSHR::Type type);

	void _clock_data_array(uint bank_index);

	bool _proccess_return(uint bank_index);
	bool _proccess_request(uint bank_index);

	bool _try_request_lfb(uint bank_index);
	void _try_forward_writes(uint bank_index);
	void _try_return_lfb(uint bank_index);

	virtual UnitMemoryBase* _get_mem_higher(paddr_t addr) { return _mem_highers[0]; }

public:
	class Log
	{
	public:
		const static uint NUM_COUNTERS = 11;
		union
		{
			struct
			{
				uint64_t requests;
				uint64_t hits;
				uint64_t misses;
				uint64_t half_misses;
				uint64_t uncached_writes;
				uint64_t lfb_hits;
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

		uint64_t get_total() { return hits + misses; }
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
			//printf("LFB Hits: %lld (%.2f%%)\n", lfb_hits / units, 100.0f * lfb_hits / total);
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