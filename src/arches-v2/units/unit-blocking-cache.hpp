#pragma once 
#include "stdafx.hpp"

#include "unit-cache-base.hpp"

namespace Arches { namespace Units {

class UnitBlockingCache : public UnitCacheBase
{
public:
	struct Configuration
	{
		uint size{1024};
		uint associativity{1};

		uint latency{1};
		uint cycle_time{1};

		uint num_ports{1};
		uint num_banks{1};
		uint cross_bar_width{1};
		uint64_t bank_select_mask{0};

		UnitMemoryBase* mem_higher{nullptr};
		uint            mem_higher_port_offset{0};
		uint            mem_higher_port_stride{1};
	};

	UnitBlockingCache(Configuration config);
	virtual ~UnitBlockingCache();

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

private:
	struct Bank
	{
		enum class State : uint8_t
		{
			IDLE,
			MISSED,
			ISSUED,
			FILLED,
		}
		state{State::IDLE};
		MemoryRequest current_request{};
		Pipline<MemoryRequest> tag_array_pipline;
		Pipline<MemoryReturn> data_array_pipline;
		Bank(uint latency, uint cycle_time) : tag_array_pipline(cycle_time, cycle_time), data_array_pipline(latency, cycle_time) {}
	};

	std::vector<Bank> _banks;
	RequestCrossBar _request_cross_bar;
	ReturnCrossBar _return_cross_bar;

	UnitMemoryBase* _mem_higher;
	uint _mem_higher_port_offset;
	uint _mem_higher_port_stride;

	void _clock_rise(uint bank_index);
	void _clock_fall(uint bank_index);

public:
	class Log
	{
	public:
		union
		{
			struct
			{
				uint64_t total;
				uint64_t hits;
				uint64_t misses;
				uint64_t uncached_writes;
				uint64_t bytes_read;
				uint64_t tag_array_access;
				uint64_t data_array_reads;
				uint64_t data_array_writes;
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

		uint64_t get_total() { return hits + misses; }
		uint64_t get_total_data_array_accesses() { return data_array_reads + data_array_writes; }

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = get_total();
			float ft = total / 100.0f;

			uint64_t da_total = get_total_data_array_accesses();

			printf("Total: %lld\n", total / units);
			printf("Hits: %lld(%.2f%%)\n", hits / units, hits / ft);
			printf("Misses: %lld(%.2f%%)\n", misses / units, misses / ft);
			printf("Tag Array Total: %lld\n", tag_array_access);
			printf("Data Array Total: %lld\n", da_total);
			printf("Data Array Reads: %lld\n", data_array_reads);
			printf("Data Array Writes: %lld\n", data_array_writes);
			printf("Data Array Writes: %lld\n", data_array_writes);
			printf("Bandwidth: %.2f Bytes/Cycle\n", (double)bytes_read / units / cycles);
		}
	}log;
};

}}