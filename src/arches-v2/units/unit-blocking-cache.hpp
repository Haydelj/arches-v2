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

		uint data_array_latency{0};

		uint num_ports{1};
		uint num_banks{1};
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
	void write_request(const MemoryRequest& request, uint port_index) override;

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
		Pipline<MemoryReturn> data_array_pipline;
		Bank(uint data_array_latency) : data_array_pipline(data_array_latency) {}
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
		uint64_t _total;
		uint64_t _hits;
		uint64_t _misses;
		uint64_t _uncached_writes;
		uint64_t _tag_array_access;
		uint64_t _data_array_reads;
		uint64_t _data_array_writes;

		Log() { reset(); }

		void reset()
		{
			_total = 0;
			_hits = 0;
			_misses = 0;
			_uncached_writes = 0;
			_tag_array_access = 0;
			_data_array_reads = 0;
			_data_array_writes = 0;
		}

		void accumulate(const Log& other)
		{
			_total += other._total;
			_hits += other._hits;
			_misses += other._misses;
			_tag_array_access += other._tag_array_access;
			_data_array_reads += other._data_array_reads;
			_data_array_writes += other._data_array_writes;
		}

		void log_requests(uint n = 1) { _total += n; } //TODO hit under miss logging

		void log_hit(uint n = 1) { _hits += n; } //TODO hit under miss logging
		void log_miss(uint n = 1) { _misses += n; }

		void log_uncached_write(uint n = 1) { _uncached_writes += n; }

		void log_tag_array_access() { _tag_array_access++; }
		void log_data_array_read() { _data_array_reads++; }
		void log_data_array_write() { _data_array_writes++; }

		uint64_t get_total() { return _hits + _misses; }
		uint64_t get_total_data_array_accesses() { return _data_array_reads + _data_array_writes; }

		void print_log(FILE* stream = stdout, uint units = 1)
		{
			uint64_t total = get_total();
			float ft = total / 100.0f;

			uint64_t da_total = get_total_data_array_accesses();

			fprintf(stream, "Total: %lld\n", total / units);
			fprintf(stream, "Hits: %lld(%.2f%%)\n", _hits / units, _hits / ft);
			fprintf(stream, "Misses: %lld(%.2f%%)\n", _misses / units, _misses / ft);
			fprintf(stream, "Tag Array Total: %lld\n", _tag_array_access);
			fprintf(stream, "Data Array Total: %lld\n", da_total);
			fprintf(stream, "Data Array Reads: %lld\n", _data_array_reads);
			fprintf(stream, "Data Array Writes: %lld\n", _data_array_writes);
		}
	}log;
};

}}