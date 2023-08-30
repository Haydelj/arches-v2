#pragma once 
#include "../../stdafx.hpp"

#include "../util/arbitration.hpp"
#include "unit-cache-base.hpp"

namespace Arches { namespace Units {

class UnitL1Cache : public UnitCacheBase
{
public:
	struct Configuration
	{
		uint size{1024};
		uint associativity{1};

		uint tag_array_access_cycles{1};
		uint data_array_access_cycles{1};

		uint num_ports{1};
		uint port_size{CACHE_BLOCK_SIZE};
		uint num_banks{1};
		uint num_lfb{1};

		MemoryMap mem_map;

		float dynamic_read_energy;
		float bank_leakge_power;
	};

	UnitL1Cache(Configuration config);
	virtual ~UnitL1Cache();

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request, uint port_index) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn& read_return(uint port_index) override;

private:
	struct _LFB //Line Fill Buffer
	{
		struct SubEntry
		{
			uint8_t  offset;
			uint8_t  size;
			uint16_t dst;

			uint     port;
		};

		enum class Type : uint8_t
		{
			READ,
			WRITE, //TODO: support write through. This will need to write the store to the buffer then load the background data and write it back to the tag array
			//we can reuse some of read logic to do this. It is basically a read that always needs to be commited at the end (hit or miss).
			//in the furture we might also want to support cache coherency
			WRITE_COMBINING,
		};

		enum class State : uint8_t
		{
			INVALID,
			EMPTY,
			MISSED,
			ISSUED,
			FILLED,
			RETIRED,
		};

		_BlockData block_data;
		addr_t block_addr{~0ull};

		uint64_t write_mask{0x0};
		std::queue<SubEntry> sub_entries;

		uint8_t lru{0u};
		Type type{Type::READ};
		State state{State::INVALID};

		_LFB() = default;

		bool operator==(const _LFB& other) const
		{
			return block_addr == other.block_addr && type == other.type;
		}

		uint64_t get_next_write(uint8_t& offset, uint8_t& size)
		{
			assert(type == _LFB::Type::WRITE_COMBINING);

			//try to find largest contiguous alligned block to issue store
			size = CACHE_BLOCK_SIZE;
			while(size > 0)
			{
				uint64_t comb_mask = generate_nbit_mask(size);
				for(offset = 0; offset < CACHE_BLOCK_SIZE; offset += size)
				{
					if((comb_mask & write_mask) == comb_mask)
					{
						write_mask &= ~(comb_mask);
						return 1;
					}

					comb_mask <<= size;
				}
				size /= 2;
			}

			return 0;
		}
	};

	struct _Bank
	{
		std::vector<_LFB> lfbs;
		std::queue<uint> lfb_request_queue;
		std::queue<uint> lfb_return_queue;
		uint64_t outgoing_write_mask;
		_Bank(uint num_lfb) : lfbs(num_lfb) {}
	};


	Configuration _configuration; //nice for debugging

	uint _bank_index_offset;
	uint64_t _bank_index_mask;

	uint _data_array_access_cycles, _tag_array_access_cycles;

	std::vector<_Bank> _banks;
	MemoryMap _mem_map;

	CrossBar<MemoryRequest> _request_cross_bar;
	CrossBar<MemoryReturn>  _return_cross_bar;

	ArbitrationNetwork64 incoming_return_network;
	ArbitrationNetwork64 outgoing_request_network;

	void _push_request(_LFB& lfb, const MemoryRequest& request, uint port);
	MemoryRequest _pop_request(_LFB& lfb, uint& port);

	uint _fetch_lfb(uint bank_index, _LFB& lfb);
	uint _allocate_lfb(uint bank_index, _LFB& lfb);
	uint _fetch_or_allocate_lfb(uint bank_index, uint64_t block_addr, _LFB::Type type);

	void _interconnects_rise();
	void _propagate_ack();
	void _interconnects_fall();

	bool _proccess_return(uint bank_index);
	bool _proccess_request(uint bank_index);

	void _try_request_lfb(uint bank_index);
	void _try_return_lfb(uint bank_index);

	paddr_t _get_bank_index(paddr_t paddr) { return (paddr >> _bank_index_offset) & _bank_index_mask; }

public:
	class Log
	{
	public:
		uint64_t _total;
		uint64_t _hits;
		uint64_t _misses;
		uint64_t _lfb_hits;
		uint64_t _half_misses;
		uint64_t _bank_conflicts;
		uint64_t _lfb_stalls;
		uint64_t _tag_array_access;
		uint64_t _data_array_reads;
		uint64_t _data_array_writes;

		Log() { reset(); }

		void reset()
		{
			_total = 0;
			_lfb_hits = 0;
			_hits = 0;
			_misses = 0;
			_half_misses = 0;
			_bank_conflicts = 0;
			_lfb_stalls = 0;
			_tag_array_access = 0;
			_data_array_reads = 0;
			_data_array_writes = 0;
		}

		void accumulate(const Log& other)
		{
			_total += other._total;
			_lfb_hits += other._lfb_hits;
			_hits += other._hits;
			_misses += other._misses;
			_half_misses += other._half_misses;;
			_bank_conflicts += other._bank_conflicts;
			_lfb_stalls += other._lfb_stalls;
			_tag_array_access += other._tag_array_access;
			_data_array_reads += other._data_array_reads;
			_data_array_writes += other._data_array_writes;
		}

		void log_requests(uint n = 1) { _total += n; } //TODO hit under miss logging

		void log_hit(uint n = 1) { _hits += n; } //TODO hit under miss logging
		void log_miss(uint n = 1) { _misses += n; }

		void log_lfb_hit(uint n = 1) { _lfb_hits += n; }
		void log_half_miss(uint n = 1) { _half_misses += n; }

		void log_bank_conflict() { _bank_conflicts++; }
		void log_lfb_stall() { _lfb_stalls++; }

		void log_tag_array_access() { _tag_array_access++; }
		void log_data_array_read() { _data_array_reads++; }
		void log_data_array_write() { _data_array_writes++; }

		uint64_t get_total() { return _hits + _misses + _lfb_hits + _half_misses; }
		uint64_t get_total_data_array_accesses() { return _data_array_reads + _data_array_writes; }

		void print_log(FILE* stream = stdout, uint units = 1)
		{
			uint64_t total = get_total();
			float ft = total / 100.0f;

			uint64_t da_total = get_total_data_array_accesses();

			fprintf(stream, "Total: %lld\n", total / units);
			fprintf(stream, "Total2: %lld\n", _total / units);
			fprintf(stream, "Hits: %lld(%.2f%%)\n", _hits / units, _hits / ft);
			fprintf(stream, "Misses: %lld(%.2f%%)\n", _misses / units, _misses / ft);
			fprintf(stream, "LFB Hits: %lld(%.2f%%)\n", _lfb_hits / units, _lfb_hits / ft);
			fprintf(stream, "Half Misses: %lld(%.2f%%)\n", _half_misses / units, _half_misses / ft);
			fprintf(stream, "Bank Conflicts: %lld\n", _bank_conflicts / units);
			fprintf(stream, "LFB Stalls: %lld\n", _lfb_stalls / units);
			fprintf(stream, "Tag Array Total: %lld\n", _tag_array_access);
			fprintf(stream, "Data Array Total: %lld\n", da_total);
			fprintf(stream, "Data Array Reads: %lld\n", _data_array_reads);
			fprintf(stream, "Data Array Writes: %lld\n", _data_array_writes);
		}
	}log;
};

}}