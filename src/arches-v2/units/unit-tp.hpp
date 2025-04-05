#pragma once

#include "stdafx.hpp"

#include "unit-base.hpp"
#include "unit-memory-base.hpp"
#include "unit-sfu.hpp"

#include "isa/riscv.hpp"

#include "util/bit-manipulation.hpp"

namespace Arches {
namespace Units {

#define ENABLE_PROFILER 0

class UnitTP : public UnitBase
{
public:
	struct Configuration
	{
		uint8_t* cheat_memory{nullptr};

		uint tp_index{0};
		uint tm_index{0};
		uint num_tps_per_i_cache{1};

		uint num_threads{8};
		uint stack_size{512};

		const std::vector<UnitBase*>* unit_table{nullptr};
		const std::vector<UnitSFU*>* unique_sfus{nullptr};
		const std::vector<UnitMemoryBase*>* unique_mems{nullptr};
		UnitMemoryBase* inst_cache{nullptr};
	};

protected:
	struct ThreadData
	{
		ISA::RISCV::IntegerRegisterFile       int_regs;
		ISA::RISCV::FloatingPointRegisterFile float_regs;
		std::vector<uint8_t>                  stack_mem;
		vaddr_t                               pc;

		uint8_t float_regs_pending[32];
		uint8_t int_regs_pending[32];

		ISA::RISCV::Instruction instr;
		ISA::RISCV::InstructionInfo instr_info;

		struct IBuffer
		{
			uint8_t data[CACHE_BLOCK_SIZE];
			paddr_t paddr;
		}
		i_buffer;
	};

	uint _tp_index;
	uint _tm_index;
	uint _num_tps_per_i_cache;
	uint64_t _stack_mask;
	uint8_t* _cheat_memory;

	uint _last_thread_id;
	uint _num_threads;
	uint _num_halted_threads;
	RoundRobinArbiter<uint16_t> _thread_exec_arbiter;
	std::vector<ThreadData> _thread_data;

	const std::vector<UnitBase*>& _unit_table;
	const std::vector<UnitSFU*>& _unique_sfus;
	const std::vector<UnitMemoryBase*>& _unique_mems;

	MemoryRequest coalescing_buffer;

public:
	UnitTP(const Configuration& config);

	void clock_rise() override;
	void clock_fall() override;
	void reset() override;
	void set_entry_point(uint64_t entry_point);

protected:
	enum class DecodePhase : uint8_t
	{
		INSTR_FETCH,
		DATA_HAZARD,
		PIPLINE_HAZARD,
	};

	bool _decode(uint thread_id);
	bool _decode(uint thread_id, ISA::RISCV::InstrType& stalling_instr_type, DecodePhase& phase);
	virtual uint8_t _check_dependancies(uint thread_id);
	virtual void _set_dependancies(uint thread_id);
	void _process_load_return(const MemoryReturn& ret);
	void _clear_register_pending(uint thread_id, ISA::RISCV::DstReg dst);
	void _log_instruction_issue(uint thread_id);

public:
	class Log
	{
	public:
		uint64_t instruction_counters[(size_t)ISA::RISCV::InstrType::NUM_TYPES];
		uint64_t _resource_stall_counters[(size_t)ISA::RISCV::InstrType::NUM_TYPES];
		uint64_t _data_stall_counters[(size_t)ISA::RISCV::InstrType::NUM_TYPES];
		std::map<paddr_t, uint64_t> _profile_counters;

	public:
		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				instruction_counters[i] = 0;
				_resource_stall_counters[i] = 0;
				_data_stall_counters[i] = 0;
			}

			_profile_counters.clear();
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				instruction_counters[i] += other.instruction_counters[i];
				_resource_stall_counters[i] += other._resource_stall_counters[i];
				_data_stall_counters[i] += other._data_stall_counters[i];
			}

			for(auto& a : other._profile_counters)
				_profile_counters[a.first] += a.second;
		}

		void profile_instruction(vaddr_t pc)
		{
		#if ENABLE_PROFILER
			_profile_counters[pc]++;
		#endif
		}

		void log_instruction_issue(const ISA::RISCV::InstrType type, vaddr_t pc)
		{
			instruction_counters[(uint)type]++;
			profile_instruction(pc);
		}

		void log_resource_stall(const ISA::RISCV::InstrType type, vaddr_t pc)
		{
			_resource_stall_counters[(uint)type]++;
			profile_instruction(pc);
		}

		void log_data_stall(const ISA::RISCV::InstrType type, vaddr_t pc)
		{
			_data_stall_counters[(uint)type]++;
			profile_instruction(pc);
		}

		void print(uint num_units = 1)
		{
			uint64_t issue_cycles = 0;
			std::vector<std::pair<const char*, uint64_t>> instruction_counter_pairs;
			for(uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				instruction_counter_pairs.push_back({ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i].c_str(), instruction_counters[i]});
				issue_cycles += instruction_counters[i];
			}
			std::sort(instruction_counter_pairs.begin(), instruction_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t dstall_cycles = 0;
			std::vector<std::pair<const char*, uint64_t>> data_stall_counter_pairs;
			for(uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				data_stall_counter_pairs.push_back({ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i].c_str(), _data_stall_counters[i]});
				dstall_cycles += _data_stall_counters[i];
			}
			std::sort(data_stall_counter_pairs.begin(), data_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t pstall_cycles = 0;
			std::vector<std::pair<const char*, uint64_t>> pipline_stall_counter_pairs;
			for(uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				pipline_stall_counter_pairs.push_back({ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i].c_str(), _resource_stall_counters[i]});
				pstall_cycles += _resource_stall_counters[i];
			}
			std::sort(pipline_stall_counter_pairs.begin(), pipline_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			uint64_t total_cycles = issue_cycles + pstall_cycles + dstall_cycles;
			printf("Issue Cycles: %lld (%.2f%%)\n", issue_cycles / num_units, 100.0f * issue_cycles / total_cycles);
			for(uint i = 0; i < instruction_counter_pairs.size(); ++i)
				if(instruction_counter_pairs[i].second)
					printf("\t%s: %lld (%.2f%%)\n", instruction_counter_pairs[i].first, instruction_counter_pairs[i].second / num_units, 100.0f * instruction_counter_pairs[i].second / total_cycles);

			printf("\nData Stall Cycles: %lld (%.2f%%)\n", dstall_cycles / num_units, 100.0f * dstall_cycles / total_cycles);
			for(uint i = 0; i < data_stall_counter_pairs.size(); ++i)
				if(data_stall_counter_pairs[i].second)
					printf("\t%s: %lld (%.2f%%)\n", data_stall_counter_pairs[i].first, data_stall_counter_pairs[i].second / num_units, 100.0f * data_stall_counter_pairs[i].second / total_cycles);
		
			printf("\nPipeline Stall Cycles: %lld (%.2f%%)\n", pstall_cycles / num_units, 100.0f * pstall_cycles / total_cycles);
			for(uint i = 0; i < pipline_stall_counter_pairs.size(); ++i)
				if(pipline_stall_counter_pairs[i].second)
					printf("\t%s: %lld (%.2f%%)\n", pipline_stall_counter_pairs[i].first, pipline_stall_counter_pairs[i].second / num_units, 100.0f * pipline_stall_counter_pairs[i].second / total_cycles);
		}

		void print_profile(uint8_t* backing_memory, FILE* stream = stdout)
		{
		#if ENABLE_PROFILER
			uint64_t total = 0;
			for(auto& counter : _profile_counters)
				total += counter.second;

			for(auto& counter : _profile_counters)
			{
				//fetch
				ISA::RISCV::Instruction instr(((uint32_t*)backing_memory)[counter.first / 4]);
				const ISA::RISCV::InstructionInfo instr_info = instr.get_info();

				float precent = 100.0f * counter.second / total;
				if     (precent > 4.0f / _profile_counters.size()) fprintf(stream, "*\t");
				else if(precent > 1.0f / _profile_counters.size()) fprintf(stream, ".\t");
				else                                              fprintf(stream, " \t");

				fprintf(stream, "%05I64x(%05.02f%%):          \t", counter.first, precent);
				instr_info.print_instr(instr, stream);
				fprintf(stream, "\n");
			}
		#else
			fprintf(stream, "PROFILING DISABLED!\n");
		#endif
		}
	}log;
};

}
}