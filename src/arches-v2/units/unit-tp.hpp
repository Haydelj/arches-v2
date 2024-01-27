#pragma once

#include "stdafx.hpp"

#include "unit-base.hpp"
#include "unit-memory-base.hpp"
#include "unit-sfu.hpp"

#include "isa/riscv.hpp"

#include "util/bit-manipulation.hpp"

namespace Arches {
namespace Units {

class UnitTP : public UnitBase
{
public:
	struct Configuration
	{
		vaddr_t pc{ 0x0 };
		vaddr_t sp{ 0x0 };
		vaddr_t gp{ 0x0 };

		uint8_t* cheat_memory{ nullptr };

		uint tp_index{ 0 };
		uint tm_index{ 0 };

		uint num_threads{ 8 };

		uint stack_size{ 512 };

		const std::vector<UnitBase*>* unit_table;
		const std::vector<UnitSFU*>* unique_sfus;
		const std::vector<UnitMemoryBase*>* unique_mems;
		UnitMemoryBase* inst_cache{nullptr};
		uint num_tps_per_i_cache;
	};

protected:

	struct ThreadData
	{
		ISA::RISCV::IntegerRegisterFile       int_regs{};
		ISA::RISCV::FloatingPointRegisterFile float_regs{};
		vaddr_t                               pc{};

		uint8_t* cheat_memory{nullptr};
		struct IBuffer
		{
			uint8_t data[CACHE_BLOCK_SIZE];
			paddr_t paddr{0};
		}i_buffer;

		ISA::RISCV::Instruction instr{0x0ull};
		ISA::RISCV::InstructionInfo instr_info;

		uint8_t float_regs_pending[32];
		uint8_t int_regs_pending[32];

		std::vector<uint8_t> stack_mem;
		uint64_t stack_mask;
	};


	uint _tp_index;
	uint _tm_index;
	uint _num_tps_per_i_cache;

	uint _last_thread_id;
	uint _num_threads;
	uint _num_halted_threads;
	RoundRobinArbiter _thread_exec_arbiter;
	RoundRobinArbiter _thread_fetch_arbiter;
	std::vector<ThreadData> _thread_data;

	const std::vector<UnitBase*>& _unit_table;
	const std::vector<UnitSFU*>& _unique_sfus;
	const std::vector<UnitMemoryBase*>& _unique_mems;
	UnitMemoryBase* _inst_cache{nullptr};

public:
	UnitTP(const Configuration& config);

	void clock_rise() override;
	void clock_fall() override;

protected:
	uint8_t _decode(uint thread_id);
	virtual uint8_t _check_dependancies(uint thread_id);
	virtual void _set_dependancies(uint thread_id);
	void _process_load_return(const MemoryReturn& ret);
	void _clear_register_pending(uint thread_id, ISA::RISCV::RegAddr dst);
	void _log_instruction_issue(uint thread_id);

public:
	class Log
	{
	protected:
		vaddr_t _elf_start_addr;
		std::vector<uint64_t> _profile_counters;
		uint _instr_index{ 0 };

		uint64_t _cycles;
		uint64_t _instruction_counters[static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES)];
		uint64_t _resource_stall_counters[static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES)];
		uint64_t _data_stall_counters[static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES)];

	public:
		Log(uint64_t elf_start_addr) : _elf_start_addr(elf_start_addr) { reset(); }

		void reset()
		{
			_cycles = 0;
			for (uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				_instruction_counters[i] = 0;
				_resource_stall_counters[i] = 0;
				_data_stall_counters[i] = 0;
				_profile_counters.clear();
			}
		}

		void accumulate(const Log& other)
		{
			_cycles += other._cycles;
			for (uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				_instruction_counters[i] += other._instruction_counters[i];
				_resource_stall_counters[i] += other._resource_stall_counters[i];
				_data_stall_counters[i] += other._data_stall_counters[i];
			}

			_profile_counters.resize(std::max(_profile_counters.size(), other._profile_counters.size()), 0ull);
			for (uint i = 0; i < other._profile_counters.size(); ++i)
			{
				_profile_counters[i] += other._profile_counters[i];
			}
		}

		void profile_instruction(vaddr_t pc)
		{
			assert(pc >= _elf_start_addr);

			uint instr_index = (pc - _elf_start_addr) / 4;
			if (instr_index >= _profile_counters.size())
				_profile_counters.resize(instr_index + 1, 0ull);

			_profile_counters[instr_index]++;
		}

		void log_instruction_issue(const ISA::RISCV::InstrType type, vaddr_t pc)
		{
			_cycles++;
			_instruction_counters[(uint)type]++;
			profile_instruction(pc);
		}

		void log_resource_stall(const ISA::RISCV::InstrType type, vaddr_t pc)
		{
			_cycles++;
			_resource_stall_counters[(uint)type]++;
			profile_instruction(pc);
		}

		void log_data_stall(const ISA::RISCV::InstrType type, vaddr_t pc)
		{
			_cycles++;
			_data_stall_counters[(uint)type]++;
			profile_instruction(pc);
		}

		void print_log(FILE* stream = stdout, uint num_units = 1)
		{
			uint64_t total = 0;

			std::vector<std::pair<const char*, uint64_t>> _instruction_counter_pairs;
			for (uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				total += _instruction_counters[i];
				_instruction_counter_pairs.push_back({ ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i].c_str(), _instruction_counters[i] });
			}
			std::sort(_instruction_counter_pairs.begin(), _instruction_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			fprintf(stream, "Issue Cycles (%.2f%%)\n", 100.0f * total / _cycles);
			fprintf(stream, "\tTotal: %lld\n", total / num_units);
			for (uint i = 0; i < _instruction_counter_pairs.size(); ++i)
				if (_instruction_counter_pairs[i].second) fprintf(stream, "\t%s: %lld (%.2f%%)\n", _instruction_counter_pairs[i].first, _instruction_counter_pairs[i].second / num_units, static_cast<float>(_instruction_counter_pairs[i].second) / total * 100.0f);

			total = 0;

			std::vector<std::pair<const char*, uint64_t>> _resource_stall_counter_pairs;
			for (uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				total += _resource_stall_counters[i];
				_resource_stall_counter_pairs.push_back({ ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i].c_str(), _resource_stall_counters[i] });
			}
			std::sort(_resource_stall_counter_pairs.begin(), _resource_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			fprintf(stream, "\nPipeline Stall Cycles (%.2f%%)\n", 100.0f * total / _cycles);
			fprintf(stream, "\tTotal: %lld \n", total / num_units);
			for (uint i = 0; i < _resource_stall_counter_pairs.size(); ++i)
				if (_resource_stall_counter_pairs[i].second) fprintf(stream, "\t%s: %lld (%.2f%%)\n", _resource_stall_counter_pairs[i].first, _resource_stall_counter_pairs[i].second / num_units, static_cast<float>(_resource_stall_counter_pairs[i].second) / total * 100.0f);

			total = 0;

			std::vector<std::pair<const char*, uint64_t>> _data_stall_counter_pairs;
			for (uint i = 0; i < static_cast<size_t>(ISA::RISCV::InstrType::NUM_TYPES); ++i)
			{
				total += _data_stall_counters[i];
				_data_stall_counter_pairs.push_back({ ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i].c_str(), _data_stall_counters[i] });
			}
			std::sort(_data_stall_counter_pairs.begin(), _data_stall_counter_pairs.end(),
				[](const std::pair<const char*, uint64_t>& a, const std::pair<const char*, uint64_t>& b) -> bool { return a.second > b.second; });

			fprintf(stream, "\nData Stall Cycles (%.2f%%)\n", 100.0f * total / _cycles);
			fprintf(stream, "\tTotal: %lld\n", total / num_units);
			for (uint i = 0; i < _data_stall_counter_pairs.size(); ++i)
				if (_data_stall_counter_pairs[i].second) fprintf(stream, "\t%s: %lld (%.2f%%)\n", _data_stall_counter_pairs[i].first, _data_stall_counter_pairs[i].second / num_units, static_cast<float>(_data_stall_counter_pairs[i].second) / total * 100.0f);
		}

		void print_profile(uint8_t* backing_memory, FILE* stream = stdout)
		{
			uint64_t total = 0;

			fprintf(stream, "Profile\n");
			for (uint i = 0; i < _profile_counters.size(); ++i) total += _profile_counters[i];
			for (uint i = 0; i < _profile_counters.size(); ++i)
			{
				//fetch
				if (_profile_counters[i] > 0)
				{
					vaddr_t pc = i * 4 + _elf_start_addr;

					ISA::RISCV::Instruction instr(reinterpret_cast<uint32_t*>(backing_memory)[pc / 4]);
					const ISA::RISCV::InstructionInfo instr_info = instr.get_info();

					float precent = 100.0f * (float)_profile_counters[i] / total;
					if (precent > 1.0f) fprintf(stream, "*\t");
					else fprintf(stream, " \t");

					fprintf(stream, "%05I64x(%05.02f%%):          \t", pc, precent);
					instr_info.print_instr(instr, stream);
					fprintf(stream, "\n");
				}
			}
		}
	}log;
};

}
}