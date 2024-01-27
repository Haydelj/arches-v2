#pragma once

#include "units/unit-tp.hpp"

namespace Arches { namespace Units { namespace TRaX
{

class UnitTP : public Arches::Units::UnitTP
{
public:
	UnitTP(Units::UnitTP::Configuration config) : Units::UnitTP(config) {}

private:
	uint8_t _check_dependancies(uint thread_id) override
	{
		ThreadData& thread = _thread_data[thread_id];
		const ISA::RISCV::Instruction& instr = thread.instr;
		const ISA::RISCV::InstructionInfo& instr_info = thread.instr_info;

		uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegType::INT ? thread.int_regs_pending : thread.float_regs_pending;
		uint8_t* src_pending = instr_info.src_reg_type == ISA::RISCV::RegType::INT ? thread.int_regs_pending : thread.float_regs_pending;

		if (instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM7) // TRACERAY
		{
			for(uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); i++) 
			{
				if(dst_pending[instr.rd + i])
					return dst_pending[instr.rd + i];
			}
			for (uint i = 0; i < sizeof(rtm::Ray) / sizeof(float); i++) 
			{
				if (src_pending[instr.rs1 + i])
					return src_pending[instr.rs1 + i];
			}
		}
		else return Units::UnitTP::_check_dependancies(thread_id);

		return 0;
	}

	void _set_dependancies(uint thread_id) override
	{
		ThreadData& thread = _thread_data[thread_id];
		const ISA::RISCV::Instruction& instr = thread.instr;
		const ISA::RISCV::InstructionInfo& instr_info = thread.instr_info;

		uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegType::INT ? thread.int_regs_pending : thread.float_regs_pending;

		if (instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM7) //LHIT
		{
			for (uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); ++i)
				dst_pending[instr.rd + i] = (uint8_t)ISA::RISCV::InstrType::CUSTOM7;
		}
		else Units::UnitTP::_set_dependancies(thread_id);
	}
};

}}}