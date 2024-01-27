#pragma once


#include "units/unit-tp.hpp"

namespace Arches { namespace Units { namespace DualStreaming
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

		uint8_t* float_regs_pending = thread.float_regs_pending;

		if(instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM1) //BOX ISECT
		{
			for(uint i = 0; i <= 11; ++i)
				if(float_regs_pending[i]) 
					return float_regs_pending[i];
		}
		else if(instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM2) //TRI ISECT
		{
			for(uint i = 0; i <= 17; ++i)
				if(float_regs_pending[i]) 
					return float_regs_pending[i];
		}
		else if(instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM3) //LWI
		{
			for(uint i = 0; i < sizeof(WorkItem) / sizeof(float); ++i)
				if(float_regs_pending[instr.rd + i])
					return float_regs_pending[instr.rd + i];
		}
		else if(instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM4) //SWI
		{
			for(uint i = 0; i < sizeof(WorkItem) / sizeof(float); ++i)
				if(float_regs_pending[instr.rs2 + i])
					return float_regs_pending[instr.rs2 + i];
		}
		else if (instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM5) { // CSHIT
			for (uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); i++) {
				if (float_regs_pending[instr.rs2 + i])
					return float_regs_pending[instr.rs2 + i];
			}
		}
		else if (instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM6) { // LHIT
			for (uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); i++) {
				if (float_regs_pending[instr.rd + i])
					return float_regs_pending[instr.rd + i];
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

		uint8_t* float_regs_pending = thread.float_regs_pending;

		//if(instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM2) //TRI ISECT
		//{
		//	for(uint i = 15; i <= 17; ++i)
		//		_float_regs_pending[i] = (uint8_t)ISA::RISCV::InstrType::CUSTOM2;
		//}
		if(instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM3) //LWI
		{
			for(uint i = 0; i < sizeof(WorkItem) / sizeof(float); ++i)
				float_regs_pending[instr.rd + i] = (uint8_t)ISA::RISCV::InstrType::CUSTOM3;
		}
		else if (instr_info.instr_type == ISA::RISCV::InstrType::CUSTOM6) //LHIT
		{
			for (uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); ++i)
				float_regs_pending[instr.rd + i] = (uint8_t)ISA::RISCV::InstrType::CUSTOM6;
		}
		else Units::UnitTP::_set_dependancies(thread_id);
	}
};

}}}