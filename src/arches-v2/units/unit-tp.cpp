#include "unit-tp.hpp"

namespace Arches {
namespace Units {

//#define ENABLE_TP_DEBUG_PRINTS (_tp_index == 0 && _tm_index == 0 && this->simulator->current_cycle > 0)
//#define ENABLE_TP_DEBUG_PRINTS (unit_id == 0x00000000000014a4 && thread_id == 0)
#define TP_PRINT_STALL_CYCLES (true)

#ifndef ENABLE_TP_DEBUG_PRINTS 
#define ENABLE_TP_DEBUG_PRINTS (false)
#endif

UnitTP::UnitTP(const Configuration& config) :
	_unit_table(*config.unit_table), 
	_unique_mems(*config.unique_mems), 
	_unique_sfus(*config.unique_sfus), 
	_cheat_memory(config.cheat_memory),
	_num_threads(config.num_threads), 
	_thread_exec_arbiter(config.num_threads),
	_thread_data(config.num_threads),
	_stack_mask(generate_nbit_mask(log2i(config.stack_size))),
	_tp_index(config.tp_index),
	_tm_index(config.tm_index),
	_num_tps_per_i_cache(config.num_tps_per_i_cache),
	log()
{
	for(uint i = 0; i < _thread_data.size(); i++)
	{
		ThreadData& thread = _thread_data[i];
		thread.stack_mem.resize(config.stack_size);
	}
}

void UnitTP::reset()
{
	log.reset();

	_num_halted_threads = 0;
	_last_thread_id = 0;

	for(uint i = 0; i < _thread_data.size(); i++)
	{
		ThreadData& thread = _thread_data[i];
		thread.int_regs.zero.u64 = 0ull;
		thread.int_regs.ra.u64 = 0ull;
		thread.int_regs.sp.u64 = 0ull;
		thread.instr.data = 0;
		thread.i_buffer.paddr = 0;

		for(uint i = 0; i < 32; ++i)
		{
			thread.int_regs_pending[i] = 0;
			thread.float_regs_pending[i] = 0;
		}

		thread.instr.data = reinterpret_cast<uint32_t*>(_cheat_memory)[thread.pc / 4];
		thread.instr_info = thread.instr.get_info();
		if(_check_dependancies(i) == 0)
			_thread_exec_arbiter.add(i);
	}

	simulator->units_executing++;
}

void UnitTP::set_entry_point(uint64_t entry_point)
{
	for(uint i = 0; i < _thread_data.size(); i++)
	{
		ThreadData& thread = _thread_data[i];
		thread.pc = entry_point;
		thread.instr.data = reinterpret_cast<uint32_t*>(_cheat_memory)[thread.pc / 4];
		thread.instr_info = thread.instr.get_info();
	}
}

void UnitTP::_clear_register_pending(uint thread_id, ISA::RISCV::DstReg dst)
{
	if(ENABLE_TP_DEBUG_PRINTS)
	{
		printf("%02d           \t                \tret \t%s       \t\n", thread_id, dst.mnemonic().c_str());
	}

	ThreadData& thread = _thread_data[thread_id];
	if (is_int(dst.type)) thread.int_regs_pending[dst.index] = 0;
	else                  thread.float_regs_pending[dst.index] = 0;

	if(_check_dependancies(thread_id) == 0)
		 _thread_exec_arbiter.add(thread_id);
}

void UnitTP::_process_load_return(const MemoryReturn& ret)
{
	BitStack58 dst = ret.dst;
	uint ret_thread_id = dst.pop(4);
	ISA::RISCV::DstReg dst_reg(dst.pop(9));
	ThreadData& ret_thread = _thread_data[ret_thread_id];
	for(uint offset = 0, i = 0; offset < ret.size; ++i)
	{
		_clear_register_pending(ret_thread_id, dst_reg);
		write_register(&ret_thread.int_regs, &ret_thread.float_regs, dst_reg, ret.data + offset);
		offset += size(dst_reg.type);
		dst_reg.index++;
	}
}

uint8_t UnitTP::_check_dependancies(uint thread_id)
{
	ThreadData& thread = _thread_data[thread_id];

	const ISA::RISCV::Instruction& instr = thread.instr;
	const ISA::RISCV::InstructionInfo& instr_info = thread.instr_info;

	uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegFile::INT ? thread.int_regs_pending : thread.float_regs_pending;
	uint8_t* src_pending = instr_info.src_reg_type == ISA::RISCV::RegFile::INT ? thread.int_regs_pending : thread.float_regs_pending;

	switch (thread.instr_info.encoding)
	{
	case ISA::RISCV::Encoding::R:
		if (dst_pending[instr.rd]) return dst_pending[instr.rd];
		if (src_pending[instr.rs1]) return src_pending[instr.rs1];
		if (src_pending[instr.rs2]) return src_pending[instr.rs2];
		break;

	case ISA::RISCV::Encoding::R4:
		if (dst_pending[instr.rd]) return dst_pending[instr.rd];
		if (src_pending[instr.rs1]) return src_pending[instr.rs1];
		if (src_pending[instr.rs2]) return src_pending[instr.rs2];
		if (src_pending[instr.rs3]) return src_pending[instr.rs3];
		break;

	case ISA::RISCV::Encoding::I:
		if (dst_pending[instr.rd]) return dst_pending[instr.rd];
		if (src_pending[instr.rs1]) return src_pending[instr.rs1];
		break;

	case ISA::RISCV::Encoding::S:
		if (dst_pending[instr.rs2]) return dst_pending[instr.rs2];
		if (src_pending[instr.rs1]) return src_pending[instr.rs1];
		break;

	case ISA::RISCV::Encoding::B:
		if (src_pending[instr.rs1]) return src_pending[instr.rs1];
		if (src_pending[instr.rs2]) return src_pending[instr.rs2];
		break;

	case ISA::RISCV::Encoding::U:
		if (dst_pending[instr.rd]) return dst_pending[instr.rd];
		break;

	case ISA::RISCV::Encoding::J:
		if (dst_pending[instr.rd]) return dst_pending[instr.rd];
		break;
	}

	thread.int_regs_pending[0] = 0;
	return 0;
}

void UnitTP::_set_dependancies(uint thread_id)
{
	ThreadData& thread = _thread_data[thread_id];
	const ISA::RISCV::Instruction& instr = thread.instr;
	const ISA::RISCV::InstructionInfo& instr_info = thread.instr_info;

	uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegFile::INT ? thread.int_regs_pending : thread.float_regs_pending;
	if ((instr_info.encoding == ISA::RISCV::Encoding::B) || (instr_info.encoding == ISA::RISCV::Encoding::S)) return;
	dst_pending[instr.rd] = (uint8_t)instr_info.instr_type;
}

void UnitTP::_log_instruction_issue(uint thread_id)
{
	ThreadData& thread = _thread_data[thread_id];
	log.log_instruction_issue(thread.instr_info.instr_type, thread.pc);

#if 1
	if (ENABLE_TP_DEBUG_PRINTS)
	{
		printf("%02d  %05I64x: \t%08x          \t", thread_id, thread.pc, thread.instr.data);
		thread.instr_info.print_instr(thread.instr);
		printf("\n");
	}
#endif
}

bool UnitTP::_decode(uint thread_id)
{
	DecodePhase phase;
	ISA::RISCV::InstrType type;
	return _decode(thread_id, type, phase);
}

bool UnitTP::_decode(uint thread_id, ISA::RISCV::InstrType& stalling_instr_type, DecodePhase& phase)
{
	ThreadData& thread = _thread_data[thread_id];

	//Check for data hazards
	ISA::RISCV::InstrType type = (ISA::RISCV::InstrType)_check_dependancies(thread_id);
	if(type != ISA::RISCV::InstrType::NA)
	{
		phase = DecodePhase::DATA_HAZARD;
		stalling_instr_type = (ISA::RISCV::InstrType)type;
		return false;
	}

	//check for pipline hazards
	if(_unit_table[(uint)thread.instr_info.instr_type])
	{
		if(thread.instr_info.exec_type == ISA::RISCV::ExecType::EXECUTABLE)
		{
			//check for pipline hazard
			UnitSFU* sfu = (UnitSFU*)_unit_table[(uint)thread.instr_info.instr_type];
			if(!sfu->request_port_write_valid(_tp_index))
			{
				phase = DecodePhase::PIPLINE_HAZARD;
				stalling_instr_type = thread.instr_info.instr_type;
				return false;
			}
		}
		else if(thread.instr_info.exec_type == ISA::RISCV::ExecType::MEMORY)
		{
			if(thread.int_regs.registers[thread.instr.rs1].u64 < (~0x0ull << 20))
			{
				//check for pipline hazard
				UnitMemoryBase* mem = (UnitMemoryBase*)_unit_table[(uint)thread.instr_info.instr_type];
				if(!mem->request_port_write_valid(_tp_index))
				{
					phase = DecodePhase::PIPLINE_HAZARD;
					stalling_instr_type = thread.instr_info.instr_type;
					return false;
				}
			}
		}
	}

	//no hazards found
	return true;
}

void UnitTP::clock_rise()
{
	for (auto& unit : _unique_mems)
	{
		if (!unit->return_port_read_valid(_tp_index)) continue;
		const MemoryReturn ret = unit->read_return(_tp_index);
		_process_load_return(ret);
	}

	for (auto& unit : _unique_sfus)
	{
		if (!unit->return_port_read_valid(_tp_index)) continue;
		SFURequest ret = unit->read_return(_tp_index);
		uint thread_id = ret.dst.pop(4);
		ISA::RISCV::DstReg dst_reg(ret.dst.pop(9));
		_clear_register_pending(thread_id, dst_reg);
	}
}

void UnitTP::clock_fall()
{
	uint thread_id = _thread_exec_arbiter.get_index();
	if(thread_id == ~0u) thread_id = _last_thread_id;
	ThreadData& thread = _thread_data[thread_id];

	DecodePhase stall_phase;
	ISA::RISCV::InstrType stall_type;
	if(!_decode(thread_id, stall_type, stall_phase))
	{
		//log stall
		if(stall_phase == DecodePhase::DATA_HAZARD)
		{
			log.log_data_stall(stall_type, thread.pc);
		}
		else if(stall_phase == DecodePhase::PIPLINE_HAZARD)
		{
			log.log_resource_stall(stall_type, thread.pc);
		}

		if (ENABLE_TP_DEBUG_PRINTS && TP_PRINT_STALL_CYCLES)
		{
			printf("\033[31m%02d  %05I64x: \t%08x          \t", thread_id, thread.pc, thread.instr.data);
			thread.instr_info.print_instr(thread.instr);
			     if(stall_phase == DecodePhase::DATA_HAZARD)    printf("\t%s data hazard!",    ISA::RISCV::InstructionTypeNameDatabase::get_instance()[stall_type].c_str());
			else if(stall_phase == DecodePhase::PIPLINE_HAZARD) printf("\t%s pipline hazard!", ISA::RISCV::InstructionTypeNameDatabase::get_instance()[stall_type].c_str());
			printf("\033[0m\n");
		}

		if(_thread_exec_arbiter.num_pending() != 0)
		{
			_thread_exec_arbiter.remove(thread_id);
			_thread_exec_arbiter.add(thread_id);
		}

		return;
	}

	_log_instruction_issue(thread_id);
	ISA::RISCV::ExecutionItem exec_item = {thread.pc, &thread.int_regs, &thread.float_regs};

	//Execute
	bool jump = false;
	if (thread.instr_info.exec_type == ISA::RISCV::ExecType::CONTROL_FLOW) //SYS is the first non memory instruction type so this divides mem and non mem ops
	{
		if(thread.instr_info.execute_branch(exec_item, thread.instr))
		{
			jump = true;
			thread.pc = exec_item.pc;
		}
	}
	else if (thread.instr_info.exec_type == ISA::RISCV::ExecType::EXECUTABLE)
	{
		thread.instr_info.execute(exec_item, thread.instr);
		
		//Because of forwarding instruction with latency 1 don't cause stalls so we don't need to set the pending bit
		UnitSFU* sfu = (UnitSFU*)_unit_table[(uint)thread.instr_info.instr_type];
		if (sfu)
		{
			//Issue to SFU
			SFURequest req;
			ISA::RISCV::DstReg dst_reg(thread.instr.rd, (thread.instr_info.dst_reg_type == ISA::RISCV::RegFile::FLOAT) ? ISA::RISCV::RegType::FLOAT32 : ISA::RISCV::RegType::UINT32);
			req.dst.push(dst_reg.u9, 9);
			req.dst.push(thread_id, 4);
			req.port = _tp_index;

			_set_dependancies(thread_id);
			sfu->write_request(req);
		} 
	}
	else if (thread.instr_info.exec_type == ISA::RISCV::ExecType::MEMORY)
	{
		MemoryRequest req = thread.instr_info.generate_request(exec_item, thread.instr);
		if (req.vaddr < (~0x0ull << 20))
		{
			_assert(req.vaddr < 4ull * 1024ull * 1024ull * 1024ull);
			req.dst.push(thread_id, 4);
			req.port = _tp_index;
			if(thread.instr_info.instr_type == ISA::RISCV::InstrType::STORE)
				req.flags.omit_cache = 0b111;
			_set_dependancies(thread_id);

			UnitMemoryBase* mem = (UnitMemoryBase*)_unit_table[(uint)thread.instr_info.instr_type];
			mem->write_request(req);
		}
		else
		{
			if ((req.vaddr | _stack_mask) != ~0x0ull) printf("STACK OVERFLOW!!!\n"), _assert(false);
			if (thread.instr_info.instr_type == ISA::RISCV::InstrType::LOAD)
			{
				//Because of forwarding instruction with latency 1 don't cause stalls so we don't need to set pending bit
				paddr_t buffer_addr = req.vaddr & _stack_mask;
				write_register(&thread.int_regs, &thread.float_regs, req.dst.pop(9), &thread.stack_mem[buffer_addr]);
			}
			else if (thread.instr_info.instr_type == ISA::RISCV::InstrType::STORE)
			{
				paddr_t buffer_addr = req.vaddr & _stack_mask;
				std::memcpy(&thread.stack_mem[buffer_addr], req.data, req.size);
			}
			else _assert(false);
		}
	}
	else _assert(false);

	if(!jump) thread.pc += 4;
	thread.int_regs.zero.u64 = 0x0ull; //Compilers generate instructions with zero register as target so we need to zero the register every cycle

	if(thread.pc == 0x0ull)
	{
		thread.instr.data = 0;
		_thread_exec_arbiter.remove(thread_id);
		if(++_num_halted_threads == _num_threads)
			--simulator->units_executing;
	}
	else
	{
		thread.instr.data = reinterpret_cast<uint32_t*>(_cheat_memory)[thread.pc / 4];
		thread.instr_info = thread.instr.get_info();
		if(_check_dependancies(thread_id) != 0)
			_thread_exec_arbiter.remove(thread_id);
	}

	_last_thread_id = thread_id;
}

}
}