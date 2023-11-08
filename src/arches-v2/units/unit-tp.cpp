#include "unit-tp.hpp"

namespace Arches { namespace Units {

//#define ENABLE_TP_DEBUG_PRINTS (_tp_index == 0 && _tm_index == 0)
//#define ENABLE_TP_DEBUG_PRINTS (unit_id == 0x0000000000000383)

#ifndef ENABLE_TP_DEBUG_PRINTS 
#define ENABLE_TP_DEBUG_PRINTS (false)
#endif

UnitTP::UnitTP(const Configuration& config) :unit_table(*config.unit_table), unique_mems(*config.unique_mems), unique_sfus(*config.unique_sfus), log(0x10000)
{
	_int_regs.zero.u64 = 0x0;
	_int_regs.sp.u64 = config.sp;
	_int_regs.ra.u64 = 0x0ull;
	_int_regs.gp.u64 = config.gp;
	_pc = config.pc;

	_cheat_memory = config.cheat_memory;

	_tp_index = config.tp_index;
	_tm_index = config.tm_index;

	_stack_mem.resize(config.stack_size);
	_stack_mask = generate_nbit_mask(log2i(config.stack_size));
	
	for(uint i = 0; i < 32; ++i)
	{
		_int_regs_pending[i] = 0;
		_float_regs_pending[i] = 0;
	}
}

void UnitTP::_clear_register_pending(const ISA::RISCV::RegAddr& dst)
{
	     if(dst.reg_type == ISA::RISCV::RegType::INT)   _int_regs_pending[dst.reg] = 0;
	else if(dst.reg_type == ISA::RISCV::RegType::FLOAT) _float_regs_pending[dst.reg] = 0;
}

uint8_t UnitTP::_check_dependancies(const ISA::RISCV::Instruction& instr, const ISA::RISCV::InstructionInfo& instr_info)
{
	uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegType::INT ? _int_regs_pending : _float_regs_pending;
	uint8_t* src_pending = instr_info.src_reg_type == ISA::RISCV::RegType::INT ? _int_regs_pending : _float_regs_pending;

	switch(instr_info.encoding)
	{
	case ISA::RISCV::Encoding::R:
		if(dst_pending[instr.rd]) return dst_pending[instr.rd];
		if(src_pending[instr.rs1]) return src_pending[instr.rs1];
		if(src_pending[instr.rs2]) return src_pending[instr.rs2];
		break;

	case ISA::RISCV::Encoding::R4:
		if(dst_pending[instr.rd]) return dst_pending[instr.rd];
		if(src_pending[instr.rs1]) return src_pending[instr.rs1];
		if(src_pending[instr.rs2]) return src_pending[instr.rs2];
		if(src_pending[instr.rs3]) return src_pending[instr.rs3];
		break;

	case ISA::RISCV::Encoding::I:
		if(dst_pending[instr.rd]) return dst_pending[instr.rd];
		if(src_pending[instr.rs1]) return src_pending[instr.rs1];
		break;

	case ISA::RISCV::Encoding::S:
		if(dst_pending[instr.rs2]) return dst_pending[instr.rs2];
		if(src_pending[instr.rs1]) return src_pending[instr.rs1];
		break;

	case ISA::RISCV::Encoding::B:
		if(src_pending[instr.rs1]) return src_pending[instr.rs1];
		if(src_pending[instr.rs2]) return src_pending[instr.rs2];
		break;

	case ISA::RISCV::Encoding::U:
		if(dst_pending[instr.rd]) return dst_pending[instr.rd];
		break;

	case ISA::RISCV::Encoding::J:
		if(dst_pending[instr.rd]) return dst_pending[instr.rd];
		break;
	}

	_int_regs_pending[0] = 0;
	return 0;
}

void UnitTP::_set_dependancies(const ISA::RISCV::Instruction& instr, const ISA::RISCV::InstructionInfo& instr_info)
{
	uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegType::INT ? _int_regs_pending : _float_regs_pending;
	if((instr_info.encoding == ISA::RISCV::Encoding::B) || (instr_info.encoding == ISA::RISCV::Encoding::S)) return;
	dst_pending[instr.rd] = (uint8_t)instr_info.instr_type;
}

void UnitTP::_process_load_return(const MemoryReturn& ret)
{
	if(ENABLE_TP_DEBUG_PRINTS)
	{
		printf("                                                                                                    \r");
		printf("           \t                \tRETURN \t%s       \t\n", ISA::RISCV::RegAddr(ret.dst).mnemonic().c_str());
		if(ret.vaddr == 0x0ull)
			_thread_id = ret.data_u32;
	}

	ISA::RISCV::RegAddr reg_addr(ret.dst);
	if(reg_addr.reg_type == ISA::RISCV::RegType::FLOAT)
	{
		for(uint i = 0; i < ret.size / sizeof(float); ++i)
		{
			write_register(&_int_regs, &_float_regs, reg_addr, 4, ret.data + i * 4);
			_clear_register_pending(reg_addr);
			reg_addr.reg++;
		}
	}
	else
	{
		write_register(&_int_regs, &_float_regs, reg_addr, ret.size, ret.data);
		_clear_register_pending(reg_addr);
	}
}

void UnitTP::_log_instruction_issue(const ISA::RISCV::Instruction& instr, const ISA::RISCV::InstructionInfo& instr_info, const ISA::RISCV::ExecutionItem& exec_item)
{
	log.log_instruction_issue(instr_info, exec_item.pc);

#if 1
	if(ENABLE_TP_DEBUG_PRINTS)
	{
		printf("    %05I64x: \t%08x          \t", exec_item.pc, instr.data);
		instr_info.print_instr(instr);
		instr_info.print_regs(instr, exec_item);
	}
#endif
}

void UnitTP::clock_rise()
{
	for(auto& unit : unique_mems)
	{
		if(!unit->return_port_read_valid(_tp_index)) continue;
		const MemoryReturn ret = unit->read_return(_tp_index);
		_process_load_return(ret);
	}

	for(auto& unit : unique_sfus)
	{
		if(!unit->return_port_read_valid(_tp_index)) continue;
		const SFURequest& ret = unit->read_return(_tp_index);
		_clear_register_pending(ret.dst);
	}
}

void UnitTP::clock_fall()
{
FREE_INSTR:
	if(_pc == 0x0ull) return;

	//Fetch
	const ISA::RISCV::Instruction instr(reinterpret_cast<uint32_t*>(_cheat_memory)[_pc / 4]);

	//Decode
	const ISA::RISCV::InstructionInfo instr_info = instr.get_info();

	//Reg/PC read
	ISA::RISCV::ExecutionItem exec_item = {_pc, &_int_regs, &_float_regs};

	if(ENABLE_TP_DEBUG_PRINTS)
	{
		printf("\033[31m    %05I64x: \t%08x          \t", exec_item.pc, instr.data);
		instr_info.print_instr(instr);
		printf("\033[0m\r");
	}

	//Check for data hazard
	if(uint8_t type = _check_dependancies(instr, instr_info))
	{
		log.log_data_stall(type, exec_item.pc);
		return;
	}

	//Check for resource hazards
	if(instr_info.exec_type == ISA::RISCV::ExecType::CONTROL_FLOW) //SYS is the first non memory instruction type so this divides mem and non mem ops
	{
		_log_instruction_issue(instr, instr_info, exec_item);
		if(ENABLE_TP_DEBUG_PRINTS) printf("\n");

		if(instr_info.execute_branch(exec_item, instr))
		{
			//jumping to address 0 is the halt condition
			_pc = exec_item.pc;
			if(_pc == 0x0ull) simulator->units_executing--;
		}
		else _pc += 4;
		_int_regs.zero.u64 = 0x0ull; //Compiler generate jalr with zero register as target so we need to zero the register after all control flow
	}
	else if(instr_info.exec_type == ISA::RISCV::ExecType::EXECUTABLE)
	{
		UnitSFU* sfu = (UnitSFU*)unit_table[(uint)instr_info.instr_type];
		if(sfu && !sfu->request_port_write_valid(_tp_index))
		{
			log.log_resource_stall(instr_info, exec_item.pc);
			return;
		}

		//Execute
		_log_instruction_issue(instr, instr_info, exec_item);
		if(ENABLE_TP_DEBUG_PRINTS) printf("\n");

		instr_info.execute(exec_item, instr);
		_pc += 4;
		_int_regs.zero.u64 = 0x0ull;

		//Issue to SFU
		ISA::RISCV::RegAddr reg_addr;
		reg_addr.reg = instr.rd;
		reg_addr.reg_type = instr_info.dst_reg_type;

		SFURequest req;
		req.dst = reg_addr.u8;
		req.port = _tp_index;

		if(sfu)
		{
			_set_dependancies(instr, instr_info);
			sfu->write_request(req, req.port);
		}
		else
		{
			//Because of forwarding instruction with latency 1 don't cause stalls so we don't need to set the pending bit
		}
	}
	else if(instr_info.exec_type == ISA::RISCV::ExecType::MEMORY)
	{
		UnitMemoryBase* mem = (UnitMemoryBase*)unit_table[(uint)instr_info.instr_type];

		if (ENABLE_TP_DEBUG_PRINTS) {
			std::cout << "1231231\n" << '\n';
		}

		if(!mem->request_port_write_valid(_tp_index))
		{
			log.log_resource_stall(instr_info, exec_item.pc);
			return;
		}

		//Executing memory instructions spawns a request
		_log_instruction_issue(instr, instr_info, exec_item);
		MemoryRequest req = instr_info.generate_request(exec_item, instr);
		req.port = _tp_index;
		_pc += 4;

		if(req.vaddr < (~0x0ull << 20))
		{
			if(ENABLE_TP_DEBUG_PRINTS)
			{
				if(req.type == MemoryRequest::Type::LOAD)
					printf(" \t(0x%llx)\n", req.vaddr);
				else if(req.type == MemoryRequest::Type::STORE)
					printf(" \t(0x%llx)\n", req.vaddr);
				else
					printf("\n");
			}

			assert(req.vaddr < 4ull * 1024ull * 1024ull * 1024ull);

			_set_dependancies(instr, instr_info);
			mem->write_request(req, req.port);
		}
		else
		{
			if(ENABLE_TP_DEBUG_PRINTS) printf("\n");

			if((req.vaddr | _stack_mask) != ~0ull)
			{
				printf("STACK OVERFLOW!!!\n");
				assert(false);
			}

			if(instr_info.instr_type == ISA::RISCV::InstrType::LOAD)
			{
				//Because of forwarding instruction with latency 1 don't cause stalls so we don't need to set pending bit
				paddr_t buffer_addr = req.vaddr & _stack_mask;
				write_register(&_int_regs, &_float_regs, req.dst, req.size, &_stack_mem[buffer_addr]);
			}
			else if(instr_info.instr_type == ISA::RISCV::InstrType::STORE)
			{
				paddr_t buffer_addr = req.vaddr & _stack_mask;
				std::memcpy(&_stack_mem[buffer_addr], req.data, req.size);
			}
			else
			{
				assert(false);
			}
		}
	}
	else assert(false);
}

}}