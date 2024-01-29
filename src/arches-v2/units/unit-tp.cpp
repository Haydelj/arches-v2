#include "unit-tp.hpp"

namespace Arches {
namespace Units {

//#define ENABLE_TP_DEBUG_PRINTS (_tp_index == 0 && _tm_index == 0)
//#define ENABLE_TP_DEBUG_PRINTS (unit_id == 0x0000000000000383)

#ifndef ENABLE_TP_DEBUG_PRINTS 
#define ENABLE_TP_DEBUG_PRINTS (false)
#endif

UnitTP::UnitTP(const Configuration& config) :
	_unit_table(*config.unit_table), 
	_unique_mems(*config.unique_mems), 
	_unique_sfus(*config.unique_sfus), 
	_inst_cache(config.inst_cache), 
	log(0x10000), 
	_num_threads(config.num_threads), 
	_thread_fetch_arbiter(config.num_threads),
	_thread_exec_arbiter(config.num_threads),
	_num_halted_threads(0), 
	_last_thread_id(0)
{
	for (int i = 0; i < config.num_threads; i++) 
	{
		ThreadData thread = {};
		thread.int_regs.zero.u64 = 0;
		thread.int_regs.sp.u64 = config.sp;
		thread.int_regs.ra.u64 = 0x0ull;
		thread.int_regs.gp.u64 = config.gp;
		thread.pc = config.pc;
		thread.cheat_memory = config.cheat_memory;
		thread.stack_mem.resize(config.stack_size);
		thread.stack_mask = generate_nbit_mask(log2i(config.stack_size));
		thread.instr.data = 0;

		for (uint i = 0; i < 32; ++i)
		{
			thread.int_regs_pending[i] = 0;
			thread.float_regs_pending[i] = 0;
		}

		_thread_data.push_back(thread);
		_thread_fetch_arbiter.add(i);
	}

	_num_tps_per_i_cache = config.num_tps_per_i_cache;
	_tp_index = config.tp_index;
	_tm_index = config.tm_index;
}

void UnitTP::_clear_register_pending(uint thread_id, ISA::RISCV::RegAddr dst)
{
	if(ENABLE_TP_DEBUG_PRINTS)
	{
		printf("%02d           \t                \tret \t%s       \t\n", thread_id, dst.mnemonic().c_str());
	}

	ThreadData& thread = _thread_data[thread_id];
	if      (dst.reg_type == ISA::RISCV::RegType::INT)  thread.int_regs_pending[dst.reg] = 0;
	else if (dst.reg_type == ISA::RISCV::RegType::FLOAT) thread.float_regs_pending[dst.reg] = 0;
}

void UnitTP::_process_load_return(const MemoryReturn& ret)
{
	uint16_t ret_thread_id = ret.dst >> 8;
	ThreadData& ret_thread = _thread_data[ret_thread_id];
	ISA::RISCV::RegAddr reg_addr((uint8_t)ret.dst);
	if(reg_addr.reg_type == ISA::RISCV::RegType::FLOAT)
	{
		for(uint i = 0; i < ret.size / sizeof(float); ++i)
		{
			write_register(&ret_thread.int_regs, &ret_thread.float_regs, reg_addr, sizeof(float), ret.data + i * sizeof(float));
			_clear_register_pending(ret_thread_id, reg_addr);
			reg_addr.reg++;
		}
	}
	else
	{
		write_register(&ret_thread.int_regs, &ret_thread.float_regs, reg_addr, ret.size, ret.data);
		_clear_register_pending(ret_thread_id, reg_addr);
	}
}

uint8_t UnitTP::_check_dependancies(uint thread_id)
{
	ThreadData& thread = _thread_data[thread_id];
	const ISA::RISCV::Instruction& instr = thread.instr;
	const ISA::RISCV::InstructionInfo& instr_info = thread.instr_info;

	uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegType::INT ? thread.int_regs_pending : thread.float_regs_pending;
	uint8_t* src_pending = instr_info.src_reg_type == ISA::RISCV::RegType::INT ? thread.int_regs_pending : thread.float_regs_pending;

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

	uint8_t* dst_pending = instr_info.dst_reg_type == ISA::RISCV::RegType::INT ? thread.int_regs_pending : thread.float_regs_pending;
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

uint8_t UnitTP::_decode(uint thread_id)
{
	ThreadData& thread = _thread_data[thread_id];

	//Check for instruction fetch hazard
	if(thread.pc == 0x0ull) return (uint8_t)ISA::RISCV::InstrType::INSRT_FETCH;
	if(thread.instr.data == 0)
	{
		if(_inst_cache == nullptr)
		{
			assert(thread.cheat_memory != nullptr);
			thread.instr.data = reinterpret_cast<uint32_t*>(thread.cheat_memory)[thread.pc / 4];
		}
		else
		{
			paddr_t addr_offset = thread.pc - thread.i_buffer.paddr;
			if(addr_offset < CACHE_BLOCK_SIZE)
			{
				thread.instr.data = reinterpret_cast<uint32_t*>(thread.i_buffer.data)[addr_offset / 4];
			}
			else return (uint8_t)ISA::RISCV::InstrType::INSRT_FETCH;
		}
		thread.instr_info = thread.instr.get_info();
	}

	//Check for data hazards
	if(uint8_t type = _check_dependancies(thread_id))
		return type;

	//check for pipline hazards
	if(_unit_table[(uint)thread.instr_info.instr_type])
	{
		if(thread.instr_info.exec_type == ISA::RISCV::ExecType::EXECUTABLE)
		{
			//check for pipline hazard
			UnitSFU* sfu = (UnitSFU*)_unit_table[(uint)thread.instr_info.instr_type];
			if(!sfu->request_port_write_valid(_tp_index))
				return 128 + (uint)thread.instr_info.instr_type;
		}
		else if(thread.instr_info.exec_type == ISA::RISCV::ExecType::MEMORY)
		{
			//check for pipline hazard
			UnitMemoryBase* mem = (UnitMemoryBase*)_unit_table[(uint)thread.instr_info.instr_type];
			if(!mem->request_port_write_valid(_tp_index))
				return 128 + (uint)thread.instr_info.instr_type;
		}
	}

	//no hazards found
	return 0;
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
		const SFURequest& ret = unit->read_return(_tp_index);
		_clear_register_pending(ret.dst >> 8, (uint8_t)ret.dst);
	}

	if(_inst_cache)
	{
		uint i_cache_port = _tp_index % _num_tps_per_i_cache;
		if(_inst_cache->return_port_read_valid(i_cache_port))
		{
			const MemoryReturn ret = _inst_cache->read_return(i_cache_port);

			uint thread_id = ret.dst;
			ThreadData& thread = _thread_data[thread_id];
			std::memcpy(thread.i_buffer.data, ret.data, CACHE_BLOCK_SIZE);
			thread.i_buffer.paddr = ret.paddr;
		}
	}
}

void UnitTP::clock_fall()
{
	//Fetch next i-buffer
	uint fetch_thread_id = _thread_fetch_arbiter.get_index();
	if(fetch_thread_id != ~0u)
	{
		ThreadData& fetch_thread = _thread_data[fetch_thread_id];
		uint i_cache_port = _tp_index % _num_tps_per_i_cache;
		if(_inst_cache)
		{
			if(_inst_cache->request_port_write_valid(i_cache_port))
			{
				MemoryRequest i_req;
				i_req.paddr = fetch_thread.pc & ~0x3full;
				i_req.port = i_cache_port;
				i_req.dst = fetch_thread_id;
				i_req.type = MemoryRequest::Type::LOAD;
				i_req.size = CACHE_BLOCK_SIZE;
				_inst_cache->write_request(i_req);
				_thread_fetch_arbiter.remove(fetch_thread_id);
			}
		}
		else
		{
			_thread_fetch_arbiter.remove(fetch_thread_id);
		}
	}

	for(uint i = 0; i < _num_threads; ++i)
		if(!_decode(i)) _thread_exec_arbiter.add(i);
		else            _thread_exec_arbiter.remove(i);

	uint exec_thread_id = _thread_exec_arbiter.get_index();
	if(exec_thread_id == ~0u)
	{
		//log data stall
		ThreadData& last_thread = _thread_data[_last_thread_id];
		uint8_t last_thread_stall_type = _decode(_last_thread_id);
		if(last_thread_stall_type < 128)
		{
			if(last_thread.pc != 0)
			{
				log.log_data_stall((ISA::RISCV::InstrType)last_thread_stall_type, last_thread.pc);
			}
		}
		else
		{
			log.log_resource_stall((ISA::RISCV::InstrType)(last_thread_stall_type - 128), last_thread.pc);
		}

		if (ENABLE_TP_DEBUG_PRINTS && last_thread.instr.data != 0)
		{
			printf("\033[31m%02d  %05I64x: \t%08x          \t", _last_thread_id, last_thread.pc, last_thread.instr.data);
			last_thread.instr_info.print_instr(last_thread.instr);
			if(last_thread_stall_type < 128) printf("\t%s data hazard!",    ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)last_thread_stall_type].c_str());
			else                             printf("\t%s pipline hazard!", ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)(last_thread_stall_type - 128)].c_str());
			printf("\033[0m\n");
		}

		_last_thread_id = (_last_thread_id + 1) % _num_threads;
		return;
	}

	//Reg/PC read
	ThreadData& thread = _thread_data[exec_thread_id];
	_log_instruction_issue(exec_thread_id);
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

		//Issue to SFU
		ISA::RISCV::RegAddr reg_addr;
		reg_addr.reg = thread.instr.rd;
		reg_addr.reg_type = thread.instr_info.dst_reg_type;

		SFURequest req;
		req.dst = (exec_thread_id << 8) | reg_addr.u8;
		req.port = _tp_index;

		//Because of forwarding instruction with latency 1 don't cause stalls so we don't need to set the pending bit
		UnitSFU* sfu = (UnitSFU*)_unit_table[(uint)thread.instr_info.instr_type];
		if (sfu)
		{
			_set_dependancies(exec_thread_id);
			sfu->write_request(req);
		} 
	}
	else if (thread.instr_info.exec_type == ISA::RISCV::ExecType::MEMORY)
	{
		MemoryRequest req = thread.instr_info.generate_request(exec_item, thread.instr);
		req.dst = (exec_thread_id << 8) | req.dst;
		req.port = _tp_index;

		if (req.vaddr < (~0x0ull << 20))
		{
			assert(req.vaddr < 4ull * 1024ull * 1024ull * 1024ull);
			UnitMemoryBase* mem = (UnitMemoryBase*)_unit_table[(uint)thread.instr_info.instr_type];
			_set_dependancies(exec_thread_id);
			mem->write_request(req);
		}
		else
		{
			if ((req.vaddr | thread.stack_mask) != ~0x0ull) printf("STACK OVERFLOW!!!\n"), assert(false);
			if (thread.instr_info.instr_type == ISA::RISCV::InstrType::LOAD)
			{
				//Because of forwarding instruction with latency 1 don't cause stalls so we don't need to set pending bit
				paddr_t buffer_addr = req.vaddr & thread.stack_mask;
				write_register(&thread.int_regs, &thread.float_regs, req.dst, req.size, &thread.stack_mem[buffer_addr]);
			}
			else if (thread.instr_info.instr_type == ISA::RISCV::InstrType::STORE)
			{
				paddr_t buffer_addr = req.vaddr & thread.stack_mask;
				std::memcpy(&thread.stack_mem[buffer_addr], req.data, req.size);
			}
			else assert(false);
		}
	}
	else assert(false);

	if(!jump) thread.pc += 4;
	thread.int_regs.zero.u64 = 0x0ull; //Compilers generate instructions with zero register as target so we need to zero the register every cycle
	thread.instr.data = 0;
	_last_thread_id = exec_thread_id;

	if(thread.pc == 0x0ull)
	{
		//thread halted add to halt mask
		_num_halted_threads++;
		if(_num_halted_threads == _num_threads)
			--simulator->units_executing;
	} 
	else if((thread.pc - thread.i_buffer.paddr) >= CACHE_BLOCK_SIZE)
	{
		_thread_fetch_arbiter.add(exec_thread_id); //queue i buffer fetch
	}
}

}
}