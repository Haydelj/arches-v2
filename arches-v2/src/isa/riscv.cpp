#include "riscv.hpp"

#include <intrin.h>

#include "errors.hpp"
#include "../util/bit-manipulation.hpp"

namespace Arches { namespace ISA { namespace RISCV {

InstructionTypeNameDatabase* InstructionTypeNameDatabase::_instance = nullptr;

#define CALLBACK_GETSTR(STR) [](Instruction const& /*instr*/) { return STR; }
#define CALLBACK_SUBISA(TO_INSTR_INF) [](Instruction const& instr) -> const char* { return TO_INSTR_INF.get_mnemonic(instr); }

#define META_NOTI [](Instruction const& instr) -> InstructionInfo const& { throw ErrNotImplInstr(instr); }
#define IMPL_NONE [](Instruction const& instr, ExecutionItem* /*unit*/) -> void { throw ErrNoSuchInstr (instr); }
#define IMPL_NOTI [](Instruction const& instr, ExecutionItem* /*unit*/) -> void { throw ErrNotImplInstr(instr); }

const InstructionInfo Instruction::get_info() const
{
	//ignore two low bits which are only used for the C extension
	return isa[opcode >> 2].resolve(*this);
}

//RV64I
int64_t sign_extend_12_to_64(int32_t in)
{
	return (in >> 11) ? (in | (~0x0ull << 11)) : (int64_t)in;
}

int64_t sign_extend_13_to_64(int32_t in)
{
	return (in >> 12) ? (in | (~0x0ull << 12)) : (int64_t)in;
}

int64_t sign_extend_21_to_64(int32_t in)
{
	return (in >> 20) ? (in | (~0x0ull << 20)) : (int64_t)in;
}

int64_t sign_extend_32_to_64(int32_t in)
{
	return (int64_t)in;
}

int64_t i_imm(Instruction instr)
{
	return sign_extend_12_to_64(
		(instr.i.imm_11_0 << 0));
}

int64_t s_imm(Instruction instr)
{
	return sign_extend_12_to_64(
		(instr.s.imm_11_5 << 5) |
		(instr.s.imm_4_0  << 0));
}

int64_t b_imm(Instruction instr)
{
	return sign_extend_13_to_64(
		(instr.b.imm_12   << 12) |
		(instr.b.imm_11   << 11) |
		(instr.b.imm_10_5 <<  5) |
		(instr.b.imm_4_1  <<  1));
}

int64_t u_imm(Instruction instr)
{
	return sign_extend_32_to_64(
		(instr.u.imm_31_12 << 12));
}

int64_t j_imm(Instruction instr)
{
	return sign_extend_21_to_64(
		(instr.j.imm_20    << 20) |
		(instr.j.imm_19_12 << 12) |
		(instr.j.imm_11    << 11) |
		(instr.j.imm_10_1  <<  1));
}

template <typename T> 
MemoryRequest _prepare_load(ExecutionItem* unit, Instruction const& instr)
{
	MemoryRequest req;
	req.type = MemoryRequest::Type::LOAD;
	req.size = sizeof(T);
	req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

	RegAddr dst_reg;
	dst_reg.reg = instr.i.rd;
	if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
	{
		dst_reg.reg_type = RegType::FLOAT;
		dst_reg.sign_ext = 0;
	}
	else
	{
		dst_reg.reg_type = RegType::INT;
		dst_reg.sign_ext = std::is_signed_v<T>;
	}

	req.dst = dst_reg.u8;

	return req;
}

template <typename T> 
MemoryRequest _prepare_store(ExecutionItem* unit, Instruction const& instr)
{
	MemoryRequest req;
	req.type = MemoryRequest::Type::STORE;
	req.size = sizeof(T);
	req.write_mask = generate_nbit_mask(sizeof(T));
	req.vaddr = unit->int_regs->registers[instr.s.rs1].u64 + s_imm(instr);

	if(typeid(T) == typeid(float) || typeid(T) == typeid(double))
	{
		std::memcpy(req.data, &unit->float_regs->registers[instr.s.rs2], sizeof(T));
	}
	else
	{
		std::memcpy(req.data, &unit->int_regs->registers[instr.s.rs2], sizeof(T));
	}

	return req;
}

template <typename T, MemoryRequest::Type TYPE>
MemoryRequest _prepare_amo(ExecutionItem* unit, Instruction const& instr)
{
	MemoryRequest req;
	req.type = TYPE;
	req.size = sizeof(T);
	req.vaddr = unit->int_regs->registers[instr.rs1].u64;

	RegAddr dst_reg;
	dst_reg.reg = instr.i.rd;
	dst_reg.reg_type = RegType::INT;
	dst_reg.sign_ext = std::is_signed_v<T>;
	req.dst = dst_reg.u8;

	std::memcpy(req.data, &unit->int_regs->registers[instr.rs2], sizeof(T));

	return req;
}

InstructionInfo isa[32] = 
{
	InstructionInfo(0b00000, META_DECL { return isa_LOAD[instr.i.funct3]; }),
	InstructionInfo(0b00001, META_DECL { return isa_LOAD_FP[instr.i.funct3]; }),
	InstructionInfo(0b00010, IMPL_NONE),//custom-0
	InstructionInfo(0b00011, IMPL_NOTI),//MISC-MEM
	InstructionInfo(0b00100, META_DECL { return isa_OP_IMM[instr.i.funct3]; }),
	InstructionInfo(0b00101, "auipc", InstrType::IADD, Encoding::U, RegType::INT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.u.rd].u64 = unit->pc + u_imm(instr);
	}),
	InstructionInfo(0b00110, META_DECL { return isa_OP_IMM_32[instr.i.funct3]; }),
	InstructionInfo(0b00111, IMPL_NOTI),//48b
	InstructionInfo(0b01000, META_DECL { return isa_STORE[instr.s.funct3]; }),
	InstructionInfo(0b01001, META_DECL { return isa_STORE_FP[instr.r.funct3]; }),
	InstructionInfo(0b01010, IMPL_NONE),//custom-1
	InstructionInfo(0b01011, META_DECL { return isa_AMO[instr.r.funct3 & 0x1]; }),
	InstructionInfo(0b01100, META_DECL { return isa_OP[(instr.r.funct7 >> 4) & 0x2 | instr.r.funct7 & 0x1]; }),
	InstructionInfo(0b01101, "lui", InstrType::MOVE, Encoding::U, RegType::INT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.u.rd].u64 = u_imm(instr);
	}),
	InstructionInfo(0b01110, META_DECL { return isa_OP_32[instr.r.funct7 & 0b01 | instr.r.funct7 >> 4 & 0b10]; }),
	InstructionInfo(0b01111, IMPL_NOTI),//64b
	InstructionInfo(0b10000, "fmadd.s", InstrType::FFMAD, Encoding::R4,  RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r4.rd].f32 = (unit->float_regs->registers[instr.r4.rs1].f32 * unit->float_regs->registers[instr.r4.rs2].f32) + unit->float_regs->registers[instr.r4.rs3].f32;
	}),
	InstructionInfo(0b10001, "fmsub.s", InstrType::FFMAD, Encoding::R4,  RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r4.rd].f32 = (unit->float_regs->registers[instr.r4.rs1].f32 * unit->float_regs->registers[instr.r4.rs2].f32) - unit->float_regs->registers[instr.r4.rs3].f32;
	}),
	InstructionInfo(0b10010, "fnmsub.s", InstrType::FFMAD, Encoding::R4,  RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r4.rd].f32 = -(unit->float_regs->registers[instr.r4.rs1].f32 * unit->float_regs->registers[instr.r4.rs2].f32) + unit->float_regs->registers[instr.r4.rs3].f32;
	}),
	InstructionInfo(0b10011, "fnmadd.s", InstrType::FFMAD, Encoding::R4,  RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r4.rd].f32 = -(unit->float_regs->registers[instr.r4.rs1].f32 * unit->float_regs->registers[instr.r4.rs2].f32) - unit->float_regs->registers[instr.r4.rs3].f32;
	}),
	InstructionInfo(0b10100, META_DECL { return isa_OP_FP[instr.r.funct5]; }),
	InstructionInfo(0b10101, IMPL_NOTI),//reserved
	InstructionInfo(0b10110, IMPL_NONE),//custom-2/rv128
	InstructionInfo(0b10111, IMPL_NOTI),//48b
	InstructionInfo(0b11000, META_DECL{ return isa_BRANCH[instr.b.funct3]; }),
	InstructionInfo(0b11001, "jalr", InstrType::JUMP, Encoding::I,  RegType::INT, CTRL_FLOW_DECL
	{
		vaddr_t next_PC = unit->pc + 4;
		unit->pc = (unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr)) & ~0x1ull; //zeroing lowbit is explicitly defined in the spec
		unit->int_regs->registers[instr.i.rd].u64 = next_PC;
		return true;
	}),
	InstructionInfo(0b11010, IMPL_NOTI),//reserved
	InstructionInfo(0b11011, "jal", InstrType::JUMP, Encoding::J,  RegType::INT, CTRL_FLOW_DECL
	{
		unit->int_regs->registers[instr.j.rd].u64 = unit->pc + 4;
		unit->pc += j_imm(instr);
		return true;
	}),
	InstructionInfo(0b11100, META_DECL{ return isa_SYSTEM[instr.i.funct12]; }),
	InstructionInfo(0b11101, IMPL_NOTI),//reserved
	InstructionInfo(0b11110, IMPL_NONE),//custom-3/rv128
	InstructionInfo(0b11111, IMPL_NOTI),//>=80b
};

InstructionInfo const isa_SYSTEM[2] =
{
	InstructionInfo(0b000000000000, "ecall", InstrType::SYS, Encoding::I, RegType::INT, EXEC_DECL
	{
		//TODO this could be useful for things like printing to console
	}),
	InstructionInfo(0b000000000001, "ebreak", InstrType::SYS, Encoding::I, RegType::INT, EXEC_DECL
	{
		//break point
		__debugbreak();
	}),
};

InstructionInfo const isa_LOAD[7] =
{
	InstructionInfo(0b000, "lb", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<int8_t>(unit, instr);
	}),
	InstructionInfo(0b001, "lh", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<int16_t>(unit, instr);
	}),
	InstructionInfo(0b010, "lw", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<int32_t>(unit, instr);
	}),
	InstructionInfo(0b011, "ld", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<uint64_t>(unit, instr);
	}),
	InstructionInfo(0b100, "lbu", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<uint8_t>(unit, instr);
	}),
	InstructionInfo(0b101, "lhu", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<uint16_t>(unit, instr);
	}),
	InstructionInfo(0b110, "lwu", InstrType::LOAD, Encoding::I, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<uint32_t>(unit, instr);
	}),
};

InstructionInfo const isa_STORE[4] =
{
	InstructionInfo(0b000, "sb", InstrType::STORE, Encoding::S, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_store<uint8_t>(unit, instr);
	}),
	InstructionInfo(0b001, "sh", InstrType::STORE, Encoding::S, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_store<uint16_t>(unit, instr);
	}),
	InstructionInfo(0b010, "sw", InstrType::STORE, Encoding::S, RegType::INT, MEM_REQ_DECL
	{ 
		return _prepare_store<uint32_t>(unit, instr);
	}),
	InstructionInfo(0b011, "sd", InstrType::STORE, Encoding::S, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_store<uint64_t>(unit, instr);
	}),
};

InstructionInfo const isa_BRANCH[8] =
{
	InstructionInfo(0b000, "beq", InstrType::BRANCH, Encoding::B, RegType::INT, CTRL_FLOW_DECL
	{
		if(unit->int_regs->registers[instr.b.rs1].u64 == unit->int_regs->registers[instr.b.rs2].u64)
		{
			unit->pc += b_imm(instr);
			return true;
		}
		return false;
	}),
	InstructionInfo(0b001, "bne", InstrType::BRANCH, Encoding::B, RegType::INT, CTRL_FLOW_DECL
	{
		if(unit->int_regs->registers[instr.b.rs1].u64 != unit->int_regs->registers[instr.b.rs2].u64)
		{
			unit->pc += b_imm(instr);
			return true;
		}
		return false;
	}),
	InstructionInfo(0b010, IMPL_NONE),
	InstructionInfo(0b011, IMPL_NONE),
	InstructionInfo(0b100, "blt", InstrType::BRANCH, Encoding::B, RegType::INT, CTRL_FLOW_DECL
	{
		if(unit->int_regs->registers[instr.b.rs1].s64 < unit->int_regs->registers[instr.b.rs2].s64)
		{
			unit->pc += b_imm(instr);
			return true;
		}
		return false;
	}),
	InstructionInfo(0b101, "bge", InstrType::BRANCH, Encoding::B, RegType::INT, CTRL_FLOW_DECL
	{
		if(unit->int_regs->registers[instr.b.rs1].s64 >= unit->int_regs->registers[instr.b.rs2].s64)
		{
			unit->pc += b_imm(instr);
			return true;
		}
		return false;
	}),
	InstructionInfo(0b110, "bltu", InstrType::BRANCH, Encoding::B, RegType::INT, CTRL_FLOW_DECL
	{
		if(unit->int_regs->registers[instr.b.rs1].u64 < unit->int_regs->registers[instr.b.rs2].u64)
		{
			unit->pc += b_imm(instr);
			return true;
		}
		return false;
	}),
	InstructionInfo(0b111, "bgeu", InstrType::BRANCH, Encoding::B, RegType::INT, CTRL_FLOW_DECL
	{
		if(unit->int_regs->registers[instr.b.rs1].u64 >= unit->int_regs->registers[instr.b.rs2].u64)
		{
			unit->pc += b_imm(instr);
			return true;
		}
		return false;
	}),
};

InstructionInfo const isa_OP[3] = //(instr.r.funct7 >> 4) & 0x2 | instr.r.funct7 & 0x1
{
	InstructionInfo(0b000'0000,	META_DECL { return isa_OP_0x00[instr.r.funct3]; }),//OP-0x00
	InstructionInfo(0b000'0001,	META_DECL { return isa_OP_MULDIV[instr.r.funct3]; }),//OP-MULDIV
	InstructionInfo(0b010'0000,	META_DECL { return isa_OP_0x30[instr.r.funct3]; }),//OP-0x30
};

InstructionInfo const isa_OP_0x00[8] = //r.funct3
{
	InstructionInfo(0b000, "add", InstrType::IADD, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s64 + unit->int_regs->registers[instr.r.rs2].s64;
	}),
	InstructionInfo(0b001, "sll", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		//take lowest 5 bits of register[r2] and shift by that ammount
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 << (unit->int_regs->registers[instr.r.rs2].u8 & 0x3f);
	}),
	InstructionInfo(0b010, "slt", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].s64 < unit->int_regs->registers[instr.r.rs2].s64;
	}),
	InstructionInfo(0b011, "sltu", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 < unit->int_regs->registers[instr.r.rs2].u64;
	}),
	InstructionInfo(0b100, "xor", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 ^ unit->int_regs->registers[instr.r.rs2].u64;
	}),
	InstructionInfo(0b101, "srl", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 >> (unit->int_regs->registers[instr.r.rs2].u8 & 0x3f);
	}),
	InstructionInfo(0b110, "or", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 | unit->int_regs->registers[instr.r.rs2].u64;
	}),
	InstructionInfo(0b111, "and", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 & unit->int_regs->registers[instr.r.rs2].u64;
	}),
};

InstructionInfo const isa_OP_0x30[8] = //r.funct3
{
	InstructionInfo(0b000, "sub", InstrType::IADD, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s64 - unit->int_regs->registers[instr.r.rs2].s64;
	}),
	InstructionInfo(0b001, IMPL_NONE),
	InstructionInfo(0b010, IMPL_NONE),
	InstructionInfo(0b011, IMPL_NONE),
	InstructionInfo(0b100, IMPL_NONE),
	InstructionInfo(0b101, "sra", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s64 >> (unit->int_regs->registers[instr.r.rs2].u8 & 0b11'1111);
	}),
	InstructionInfo(0b110, IMPL_NONE),
	InstructionInfo(0b111, IMPL_NONE),
};

InstructionInfo const isa_OP_IMM[8] = //i.funct3
{
	InstructionInfo(0b000, "addi", InstrType::IADD, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.i.rd].s64 = unit->int_regs->registers[instr.i.rs1].s64 + i_imm(instr);
	}),
	InstructionInfo(0b001, "slli", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.i.rs1].u64 << instr.i.shamt6;
	}),
	InstructionInfo(0b010, "slti", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.i.rd].u64 = unit->int_regs->registers[instr.i.rs1].s64 < i_imm(instr);
	}),
	InstructionInfo(0b011, "sltui", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.i.rd].u64 = unit->int_regs->registers[instr.i.rs1].u64 < (uint64_t)i_imm(instr);
	}),
	InstructionInfo(0b100, "xori", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL{
		unit->int_regs->registers[instr.i.rd].u64 = unit->int_regs->registers[instr.i.rs1].u64 ^ i_imm(instr);
	}),
	InstructionInfo(0b101,	META_DECL { return isa_OP_IMM_SR[instr.i.imm_11_6 >> 4]; }),
	InstructionInfo(0b110, "ori", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.i.rd].u64 = unit->int_regs->registers[instr.i.rs1].u64 | i_imm(instr);
	}),
	InstructionInfo(0b111, "andi", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL{
		unit->int_regs->registers[instr.i.rd].u64 = unit->int_regs->registers[instr.i.rs1].u64 & i_imm(instr);
	}),
};

InstructionInfo const isa_OP_IMM_SR[2] = //i.imm_11_6 >> 4
{
	InstructionInfo(0b00'0000'101, "srli", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.i.rs1].u64 >> instr.i.shamt6;
	}),
	InstructionInfo(0b01'0000'101, "srai", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.i.rs1].s64 >> instr.i.shamt6;
	}),
};

InstructionInfo const isa_OP_32[3] =
{
	InstructionInfo(0b000'0000,	META_DECL { return isa_OP_32_0x00[instr.r.funct3]; }),//OP-32-0x00
	InstructionInfo(0b000'0001,	META_DECL { return isa_OP_32_MULDIV[instr.r.funct3]; }),//OP-32-MULDIV
	InstructionInfo(0b010'0000,	META_DECL { return isa_OP_32_0x30[instr.r.funct3]; }),//OP-32-0x30
};

InstructionInfo const isa_OP_32_0x00[8] = //r.funct3
{
	InstructionInfo(0b000, "addw", InstrType::IADD, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s32 + unit->int_regs->registers[instr.r.rs2].s32;
	}),
	InstructionInfo(0b001, "sllw", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u32 = unit->int_regs->registers[instr.r.rs1].u32 << (unit->int_regs->registers[instr.r.rs2].u8 & 0x1f);
	}),
	InstructionInfo(0b010, IMPL_NONE),
	InstructionInfo(0b011, IMPL_NONE),
	InstructionInfo(0b100, IMPL_NONE),
	InstructionInfo(0b101, "srlw", InstrType::ILOGICAL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u32 = unit->int_regs->registers[instr.r.rs1].u32 >> (unit->int_regs->registers[instr.r.rs2].u8 & 0x1f);
	}),
	InstructionInfo(0b110, IMPL_NONE),
	InstructionInfo(0b111, IMPL_NONE),
};

InstructionInfo const isa_OP_32_0x30[8] = //r.funct3
{
	InstructionInfo(0b000, "subw", InstrType::IADD, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s32 - unit->int_regs->registers[instr.r.rs2].s32;
	}),
	InstructionInfo(0b001, IMPL_NONE),
	InstructionInfo(0b010, IMPL_NONE),
	InstructionInfo(0b011, IMPL_NONE),
	InstructionInfo(0b100, IMPL_NONE),
	InstructionInfo(0b101, "sraw", InstrType::ILOGICAL, Encoding::R,  RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s32 = unit->int_regs->registers[instr.r.rs1].s32 >> (unit->int_regs->registers[instr.r.rs2].u8 & 0b1'1111);
	}),
	InstructionInfo(0b110, IMPL_NONE),
	InstructionInfo(0b111, IMPL_NONE),
};

InstructionInfo const isa_OP_IMM_32[8] = //i.funct3
{
	InstructionInfo(0b000, "addiw", InstrType::IADD, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.i.rd].s64 = unit->int_regs->registers[instr.i.rs1].s32 + (int32_t)i_imm(instr);
	}),
	InstructionInfo(0b001, "slliw", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.i.rd].u32 = unit->int_regs->registers[instr.i.rs1].u32 << instr.i.shamt5;
	}),
	InstructionInfo(0b010, IMPL_NONE),
	InstructionInfo(0b011, IMPL_NONE),
	InstructionInfo(0b100, IMPL_NONE),
	InstructionInfo(0b101, META_DECL { return isa_OP_IMM_32_SR[instr.i.imm_11_5 >> 5]; }), //OP-32-IMM-SR
	InstructionInfo(0b110, IMPL_NONE),
	InstructionInfo(0b111, IMPL_NONE),
};

InstructionInfo const isa_OP_IMM_32_SR[2] = //i.imm_11_5 >> 5
{
	InstructionInfo(0b00'0000'101, "srliw", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u32 = unit->int_regs->registers[instr.i.rs1].u32 >> instr.i.shamt5;
	}),
	InstructionInfo(0b01'0000'101, "sraiw", InstrType::ILOGICAL, Encoding::I, RegType::INT, EXEC_DECL
	{
			unit->int_regs->registers[instr.r.rd].s32 = unit->int_regs->registers[instr.i.rs1].s32 >> instr.i.shamt5;
	}),
};



//RV64M
InstructionInfo const isa_OP_MULDIV[8] = //r.funct3
{
	InstructionInfo(0b000, "mul", InstrType::IMUL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s64 * unit->int_regs->registers[instr.r.rs2].s64;
	}),
	InstructionInfo(0b001, IMPL_NOTI),//mulh
	InstructionInfo(0b010, IMPL_NOTI),//mulhsu
	InstructionInfo(0b011, IMPL_NOTI),//mulhu
	InstructionInfo(0b100, "div", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s64 / unit->int_regs->registers[instr.r.rs2].s64;
	}),
	InstructionInfo(0b101, "divu", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 / unit->int_regs->registers[instr.r.rs2].u64;
	}),
	InstructionInfo(0b110, "rem", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s64 % unit->int_regs->registers[instr.r.rs2].s64;
	}),
	InstructionInfo(0b111, "remu", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].u64 = unit->int_regs->registers[instr.r.rs1].u64 % unit->int_regs->registers[instr.r.rs2].u64;
	}),
};

InstructionInfo const isa_OP_32_MULDIV[8] = //r.funct3
{
	InstructionInfo(0b000, "mulw", InstrType::IMUL, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s32 * unit->int_regs->registers[instr.r.rs2].s32;
	}),
	InstructionInfo(0b001, IMPL_NONE),
	InstructionInfo(0b010, IMPL_NONE),
	InstructionInfo(0b011, IMPL_NONE),
	InstructionInfo(0b100, "divw", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s32 / unit->int_regs->registers[instr.r.rs2].s32;
	}),
	InstructionInfo(0b101, "divuw", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].u32 / unit->int_regs->registers[instr.r.rs2].u32;
	}),
	InstructionInfo(0b110, "remw", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].s32 % unit->int_regs->registers[instr.r.rs2].s32;
	}),
	InstructionInfo(0b111, "remuw", InstrType::IDIV, Encoding::R, RegType::INT, EXEC_DECL
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->int_regs->registers[instr.r.rs1].u32 % unit->int_regs->registers[instr.r.rs2].u32;
	}),
};



//RV64A
InstructionInfo const isa_AMO[2] = //r.funct3 & 0x1
{
	InstructionInfo(0b000,	META_DECL { return isa_AMO_32[instr.r.funct5 >> 2]; }),
	InstructionInfo(0b001,	META_DECL { return isa_AMO_64[instr.r.funct5 >> 2]; }),
};

InstructionInfo const isa_AMO_32[8] = //r.funct5 >> 2
{
	InstructionInfo(0b000, "amoadd.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int32_t, MemoryRequest::Type::AMO_ADD> (unit, instr);
	}),
	InstructionInfo(0b001, "amoxor.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int32_t, MemoryRequest::Type::AMO_XOR>(unit, instr);
	}),
	InstructionInfo(0b010, "amoor.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int32_t, MemoryRequest::Type::AMO_OR>(unit, instr);
	}),
	InstructionInfo(0b011, "amoand.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int32_t, MemoryRequest::Type::AMO_AND>(unit, instr);
	}),
	InstructionInfo(0b100, "amomin.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int32_t, MemoryRequest::Type::AMO_MIN>(unit, instr);
	}),
	InstructionInfo(0b101, "amomax.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int32_t, MemoryRequest::Type::AMO_MAX>(unit, instr);
	}),
	InstructionInfo(0b110, "amominu.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<uint32_t, MemoryRequest::Type::AMO_MINU>(unit, instr);
	}),
	InstructionInfo(0b111, "amomaxu.w", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<uint32_t, MemoryRequest::Type::AMO_MAXU>(unit, instr);
	}),
};

InstructionInfo const isa_AMO_64[8] = //r.funct5 >> 2
{
	InstructionInfo(0b000, "amoadd.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_ADD>(unit, instr);
	}),
	InstructionInfo(0b001, "amoxor.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_XOR>(unit, instr);
	}),
	InstructionInfo(0b010, "amoor.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_OR>(unit, instr);
	}),
	InstructionInfo(0b011, "amoand.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_AND>(unit, instr);
	}),
	InstructionInfo(0b100, "amomin.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_MIN>(unit, instr);
	}),
	InstructionInfo(0b101, "amomax.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_MAX>(unit, instr);
	}),
	InstructionInfo(0b110, "amominu.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_MINU>(unit, instr);
	}),
	InstructionInfo(0b111, "amomaxu.d", InstrType::ATOMIC, Encoding::R, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_amo<int64_t, MemoryRequest::Type::AMO_MAXU>(unit, instr);
	}),
};



//RV64F
InstructionInfo const isa_LOAD_FP[4] = //i.funct3
{
	InstructionInfo(0b001, IMPL_NOTI),//flb
	InstructionInfo(0b010, IMPL_NOTI),//flh
	InstructionInfo(0b010, "flw", InstrType::LOAD, Encoding::I, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_load<float>(unit, instr);
	}),
	InstructionInfo(0b011, IMPL_NOTI),//fld
};

InstructionInfo const isa_STORE_FP[4] = //r.funct3
{
	InstructionInfo(0b001, IMPL_NOTI),//fsb
	InstructionInfo(0b010, IMPL_NOTI),//fsh
	InstructionInfo(0b010, "fsw", InstrType::STORE, Encoding::S, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{
		return _prepare_store<float>(unit,instr);
	}),
	InstructionInfo(0b011, IMPL_NOTI),//fsd
};

InstructionInfo const isa_OP_FP[32] = //r.funct5
{
	InstructionInfo(0b000'00, "fadd.s", InstrType::FADD, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = unit->float_regs->registers[instr.r.rs1].f32 + unit->float_regs->registers[instr.r.rs2].f32;
	}),
	InstructionInfo(0b000'01, "fsub.s", InstrType::FADD, Encoding::R, RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r.rd].f32 = unit->float_regs->registers[instr.r.rs1].f32 - unit->float_regs->registers[instr.r.rs2].f32;
	}),
	InstructionInfo(0b000'10, "fmul.s", InstrType::FMUL, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = unit->float_regs->registers[instr.r.rs1].f32 * unit->float_regs->registers[instr.r.rs2].f32;
	}),
	InstructionInfo(0b000'11, "fdiv.s", InstrType::FDIV, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = unit->float_regs->registers[instr.r.rs1].f32 / unit->float_regs->registers[instr.r.rs2].f32;
	}),
	InstructionInfo(0b001'00, META_DECL {return isa_OP_FSGNJ_FP[instr.r.funct3]; }),//FSGNJ
	InstructionInfo(0b001'01, META_DECL {return isa_OP_0x14_FP[instr.r.funct3]; }),//0x14
	InstructionInfo(0b001'10, IMPL_NONE),
	InstructionInfo(0b001'11, IMPL_NONE),

	InstructionInfo(0b010'00, IMPL_NONE),
	InstructionInfo(0b010'01, IMPL_NONE),
	InstructionInfo(0b010'10, IMPL_NONE),
	InstructionInfo(0b010'11, "fsqrt.s", InstrType::FSQRT, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ps1(unit->float_regs->registers[instr.r.rs1].f32)));
	}),
	InstructionInfo(0b011'00, "fisqrt.s", InstrType::FSQRT, Encoding::R, RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r.rd].f32 = _mm_cvtss_f32(_mm_rsqrt_ps(_mm_set_ps1(unit->float_regs->registers[instr.r.rs1].f32)));
	}),
	InstructionInfo(0b011'01, "frcp.s", InstrType::FRCP, Encoding::R, RegType::FLOAT, EXEC_DECL
	{
		unit->float_regs->registers[instr.r.rd].f32 = _mm_cvtss_f32(_mm_rcp_ss(_mm_set_ps1(unit->float_regs->registers[instr.r.rs1].f32)));
	}),
	InstructionInfo(0b011'10, IMPL_NONE),
	InstructionInfo(0b011'11, IMPL_NONE),

	InstructionInfo(0b100'00, IMPL_NONE),
	InstructionInfo(0b100'01, IMPL_NONE),
	InstructionInfo(0b100'10, IMPL_NONE),
	InstructionInfo(0b100'11, IMPL_NONE),

	InstructionInfo(0b101'00, META_DECL {return isa_OP_FCMP_FP[instr.r.funct3]; }),//0x50
	InstructionInfo(0b101'01, IMPL_NONE),
	InstructionInfo(0b101'10, IMPL_NONE),
	InstructionInfo(0b101'11, IMPL_NONE),

	InstructionInfo(0b110'00, META_DECL {return isa_OP_0x60_FP[instr.r.rs2]; }),//0x60
	InstructionInfo(0b110'01, IMPL_NONE),
	InstructionInfo(0b110'10, META_DECL {return isa_OP_0x68_FP[instr.r.rs2]; }),//0x68
	InstructionInfo(0b110'11, IMPL_NONE),

	InstructionInfo(0b111'00, "fmv.x.w", InstrType::MOVE, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].s64 = unit->float_regs->registers[instr.r.rs1].s32;
	}),
	InstructionInfo(0b111'01, IMPL_NOTI),
	InstructionInfo(0b111'10, "fmv.w.x", InstrType::MOVE, Encoding::R, RegType::FLOAT, RegType::INT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].u32 = unit->int_regs->registers[instr.r.rs1].u32;
	}),
	InstructionInfo(0b1'1111, IMPL_NOTI),
};

InstructionInfo const isa_OP_FSGNJ_FP[3] = //r.funct3
{
	InstructionInfo(0b0'0000, "fsgnj.s", InstrType::FSIGN, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = copysignf(unit->float_regs->registers[instr.r.rs1].f32, unit->float_regs->registers[instr.r.rs2].f32);
	}),
	InstructionInfo(0b0'0001, "fsgnjn.s", InstrType::FSIGN, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = copysignf(unit->float_regs->registers[instr.r.rs1].f32, -unit->float_regs->registers[instr.r.rs2].f32);
	}),
	InstructionInfo(0b0'0010, "fsgnjx.s", InstrType::FSIGN, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		float rs1 = unit->float_regs->registers[instr.r.rs1].f32; 
		float rs2 = unit->float_regs->registers[instr.r.rs2].f32;
		unit->float_regs->registers[instr.r.rd].f32 = copysignf(rs1, (signbit(rs1) != signbit(rs2)) ? -1.0f : 1.0f);
	}),
};

InstructionInfo const isa_OP_0x14_FP[2] = //r.funct3
{
	InstructionInfo(0b0'0000, "fmin.s", InstrType::FMIN_MAX, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = std::min(unit->float_regs->registers[instr.r.rs1].f32, unit->float_regs->registers[instr.r.rs2].f32);
	}),
	InstructionInfo(0b0'0001, "fmax.s", InstrType::FMIN_MAX, Encoding::R, RegType::FLOAT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = std::max(unit->float_regs->registers[instr.r.rs1].f32, unit->float_regs->registers[instr.r.rs2].f32);
	}),
};

InstructionInfo const isa_OP_FCMP_FP[3] = //r.funct3
{
	InstructionInfo(0b0'0000, "fle.s", InstrType::FCMP, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].u64 = (unit->float_regs->registers[instr.r.rs1].f32 <= unit->float_regs->registers[instr.r.rs2].f32);
	}),
	InstructionInfo(0b0'0001, "flt.s", InstrType::FCMP, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].u64 = (unit->float_regs->registers[instr.r.rs1].f32 <  unit->float_regs->registers[instr.r.rs2].f32);
	}),
	InstructionInfo(0b0'0010, "feq.s", InstrType::FCMP, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].u64 = (unit->float_regs->registers[instr.r.rs1].f32 == unit->float_regs->registers[instr.r.rs2].f32);
	}),
};

InstructionInfo const isa_OP_0x60_FP[4] = //r.rs2
{
	InstructionInfo(0b0'0000, "fcvt.w.s", InstrType::CONVERT, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].s32 = static_cast<int32_t>(unit->float_regs->registers[instr.r.rs1].f32);
	}),
	InstructionInfo(0b0'0001, "fcvt.wu.s", InstrType::CONVERT, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].u32 = static_cast<uint32_t>(unit->float_regs->registers[instr.r.rs1].f32);
	}),
	InstructionInfo(0b0'0010, "fcvt.l.s", InstrType::CONVERT, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].s64 = static_cast<uint64_t>(unit->float_regs->registers[instr.r.rs1].f32);
	}),
	InstructionInfo(0b0'0011, "fcvt.lu.s", InstrType::CONVERT, Encoding::R, RegType::INT, RegType::FLOAT, EXEC_DECL 
	{
		unit->int_regs->registers[instr.r.rd].u64 = static_cast<uint64_t>(unit->float_regs->registers[instr.r.rs1].f32);
	}),
};

InstructionInfo const isa_OP_0x68_FP[4] = //r.rs2
{
	InstructionInfo(0b0'0000, "fcvt.s.w", InstrType::CONVERT, Encoding::R, RegType::FLOAT, RegType::INT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = static_cast<float>(unit->int_regs->registers[instr.r.rs1].s32);
	}),
	InstructionInfo(0b0'0001, "fcvt.s.wu", InstrType::CONVERT, Encoding::R, RegType::FLOAT, RegType::INT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = static_cast<float>(unit->int_regs->registers[instr.r.rs1].u32);
	}),
	InstructionInfo(0b0'0010, "fcvt.s.l", InstrType::CONVERT, Encoding::R, RegType::FLOAT, RegType::INT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = static_cast<float>(unit->int_regs->registers[instr.r.rs1].s64);
	}),
	InstructionInfo(0b0'0011, "fcvt.s.lu", InstrType::CONVERT, Encoding::R, RegType::FLOAT, RegType::INT, EXEC_DECL 
	{
		unit->float_regs->registers[instr.r.rd].f32 = static_cast<float>(unit->int_regs->registers[instr.r.rs1].u64);
	}),
};

}}}
