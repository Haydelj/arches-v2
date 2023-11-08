#pragma once

#include "stdafx.hpp"
#include "simulator/transactions.hpp"
#include "execution-base.hpp"

namespace Arches { namespace ISA { namespace RISCV {

class ExecutionItem;

enum : uint32_t
{
	CUSTOM_OPCODE0 = 0b00010,
	CUSTOM_OPCODE1 = 0b01010,
	CUSTOM_OPCODE2 = 0b10110,
	CUSTOM_OPCODE3 = 0b11110,
};

constexpr size_t NUM_OPCODES = 32;

const std::string instr_opcode_names[NUM_OPCODES] =
{
	"LOAD",
	"LOAD-FP",
	"custom-0",
	"MISC-MEM",
	"OP-IMM",
	"AUIPC",
	"OP-IMM-32",
	"48b",
	"STORE",
	"STORE-FP",
	"custom-1",
	"AMO",
	"OP",
	"LUI",
	"OP-32",
	"64b",
	"FMADD",
	"FMSUB",
	"FNMSUB",
	"FNMADD",
	"OP-FP",
	"reserved",
	"custom-2/rv128",
	"48b",
	"BRANCH",
	"JALR",
	"reserved",
	"JAL",
	"ECALL",
	"reserved",
	"custom-3/rv128"
	">=80b"
};

enum class InstrType : uint8_t
{
	NA,

	LOAD,
	STORE,
	ATOMIC,
	PREFETCH,

	SYS,

	JUMP,
	BRANCH,

	MOVE,
	CONVERT,

	ILOGICAL,
	IADD,
	IMUL,
	IDIV,

	FCMP,
	FSIGN,
	FMIN_MAX,
	FADD,
	FMUL,
	FFMAD,
	FDIV,
	FSQRT,
	FRCP,

	CUSTOM0,
	CUSTOM1,
	CUSTOM2,
	CUSTOM3,
	CUSTOM4,
	CUSTOM5,
	CUSTOM6,
	CUSTOM7,

	NUM_TYPES,
};



class InstructionTypeNameDatabase
{
public:
	InstructionTypeNameDatabase(const InstructionTypeNameDatabase& other) = delete;

	static InstructionTypeNameDatabase& get_instance()
	{
		if(!InstructionTypeNameDatabase::_instance)
			InstructionTypeNameDatabase::_instance = new InstructionTypeNameDatabase();
		
		return *InstructionTypeNameDatabase::_instance;
	}

	std::string& operator[](const InstrType type)
	{
		return _instr_type_names[(uint)type];
	}

private:
	static InstructionTypeNameDatabase* _instance;

	std::string _instr_type_names[(uint)InstrType::NUM_TYPES]
	{
		"NA",

		"LOAD",
		"STORE",
		"ATOMIC",
		"PREFETCH",

		"SYS",

		"JUMP",
		"BRANCH",

		"MOVE",
		"CONVERT",

		"ILOGICAL",
		"IADD",
		"IMUL",
		"IDIV",

		"FCMP",
		"FSIGN",
		"FMIN_MAX",
		"FADD",
		"FMUL",
		"FFMAD",
		"FDIV",
		"FSQRT",
		"FRCP",

		"CUSTOM0",
		"CUSTOM1",
		"CUSTOM2",
		"CUSTOM3",
		"CUSTOM4",
		"CUSTOM5",
		"CUSTOM6",
		"CUSTOM7",
	};

	InstructionTypeNameDatabase() = default;
};

enum class ExecType : uint8_t
{
	INVALID,
	META,
	EXECUTABLE,
	CONTROL_FLOW,
	MEMORY,
};

enum class Encoding : uint8_t
{
	NA,
	R,
	R4,
	I,
	S,
	B,
	U,
	J,
	C,
};

class InstructionInfo;

class Instruction final 
{
public:
	union {
		//common instruction bit range names
		struct 
		{
			uint32_t opcode  : 7;
			uint32_t rd      : 5;
			uint32_t funct3  : 3;
			uint32_t rs1     : 5;
			uint32_t rs2     : 5;
			uint32_t funct2  : 2;
			uint32_t rs3     : 5;
		};

		union
		{
			struct 
			{
				uint32_t        : 7;
				uint32_t rd     : 5;
				uint32_t funct3 : 3;
				uint32_t rs1    : 5;
				uint32_t rs2    : 5;
				uint32_t funct7 : 7;
			};
			struct
			{
				uint32_t        : 12;
				uint32_t rm     : 3;
				uint32_t        : 10;
				uint32_t fmt    : 2;
				uint32_t funct5 : 5;
			};
			struct
			{
				uint32_t    : 25;
				uint32_t rl : 1;
				uint32_t aq : 1;
				uint32_t    : 5;
			};
		}r;
	
		struct 
		{
			uint32_t        : 7;
			uint32_t rd	    : 5;
			uint32_t funct3 : 3;
			uint32_t rs1    : 5;
			uint32_t rs2    : 5;
			uint32_t funct2 : 2;
			uint32_t rs3    : 5;
		}r4;
	
		union 
		{
			struct 
			{
				uint32_t          : 7;
				uint32_t rd       : 5;
				uint32_t funct3   : 3;
				uint32_t rs1      : 5;
				uint32_t imm_11_0 : 12;
			};
			struct 
			{	
				uint32_t          : 20;
				uint32_t shamt5   : 5;
				uint32_t imm_11_5 : 7;
			};
			struct 
			{
				uint32_t          : 20;
				uint32_t shamt6   : 6;
				uint32_t imm_11_6 : 6;
			};
			struct 
			{
				uint32_t         : 20;
				uint32_t funct12 : 12;
			};
		}i;
	
		struct 
		{
			uint32_t            : 7;
			uint32_t imm_4_0	: 5;
			uint32_t funct3		: 3;
			uint32_t rs1		: 5;
			uint32_t rs2		: 5;
			uint32_t imm_11_5	: 7;
	
		}s;
	
		struct 
		{
			uint32_t            : 7;
			uint32_t imm_11     : 1;
			uint32_t imm_4_1	: 4;
			uint32_t funct3		: 3;
			uint32_t rs1		: 5;
			uint32_t rs2		: 5;
			uint32_t imm_10_5	: 6;
			uint32_t imm_12     : 1;
		}b;
	
		struct 
		{
			uint32_t            : 7;
			uint32_t rd			: 5;
			uint32_t imm_31_12  : 20;
		}u;
	
		struct 
		{
			uint32_t            : 7;
			uint32_t rd			: 5;
			uint32_t imm_19_12	: 8;
			uint32_t imm_11		: 1;
			uint32_t imm_10_1	: 10;
			uint32_t imm_20		: 1;
		}j;

		uint32_t data;
	};
	
	Instruction(uint32_t data) : data(data) {}
	
	//use this to get the info for a given instruction
	const InstructionInfo get_info() const;
};

int64_t i_imm(Instruction instr);
int64_t s_imm(Instruction instr);
int64_t b_imm(Instruction instr);
int64_t u_imm(Instruction instr);
int64_t j_imm(Instruction instr);

class InstructionInfo final 
{
public:
	typedef InstructionInfo const& (*ResolveFunction)(Instruction const&);
	typedef void (*ExecutionFunction)(Instruction const&, ExecutionItem*);
	typedef bool (*ControlFlowFunction)(Instruction const&, ExecutionItem*);
	typedef MemoryRequest (*MemoryRequestFunction)(Instruction const&, ExecutionItem*);

	char const* mnemonic{nullptr};
	InstrType   instr_type{InstrType::NA};
	Encoding    encoding{Encoding::NA};
	RegType     dst_reg_type{RegType::INT};
	RegType     src_reg_type{RegType::INT};
	ExecType    exec_type{ExecType::INVALID};

private:
	union
	{
		ResolveFunction _resolve_fn;
		ExecutionFunction _exec_fn;
		ControlFlowFunction _ctrl_fn;
		MemoryRequestFunction _req_fn;
	};

public:
	InstructionInfo() = default;

	InstructionInfo(uint32_t func_code, ResolveFunction resolve_fn) : _resolve_fn(resolve_fn), exec_type(ExecType::META) {}

	InstructionInfo(uint32_t func_code, ExecutionFunction impl_fn) : _exec_fn(impl_fn) {}
	InstructionInfo(uint32_t func_code, char const* mnemonic, ExecutionFunction impl_fn) : mnemonic(mnemonic), _exec_fn(impl_fn) {}

	InstructionInfo(uint32_t func_code, char const* mnemonic, InstrType instr_type, Encoding encoding, RegType reg_type, ExecutionFunction exec_fn) : mnemonic(mnemonic), instr_type(instr_type), encoding(encoding), dst_reg_type(reg_type), src_reg_type(reg_type), exec_type(ExecType::EXECUTABLE), _exec_fn(exec_fn) {}
	InstructionInfo(uint32_t func_code, char const* mnemonic, InstrType instr_type, Encoding encoding, RegType dst_reg_type, RegType src_reg_type, ExecutionFunction exec_fn) : mnemonic(mnemonic), instr_type(instr_type), encoding(encoding), dst_reg_type(dst_reg_type), src_reg_type(src_reg_type), exec_type(ExecType::EXECUTABLE), _exec_fn(exec_fn) {}
	
	InstructionInfo(uint32_t func_code, char const* mnemonic, InstrType instr_type, Encoding encoding, RegType reg_type, ControlFlowFunction ctrl_fn) : mnemonic(mnemonic), instr_type(instr_type), encoding(encoding), dst_reg_type(reg_type), src_reg_type(reg_type), exec_type(ExecType::CONTROL_FLOW), _ctrl_fn(ctrl_fn) {}

	InstructionInfo(uint32_t func_code, char const* mnemonic, InstrType instr_type, Encoding encoding, RegType reg_type, MemoryRequestFunction req_fn) : mnemonic(mnemonic), instr_type(instr_type), encoding(encoding), dst_reg_type(reg_type), src_reg_type(reg_type), exec_type(ExecType::MEMORY), _req_fn(req_fn) {}
	InstructionInfo(uint32_t func_code, char const* mnemonic, InstrType instr_type, Encoding encoding, RegType dst_reg_type, RegType src_reg_type, MemoryRequestFunction req_fn) : mnemonic(mnemonic), instr_type(instr_type), encoding(encoding), dst_reg_type(dst_reg_type), src_reg_type(src_reg_type), exec_type(ExecType::MEMORY), _req_fn(req_fn) {}


	~InstructionInfo() = default;

	const InstructionInfo resolve(const Instruction& instr) const
	{
		if (exec_type != ExecType::META) return *this;
		else                             return _resolve_fn(instr).resolve(instr);
	}

	void execute(ExecutionItem& unit, const Instruction& instr) const
	{ 
		assert(exec_type == ExecType::EXECUTABLE);
		_exec_fn(instr, &unit); 
	}

	bool execute_branch(ExecutionItem& unit, const Instruction& instr) const
	{
		assert(exec_type == ExecType::CONTROL_FLOW);
		return _ctrl_fn(instr, &unit);
	}

	MemoryRequest generate_request(ExecutionItem& unit, const Instruction& instr) const
	{
		assert(exec_type == ExecType::MEMORY);
		return _req_fn(instr, &unit);
	}

	void print_instr(Instruction const& instr, FILE* stream = stdout) const
	{
		char drfc = dst_reg_type == RegType::INT ? 'x' : 'f';
		char srfc = src_reg_type == RegType::INT ? 'x' : 'f';

		switch(encoding)
		{
		case ISA::RISCV::Encoding::R:
			fprintf(stream, "%s\t%c%d,%c%d,%c%d", mnemonic, drfc, instr.rd, srfc, instr.rs1, srfc, instr.rs2);
			break;

		case ISA::RISCV::Encoding::R4:
			fprintf(stream, "%s\t%c%d,%c%d,%c%d,%c%d", mnemonic, drfc, instr.rd, srfc, instr.rs1, srfc, instr.rs2, srfc, instr.rs3);
			break;

		case ISA::RISCV::Encoding::I:
			fprintf(stream, "%s\t%c%d,%c%d,%d", mnemonic, drfc, instr.rd, srfc, instr.rs1, (int32_t)i_imm(instr));
			break;

		case ISA::RISCV::Encoding::S:
			fprintf(stream, "%s\t%c%d,%c%d,%d", mnemonic, drfc, instr.rs2, srfc, instr.rs1, (int32_t)s_imm(instr));
			break;

		case ISA::RISCV::Encoding::B:
			fprintf(stream, "%s\t%c%d,%c%d,%d", mnemonic, srfc, instr.rs1, srfc, instr.rs2, (int32_t)b_imm(instr));
			break;

		case ISA::RISCV::Encoding::U:
			fprintf(stream, "%s\t%c%d,%d", mnemonic, drfc, instr.rd, (int32_t)u_imm(instr));
			break;

		case ISA::RISCV::Encoding::J:
			fprintf(stream, "%s\t%c%d,%d", mnemonic, drfc, instr.rd, (int32_t)j_imm(instr));
			break;

		default:
			fprintf(stream, "%s", mnemonic);
			break;
		}
	}

	void print_regs(const Instruction& instr, const ExecutionItem& exec_item) const
	{
		if(src_reg_type != RegType::INT) return;
		if(dst_reg_type != RegType::INT) return;

		switch(encoding)
		{
		case ISA::RISCV::Encoding::R:
			printf("\t%lld,%lld,%lld", exec_item.int_regs->registers[instr.r.rd].u64, exec_item.int_regs->registers[instr.r.rs1].u64, exec_item.int_regs->registers[instr.r.rs2].u64);
			break;

		case ISA::RISCV::Encoding::R4:
			printf("\t%lld,%lld,%lld,%lld", exec_item.int_regs->registers[instr.r4.rd].u64, exec_item.int_regs->registers[instr.r4.rs1].u64, exec_item.int_regs->registers[instr.r4.rs2].u64, exec_item.int_regs->registers[instr.r4.rs3].u64);
			break;

		case ISA::RISCV::Encoding::I:
			printf("\t%lld,%lld", exec_item.int_regs->registers[instr.i.rd].u64, exec_item.int_regs->registers[instr.i.rs1].u64);
			break;

		case ISA::RISCV::Encoding::S:
			printf("\t%lld,%lld", exec_item.int_regs->registers[instr.s.rs2].u64, exec_item.int_regs->registers[instr.s.rs1].u64);
			break;

		case ISA::RISCV::Encoding::B:
			printf("\t%lld,%lld", exec_item.int_regs->registers[instr.b.rs1].u64, exec_item.int_regs->registers[instr.b.rs2].u64);
			break;

		case ISA::RISCV::Encoding::U:
			printf("\t%lld", exec_item.int_regs->registers[instr.u.rd].u64);
			break;

		case ISA::RISCV::Encoding::J:
			printf("\t%lld", exec_item.int_regs->registers[instr.j.rd].u64);
			break;

		default:
			printf("\t");
			break;
		}
	}
};


//RV64C
//extern InstructionInfo const compressed_isa[4]; //TODO this

//RV64I
extern InstructionInfo isa[32];

extern InstructionInfo const isa_SYSTEM[2];

extern InstructionInfo const isa_LOAD[7];
extern InstructionInfo const isa_STORE[4];

extern InstructionInfo const isa_BRANCH[8];

extern InstructionInfo const isa_OP[3];
extern InstructionInfo const isa_OP_0x00[8];
extern InstructionInfo const isa_OP_0x30[8];

extern InstructionInfo const isa_OP_IMM[8];
extern InstructionInfo const isa_OP_IMM_SR[2];

extern InstructionInfo const isa_OP_32[3];
extern InstructionInfo const isa_OP_32_0x00[8];
extern InstructionInfo const isa_OP_32_0x30[8];

extern InstructionInfo const isa_OP_IMM_32[8];
extern InstructionInfo const isa_OP_IMM_32_SR[2];

//RV64M
extern InstructionInfo const isa_OP_MULDIV[8];
extern InstructionInfo const isa_OP_32_MULDIV[8];

//RV64A
extern InstructionInfo const isa_AMO[2];
extern InstructionInfo const isa_AMO_32[8];
extern InstructionInfo const isa_AMO_64[8];

//RV64F
extern InstructionInfo const isa_LOAD_FP[4];
extern InstructionInfo const isa_STORE_FP[4];
extern InstructionInfo const isa_OP_FP[32];

extern InstructionInfo const isa_OP_FSGNJ_FP[3];
extern InstructionInfo const isa_OP_0x14_FP[2];
extern InstructionInfo const isa_OP_FCMP_FP[3];
extern InstructionInfo const isa_OP_0x60_FP[4];
extern InstructionInfo const isa_OP_0x68_FP[4];

#define META_DECL [](Instruction const& instr) -> InstructionInfo const&
#define EXEC_DECL [](Instruction const& instr, ExecutionItem* unit) -> void
#define CTRL_FLOW_DECL [](Instruction const& instr, ExecutionItem* unit) -> bool
#define MEM_REQ_DECL [](Instruction const& instr, ExecutionItem* unit) -> MemoryRequest

}}}
