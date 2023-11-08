#pragma once

#include "stdafx.hpp"


namespace Arches { namespace ISA { namespace RISCV {

enum class RegType : uint8_t
{
	INT,
	FLOAT,
};

struct RegAddr
{
	union
	{
		struct
		{
			uint8_t reg : 5;
			RegType reg_type : 1;
			bool    sign_ext : 1;
			uint8_t _reserved : 1;
		};

		uint8_t u8;
	};

	RegAddr() = default;
	RegAddr(uint8_t u8) : u8(u8) {}

	std::string mnemonic()
	{
		return (reg_type == RegType::INT ? "x" : "f") + std::to_string(reg);
	}
};

class Register32 final {
public:
	union {
		uint8_t  u8;  int8_t  s8;
		uint16_t u16; int16_t s16;
		uint32_t u32; int32_t s32;
		float f32;
	};
};

class Register64 final {
public:
	union {
		uint8_t  u8;  int8_t  s8;
		uint16_t u16; int16_t s16;
		uint32_t u32; int32_t s32;
		uint64_t u64; uint64_t s64;
		float f32; double f64;
	};
};

class FloatingPointControlStatusRegister final {
public:
	union {
		uint32_t data;
		struct {
			uint32_t NX		: 1;
			uint32_t UF		: 1;
			uint32_t OF		: 1;
			uint32_t DZ		: 1;
			uint32_t NV		: 1;
			uint32_t frm	: 3;
			uint32_t		: 24;
		};
	};
};

class IntegerRegisterFile final 
{
	public:
		//See table of conventional uses for these registers here:
		//	http://www.cs.uwm.edu/classes/cs315/Bacon/Lecture/HTML/ch05s03.html
		union 
		{
			Register64 registers[32];
			struct
			{
				Register64 zero; //0
				Register64 ra; //1
				Register64 sp; //2
				Register64 gp; //3
				Register64 tp; //4 
				Register64 t0; //5
				Register64 t1; //6
				Register64 t2; //7
				Register64 s0; //8
				Register64 s1; //9
				Register64 a0; //10
				Register64 a1; //11
				Register64 a2; //12
				Register64 a3; //13
				Register64 a4; //14
				Register64 a5; //15
				Register64 a6; //16
				Register64 a7; //17
				Register64 s2; //18
				Register64 s3; //19
				Register64 s4; //20
				Register64 s5; //21
				Register64 s6; //22
				Register64 s7; //23
				Register64 s8; //24
				Register64 s9; //25
				Register64 s10; //26
				Register64 s11; //27
				Register64 t3; //28
				Register64 t4; //29 
				Register64 t5; //30
				Register64 t6; //31
			};
			struct
			{
				Register64 _[8];
				Register64 fp;
			};
			struct 
			{
				Register64  x0;
				Register64  x1; 
				Register64  x2; 
				Register64  x3; 
				Register64  x4; 
				Register64  x5; 
				Register64  x6; 
				Register64  x7; 
				Register64  x8;   
				Register64  x9; 
				Register64 x10; 
				Register64 x11; 
				Register64 x12; 
				Register64 x13; 
				Register64 x14; 
				Register64 x15; 
				Register64 x16; 
				Register64 x17; 
				Register64 x18; 
				Register64 x19; 
				Register64 x20; 
				Register64 x21; 
				Register64 x22; 
				Register64 x23; 
				Register64 x24; 
				Register64 x25; 
				Register64 x26; ;
				Register64 x27; ;
				Register64 x28; 
				Register64 x29; 
				Register64 x30; 
				Register64 x31; 
			};
		};
		bool valid[32];

	public:
		IntegerRegisterFile();
		~IntegerRegisterFile() = default;
		void print()
		{
			for (uint i = 0; i < 32u; ++i)
				printf("Register%d: %lld\n", i, registers[i].u64);
		}
};

class FloatingPointRegisterFile final {
public:
	union {
		Register32 registers[32];
		struct
		{
			Register32 ft0; //0
			Register32 ft1; //1
			Register32 ft2; //2
			Register32 ft3; //3
			Register32 ft4; //4
			Register32 ft5; //5
			Register32 ft6; //6
			Register32 ft7; //7
			Register32 fs0; //8
			Register32 fs1; //9
			Register32 fa0; //10
			Register32 fa1; //11
			Register32 fa2; //12
			Register32 fa3; //13
			Register32 fa4; //14
			Register32 fa5; //15
			Register32 fa6; //16
			Register32 fa7; //17
			Register32 fs2; //18
			Register32 fs3; //19
			Register32 fs4; //20
			Register32 fs5; //21
			Register32 fs6; //22
			Register32 fs7; //23
			Register32 fs8; //24
			Register32 fs9; //25
			Register32 fs10; //26
			Register32 fs11; //27
			Register32 ft8; //28
			Register32 ft9; //29
			Register32 ft10; //30
			Register32 ft11; //31
		};
		struct 
		{
			Register32  f0;
			Register32  f1;
			Register32  f2;
			Register32  f3;
			Register32  f4;
			Register32  f5;
			Register32  f6;
			Register32  f7;
			Register32  f8;
			Register32  f9;
			Register32 f10;
			Register32 f11;
			Register32 f12;
			Register32 f13;
			Register32 f14;
			Register32 f15;
			Register32 f16;
			Register32 f17;
			Register32 f18;
			Register32 f19;
			Register32 f20;
			Register32 f21;
			Register32 f22;
			Register32 f23;
			Register32 f24;
			Register32 f25;
			Register32 f26;
			Register32 f27;
			Register32 f28;
			Register32 f29;
			Register32 f30;
			Register32 f31;
		};
	};
	bool valid[32];

	FloatingPointControlStatusRegister fcsr;

public:
	FloatingPointRegisterFile();
	~FloatingPointRegisterFile() = default;
};

void write_register(IntegerRegisterFile* int_regs, FloatingPointRegisterFile* float_regs, RegAddr dst, uint8_t size, const uint8_t* data);

}}}
