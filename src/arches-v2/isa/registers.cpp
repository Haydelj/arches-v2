#include "registers.hpp"


namespace Arches { namespace ISA { namespace RISCV {

IntegerRegisterFile::IntegerRegisterFile() 
{
	//Zero register
	registers[0].u64 = 0ull;

	//Stack and frame pointers
	sp.u64 = 0ull;
	fp.u64 = 0ull;

	for (int i = 0; i < sizeof(valid); ++i) valid[i] = true;
}

FloatingPointRegisterFile::FloatingPointRegisterFile() 
{
	fcsr.data = 0u;
	//Make rounding mode match simulator rounding mode
	switch ((_mm_getcsr() >> 13) & 0b11) {
	case 0b00: //nearest (even)
		fcsr.frm = 0b000;
		break;
	case 0b01: //toward minus infinity
		fcsr.frm = 0b010;
		break;
	case 0b10: //toward infinity
		fcsr.frm = 0b011;
		break;
	case 0b11: //toward zero
		fcsr.frm = 0b001;
		break;
	}

	for (int i = 0; i < sizeof(valid); ++i) valid[i] = true;
}

void write_register(IntegerRegisterFile* int_regs, FloatingPointRegisterFile* float_regs, DstReg dst_reg, const uint8_t* data)
{
	if(is_int(dst_reg.type))
	{
		if(!sign_ext(dst_reg.type))
		{
			switch(size(dst_reg.type))
			{
			case 1: int_regs->registers[dst_reg.index].u64 = *((uint8_t*)data); break;
			case 2: int_regs->registers[dst_reg.index].u64 = *((uint16_t*)data); break;
			case 4: int_regs->registers[dst_reg.index].u64 = *((uint32_t*)data); break;
			case 8: int_regs->registers[dst_reg.index].u64 = *((uint64_t*)data); break;
			default: _assert(false);
			}
		}
		else
		{
			switch(size(dst_reg.type))
			{
			case 1: int_regs->registers[dst_reg.index].s64 = *((int8_t*)data); break;
			case 2: int_regs->registers[dst_reg.index].s64 = *((int16_t*)data); break;
			case 4: int_regs->registers[dst_reg.index].s64 = *((int32_t*)data); break;
			case 8: int_regs->registers[dst_reg.index].s64 = *((uint64_t*)data); break;
			default: _assert(false);
			}
		}
	}
	else
	{
		switch(size(dst_reg.type))
		{
		case 4: float_regs->registers[dst_reg.index].f32 = *((float*)data); break;
		default: _assert(false);
		}
	}
}

}}}
