#pragma once

#include "stdafx.hpp"

#include "registers.hpp"

namespace Arches { namespace ISA { namespace RISCV {

	struct ExecutionItem
	{
		vaddr_t                    pc;
		IntegerRegisterFile*       int_regs{nullptr};
		FloatingPointRegisterFile* float_regs{nullptr};
	};
}}}
