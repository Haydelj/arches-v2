#pragma once
#include "stdafx.hpp"

#include "ray-tracing-include.hpp"

#ifdef ARCH_X86
static std::atomic_uint32_t _next_thread;
#endif

inline uint32_t fchthrd()
{
	#ifdef ARCH_RISCV
	uint32_t value;
	asm volatile("traxamoin %[rd]\n\t" : [rd] "=r" (value));
	return value;
	#endif

	#ifdef ARCH_X86
	return _next_thread++;
	#endif

	return 0;
}

#ifdef ARCH_X86
void reset_atomicinc()
{
 	_next_thread = 0;
}
#endif
