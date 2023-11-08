#pragma once
#include "stdafx.hpp"

inline uint log2i(uint64_t in)
{
	uint i = 0;
	while (in >>= 1) ++i;
	return i;
}

inline uint64_t generate_nbit_mask(uint n)
{
	if(n >= 64) return ~0ull;
	return ~(~0ull << n);
}


inline Arches::paddr_t align_to(size_t alignment, Arches::paddr_t paddr)
{
	return (paddr + alignment - 1) & ~(alignment - 1);
}

inline uint64_t rotr(uint64_t mask, uint n)
{
	return _rotr64(mask, n);
}

inline uint64_t ctz(uint64_t mask)
{
	return _tzcnt_u64(mask);
}

inline uint64_t popcnt(uint64_t mask)
{
	return __popcnt64(mask);
}

inline uint64_t pdep(uint64_t data, uint64_t mask)
{
	return _pdep_u64(data, mask);
}

inline uint64_t pext(uint64_t data, uint64_t mask)
{
	return _pext_u64(data, mask);
}

