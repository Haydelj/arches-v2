#pragma once
#include "stdafx.hpp"

inline uint log2i(uint64_t in)
{
	uint i = 0;
	while(in >>= 1) ++i;
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

inline uint ctz(uint64_t mask)
{
	return _tzcnt_u64(mask);
}

inline uint popcnt(uint64_t mask)
{
	return __popcnt64(mask);
}

inline uint64_t rotr(uint64_t mask, uint n)
{
	return _rotr64(mask, n);
}

inline uint64_t pdep(uint64_t data, uint64_t mask)
{
	return _pdep_u64(data, mask);
}

inline uint64_t pext(uint64_t data, uint64_t mask)
{
	return _pext_u64(data, mask);
}

class alignas(16) uint128_t 
{
public:
	uint64_t hi;
	uint64_t lo;

public:
	uint128_t() = default;
	uint128_t(uint64_t lo) : hi(0), lo(lo) {}
	uint128_t(uint64_t hi, uint64_t lo) : hi(hi), lo(lo) {}
	uint128_t(const uint128_t& other) : hi(other.hi), lo(other.lo) {}

	uint128_t& operator=(const uint128_t& other)
	{
		lo = other.lo;
		hi = other.hi;
		return *this;
	}

	uint128_t& operator|=(const uint128_t& other)
	{
		lo |= other.lo;
		hi |= other.hi;
		return *this;
	}

	uint128_t& operator&=(const uint128_t& other)
	{
		lo &= other.lo;
		hi &= other.hi;
		return *this;
	}

	uint128_t operator<<(uint n)
	{
		if(n == 0)
		{
			return *this;
		}
		else if(n < 64)
		{
			return {(hi << n) | (lo >> (64 - n)), lo << n};
		}
		else if(n == 64)
		{
			return {lo, 0};
		}
		else
		{
			n -= 64;
			return {lo << n, 0};
		}
	}

	uint128_t operator~()
	{
		return {~hi, ~lo};
	}

	bool operator==(const uint128_t& other)
	{
		return hi == other.hi && lo == other.lo;
	}

	bool operator!()
	{
		return hi == 0 && lo == 0;
	}
};

inline uint ctz(uint128_t mask)
{
	uint lo_tz = ctz(mask.lo);
	if(lo_tz < 64) return lo_tz;
	return ctz(mask.hi) + 64;
}

inline uint popcnt(uint128_t mask)
{
	return popcnt(mask.lo) + popcnt(mask.hi);
}

inline uint128_t rotr(uint128_t mask, uint n)
{
	n = n % 128;
	if(n == 0)
	{
		return {mask.hi, mask.lo};
	}
	else if(n < 64)
	{
		return {(mask.hi >> n) | (mask.lo << (64 - n)), (mask.lo >> n) | (mask.hi << (64 - n))};
	}
	else if(n == 64)
	{
		return {mask.lo, mask.hi};
	}
	else
	{
		n -= 64;
		return {(mask.lo >> n) | (mask.hi << (64 - n)), (mask.hi >> n) | (mask.lo << (64 - n))};
	}
}



