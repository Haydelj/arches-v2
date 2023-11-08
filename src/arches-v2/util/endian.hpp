#pragma once

#include "stdafx.hpp"


namespace Arches { namespace Util { namespace Endian {


//Reverse the endianness of the argument, whatever it may be.
//	http://stackoverflow.com/questions/2182002/convert-big-endian-to-little-endian-in-c-without-using-provided-func
inline uint16_t reverse(uint16_t val) {
	uint32_t temp = static_cast<uint32_t>(val);
	return static_cast<uint16_t>( (temp << 8u) | (temp >> 8u) );
}
inline  int16_t reverse( int16_t val) {
	uint16_t swapped = reverse(reinterpret_cast<uint16_t&>(val));
	return reinterpret_cast<int16_t&>(swapped);
}
inline uint32_t reverse(uint32_t val) {
	val = ((val<<8u)&0xFF00FF00u) | ((val>>8u)&0x00FF00FFu);
	return (val<<16u) | (val>>16u);
}
inline  int32_t reverse( int32_t val) {
	uint32_t swapped = reverse(reinterpret_cast<uint32_t&>(val));
	return reinterpret_cast<int32_t&>(swapped);
}
inline uint64_t reverse(uint64_t val) {
	val = ((val<< 8ull)&0xFF00FF00FF00FF00ull) | ((val>> 8ull)&0x00FF00FF00FF00FFull);
	val = ((val<<16ull)&0xFFFF0000FFFF0000ull) | ((val>>16ull)&0x0000FFFF0000FFFFull);
	return (val<<32ull) | (val >> 32ull);
}
inline  int64_t reverse( int64_t val) {
	uint64_t swapped = reverse(reinterpret_cast<uint64_t&>(val));
	return reinterpret_cast<int64_t&>(swapped);
}
//We need to be careful with the floating point conversions.  Unlike above, strict-aliasing factors
//	in since e.g. `float` and `uint32_t` are not compatible types.  The best solution is
//	`memcpy(...)` which, amazingly, compiles out into ideal machine code.
inline    float reverse(   float val) {
	static_assert(sizeof(float)==4,"Float is not 4 bytes!");

	uint32_t temp0; memcpy(&temp0,&val, 4);
	uint32_t swapped = reverse(temp0);
	float temp1; memcpy(&temp1,&swapped, 4);

	return temp1;
}
inline   double reverse(  double val) {
	static_assert(sizeof(double)==8,"Double is not 8 bytes!");

	uint64_t temp0; memcpy(&temp0,&val, 8);
	uint64_t swapped = reverse(temp0);
	double temp1; memcpy(&temp1,&swapped, 8);

	return temp1;
}


}}}
