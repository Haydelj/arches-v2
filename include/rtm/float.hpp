#pragma once

#include "macros.hpp"
#include "int.hpp"

#ifndef __riscv
#include <cmath>
#include <intrin.h>
#include <cfenv>
#endif

namespace rtm
{

inline uint32_t to_u32(float f32)
{
	return *((uint32_t*)&f32);
}

inline float to_f32(uint32_t u32)
{
	return *((float*)&u32);
}

inline float min(float a, float b) { return (b < a) ? b : a;}
inline float max(float a, float b) { return (a < b) ? b : a;}
inline float abs(float a) { return a > 0.0f ? a : -a; }

inline float sqrt(float input)
{
    #ifdef __riscv
    float output;
	asm volatile ("fsqrt.s %0, %1\n\t" : "=f" (output) : "f" (input));
	return output;
    #else
	return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ps1(input)));
	#endif
}

inline float rsqrt(float input)
{
	#ifdef __riscv
	float output;
	asm volatile ("frsqrt.s %0, %1\n\t" : "=f" (output) : "f" (input));
	return output;
	#else
	return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ps1(input)));
	#endif
}

inline float rcp(float input)
{
	#ifdef __riscv
	float output;
	asm volatile ("frcp.s %0, %1\n\t" : "=f" (output) : "f" (input));
	return output;
	#else
	return _mm_cvtss_f32(_mm_rcp_ss(_mm_set_ps1(input)));
	#endif
}

//only valid to 3.2 digits and for -2*PI to 2*PI
inline static float cos_32s(float x)
{
	constexpr float c1 = 0.99940307f;
	constexpr float c2 =-0.49558072f;
	constexpr float c3 = 0.03679168f;
	float x_sq = x * x;
	return (c1 + x_sq * (c2 + c3 * x_sq));
}

inline static float cos_32(float x)
{
	x = abs(x); // cos(-x) = cos(x)
	if(x > (2.0f * PI))
	{
		// Get rid of values > 2PI
		uint n  = (uint)(x * (0.5f / PI));
		x -= n * 2.0f * PI;
	}

	uint32_t quad = (uint)(x * (2.0f / PI)); // Get quadrant # (0 to 3)
	switch (quad)
	{
	case 0: return cos_32s(x);
	case 1: return -cos_32s(PI - x);
	case 2: return -cos_32s(x - PI);
	case 3: return cos_32s(2.0f * PI - x);
	}

	return 0.0f;
}

//only valid to 3.2 digits and for -3/2*PI to 5/2*PI
inline static float sin_32(float x) { return cos_32(x - PI / 2.0f);}

inline float cos(float input)
{
	#ifdef __riscv
	return cos_32(input);
	#else
	return _mm_cvtss_f32(_mm_cos_ps(_mm_set_ps1(input)));
	#endif
}

inline float sin(float input)
{
	#ifdef __riscv
	return sin_32(input);
	#else
	return _mm_cvtss_f32(_mm_sin_ps(_mm_set_ps1(input)));
	#endif
}

union float32_bf
{
	struct
	{
		uint32_t mantisa : 23;
		uint32_t exp : 8;
		uint32_t sign : 1;
	};
	float f32;
	uint32_t u32;

	float32_bf(float f) : f32(f) {}
	float32_bf(uint8_t sign, uint8_t exp, uint32_t mantisa) : sign(sign), exp(exp), mantisa(mantisa) {}
};

#ifndef __riscv
inline int32_t f32_to_i24(float f32, uint8_t max_exp = 127, int rounding = 0)
{
	float32_bf mult(0, 2 * 127 - max_exp + 23, 0x0);
	double norm = (double)f32 * mult.f32;

	if(rounding == 0)
		norm = std::round(norm);
	else if(rounding == -1)
		norm = std::floor(norm);
	else if(rounding == 1)
		norm = std::ceil(norm);
	
	if(norm > ((1 << 23) - 1) || norm < -(1 << 23)) __debugbreak();

	return (int32_t)norm;
}
#endif

inline float i24_to_f32(int32_t i24, uint8_t max_exp = 127)
{
	//ensure proper sign extension
	i24 &= 0xffffff;
	if(i24 >= 1 << 23) i24 |= 0xff000000;
	float32_bf mult(0, max_exp - 23, 0x0);
	return mult.f32 * i24;
}

#ifndef __riscv
inline uint16_t f32_to_i16(float f32, uint8_t max_exp = 127, int rounding = 0)
{
	float32_bf mult(0, 2 * 127 - max_exp + 15, 0x0);
	float norm = f32 * mult.f32;

	if(rounding == 0)
		norm = std::round(norm);
	else if(rounding == -1)
		norm = std::floor(norm);
	else if(rounding == 1)
		norm = std::ceil(norm);

	if(norm > ((1 << 15) - 1) || norm < -(1 << 15)) __debugbreak();
	return (int16_t)norm;
}
#endif

//input needs to be low 24 bits, with 0 in the top 8 bits
inline float i16_to_f32(int16_t i16, uint8_t max_exp = 127)
{
	float32_bf mult(0, max_exp - 15, 0x0);
	return mult.f32 * i16;
}

}