#pragma once
#include "stdafx.hpp"

#include"rtm.hpp"

inline rtm::vec3 calculate_arbitrary_tangent(const rtm::vec3& normal)
{
	rtm::vec3 normalized = rtm::normalize(normal);

	if (std::abs(normalized[0]) < 0.707106f)
		return rtm::normalize(rtm::vec3(1.0f, 0.0f, 0.0f) - normalized * normalized.x);

	return rtm::normalize(rtm::vec3(0.0f, 1.0f, 0.0f) - normalized * normalized.y);
}


// approximation from: http://nnp.ucsd.edu/phy120b/student_dirs/shared/gnarly/misc_docs/trig_approximations.pdf
inline static float cos_32s(float x)
{
	constexpr float c1 = 0.99940307f;
	constexpr float c2 =-0.49558072f;
	constexpr float c3 = 0.03679168f;
	float x_sq = x * x;
	return (c1 + x_sq * (c2 + c3 * x_sq));
}

//only valid to 3.2 digits and for -2*PI to 2*PI
inline static float cos_32(float x)
{
	//x = x % twopi; // Get rid of values > 2* pi
	x = std::abs(x); // cos(-x) = cos(x)
	uint32_t quad = static_cast<uint>(x * (2.0f / PI)); // Get quadrant # (0 to 3)
	//uint32_t quad = static_cast<uint>(x / (PI / 2.0f)); // Get quadrant # (0 to 3
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

#if 1
inline rtm::vec2 uniform_sample_circle(RNG& rng)
{
	float r = fast_sqrtf(rng.randf());
	float theta = 2.0f * PI * rng.randf();
	rtm::vec2 dir = rtm::normalize(rtm::vec2(cos_32(theta), sin_32(theta)));
	return r * dir;
}
#else
inline rtm::vec2 uniform_sample_circle(RNG& rng)
{
	rtm::vec2 r;
	while(rtm::squared_length(r = (rng.randv2() * 2.0f - rtm::vec2(1.0f))) > 1.0f);
	return r;
}
#endif

inline rtm::vec3 uniform_sample_circle(const rtm::vec3& normal, RNG& rng)
{
	rtm::vec2 sample = uniform_sample_circle(rng);
	rtm::vec3 tan = calculate_arbitrary_tangent(normal);
	rtm::vec3 bitan = rtm::cross(normal, tan);
	return sample[0] * tan + sample[1] * bitan;
}

inline rtm::vec3 cosine_sample_hemisphere(RNG& rng)
{
	rtm::vec2 sample = uniform_sample_circle(rng);
	return rtm::vec3(sample.x, sample.y, fast_sqrtf(1.0f - rtm::length2(sample)));
}

inline rtm::vec3 cosine_sample_hemisphere(const rtm::vec3& normal, RNG& rng)
{
	rtm::vec3 sample = cosine_sample_hemisphere(rng);
	rtm::vec3 tan = calculate_arbitrary_tangent(normal);
	rtm::vec3 bitan = rtm::cross(normal, tan);
	return rtm::normalize(sample[0] * tan + sample[1] * bitan + sample[2] * normal);
}
