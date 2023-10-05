#pragma once

#include "rng.hpp"

namespace rtm
{

inline vec3 calculate_arbitrary_tangent(const vec3& normal)
{
	vec3 normalized = normalize(normal);

	if (abs(normalized[0]) < 0.707106f)
		return normalize(vec3(1.0f, 0.0f, 0.0f) - normalized * normalized.x);

	return normalize(vec3(0.0f, 1.0f, 0.0f) - normalized * normalized.y);
}

#if 0
inline vec2 uniform_sample_disk(RNG& rng)
{
	float r = sqrtf(rng.randf());
	float theta = 2.0f * PI * rng.randf();
	vec2 dir = normalize(vec2(cos_32(theta), sin_32(theta)));
	return r * dir;
}
#else
inline vec2 uniform_sample_disk(RNG& rng)
{
	vec2 r;
	while(length2(r = (rng.randv2() * 2.0f - vec2(1.0f))) > 1.0f);
	return r;
}
#endif

inline vec3 uniform_sample_disk(const vec3& normal, RNG& rng)
{
	vec2 sample = uniform_sample_disk(rng);
	vec3 tan = calculate_arbitrary_tangent(normal);
	vec3 bitan = cross(normal, tan);
	return sample[0] * tan + sample[1] * bitan;
}

inline vec3 cosine_sample_hemisphere(RNG& rng)
{
	vec2 sample = uniform_sample_disk(rng);
	return vec3(sample.x, sample.y, sqrt(1.0f - length2(sample)));
}

inline vec3 cosine_sample_hemisphere(const vec3& normal, RNG& rng)
{
	vec3 sample = cosine_sample_hemisphere(rng);
	vec3 tan = calculate_arbitrary_tangent(normal);
	vec3 bitan = cross(normal, tan);
	return normalize(sample[0] * tan + sample[1] * bitan + sample[2] * normal);
}

}