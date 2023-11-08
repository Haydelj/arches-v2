#pragma once
#include "stdafx.hpp"

namespace rtm {

//vec4------------------------------------------------------------------------
class vec4
{
public:
	union
	{
		float e[4];
		struct
		{
			float x;
			float y;
			float z;
			float w;
		};
		struct
		{
			float r;
			float g;
			float b;
			float a;
		};
	};

	vec4() = default;
	vec4(float e0) { e[0] = e0; e[1] = e0; e[2] = e0; e[3] = e0; }
	vec4(float e0, float e1, float e2, float e3) { e[0] = e0; e[1] = e1; e[2] = e2; e[3] = e3; }

	inline const vec4& operator+() const { return *this; }
	inline vec4 operator-() const { return vec4(-e[0], -e[1], -e[2], -e[3]); }
	inline float operator[](int i) const { return e[i]; }
	inline float& operator[](int i) { return e[i]; }

	inline vec4& operator+=(const vec4& v);
	inline vec4& operator-=(const vec4& v);
	inline vec4& operator*=(const vec4& v);
	inline vec4& operator/=(const vec4& v);
	inline vec4& operator=(const vec4& v);
};

inline vec4& vec4::operator=(const vec4& v) {
	e[0] = v[0];
	e[1] = v[1];
	e[2] = v[2];
	e[3] = v[3];
	return *this;
}

inline vec4& vec4::operator+=(const vec4& v)
{
	e[0] += v[0];
	e[1] += v[1];
	e[2] += v[2];
	e[3] += v[3];
	return *this;
}

inline vec4& vec4::operator-=(const vec4& v)
{
	e[0] -= v[0];
	e[1] -= v[1];
	e[2] -= v[2];
	e[3] -= v[3];
	return *this;
}

inline vec4& vec4::operator*=(const vec4& v) {
	e[0] *= v[0];
	e[1] *= v[1];
	e[2] *= v[2];
	e[3] *= v[3];
	return *this;
}

inline vec4& vec4::operator/=(const vec4& v) {
	e[0] /= v[0];
	e[1] /= v[1];
	e[2] /= v[2];
	e[3] /= v[3];
	return *this;
}



inline vec4 operator+(const vec4& a, const vec4& b)
{
	return vec4(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
}

inline vec4 operator-(const vec4& a, const vec4& b)
{
	return vec4(a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]);
}

inline vec4 operator*(const vec4& a, const vec4& b)
{
	return vec4(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
}

inline vec4 operator/(const vec4& a, const vec4& b)
{
	return vec4(a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]);
}



inline float dot(const vec4& a, const vec4& b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

inline float length2(const vec4& v)
{
	return rtm::dot(v, v);
}

inline float length(const vec4& v)
{
	return fast_sqrtf(length2(v));
}

inline float inv_length(const vec4& v)
{
	return fast_inv_sqrtf(length2(v));
}

inline vec4 normalize(const vec4& v)
{
	return v / length(v);
}

inline vec4 min(const vec4& a, const vec4& b)
{
	return vec4(std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2]), std::min(a[3], b[3]));
}

inline vec4 max(const vec4& a, const vec4& b)
{
	return vec4(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]), std::max(a[3], b[3]));
}

inline vec4 clamp(const vec4& v, const vec4& min, const vec4& max)
{
	return rtm::min(rtm::max(v, min), max);
}

inline vec4 mix(const vec4& a, const vec4& b, float t)
{
	return t * (b - a) + a;
}

#ifdef RTM_POW
inline vec4 pow(const vec4 & a, const vec4 &b)
{
	return vec4(std::pow(a[0], b[0]), std::pow(a[1], b[1]), std::pow(a[2], b[2]), std::pow(a[3], b[3]));
}
#endif

}
