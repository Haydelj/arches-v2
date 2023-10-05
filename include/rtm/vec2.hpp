#pragma once

#include "float.hpp"

namespace rtm {

class vec2
{
public:
	union
	{
		float e[2];
		struct
		{
			float x;
			float y;
		};
		struct
		{
			float u;
			float v;
		};
	};
	
	vec2() = default;
	vec2(float e0) { e[0] = e0; e[1] = e0; }
	vec2(float e0, float e1) { e[0] = e0; e[1] = e1; }

	inline const vec2& operator+() const { return *this; }
	inline vec2 operator-() const { return vec2(-e[0], -e[1]); }
	inline float operator[](int i) const { return e[i]; }
	inline float& operator[](int i) { return e[i]; }

	inline vec2& operator=(const vec2& b);
	inline vec2& operator+=(const vec2& b);
	inline vec2& operator-=(const vec2& b);
	inline vec2& operator*=(const vec2& b);
	inline vec2& operator/=(const vec2& b);
};



inline vec2& vec2::operator=(const vec2& v)
{
	e[0] = v[0];
	e[1] = v[1];
	return *this;
}

inline vec2& vec2::operator+=(const vec2 &v)
{
	e[0] += v[0];
	e[1] += v[1];
	return *this;
}

inline vec2& vec2::operator-=(const vec2 &v)
{
	e[0] -= v[0];
	e[1] -= v[1];
	return *this;
}

inline vec2& vec2::operator*=(const vec2 &v)
{
	e[0] *= v.e[0];
	e[1] *= v.e[1];
	return *this;
}

inline vec2& vec2::operator/=(const vec2 &v)
{
	e[0] /= v.e[0];
	e[1] /= v.e[1];
	return *this;
}



inline vec2 operator+(const vec2& a, const vec2& b)
{
	return vec2(a[0] + b[0], a[1] + b[1]);
}

inline vec2 operator-(const vec2& a, const vec2& b)
{
	return vec2(a[0] - b[0], a[1] - b[1]);
}

inline vec2 operator*(const vec2& a, const vec2& b)
{
	return vec2(a.e[0] * b.e[0], a.e[1] * b.e[1]);
}

inline vec2 operator/(const vec2 &a, const vec2 &b)
{
	return vec2(a.e[0] / b.e[0], a.e[1] / b.e[1]);
}


inline float dot(const vec2& a, const vec2 &b)
{
	return a.e[0] * b.e[0] + a.e[1] * b.e[1];
}

inline float cross(const vec2& a, const vec2& b)
{
	return a.x * b.y - b.x * a.y;
}

inline float length2(const vec2 &v)
{
	return rtm::dot(v, v);
}

inline float length(const vec2& v)
{
	return sqrt(length2(v));
}

inline vec2 normalize(const vec2 &v)
{
	return v / length(v);
}

inline vec2 min(const vec2& a, const vec2& b)
{
	return vec2(rtm::min(a[0], b[0]), rtm::min(a[1], b[1]));
}

inline vec2 max(const vec2& a, const vec2& b)
{
	return vec2(rtm::max(a[0], b[0]), rtm::max(a[1], b[1]));
}

inline vec2 clamp(const vec2& v, const vec2& min, const vec2& max)
{
	return rtm::min(rtm::max(v, min), max);
}

inline vec2 mix(const vec2& a, const vec2& b, float t)
{
	return t * (b - a) + a;
}

#ifdef RTM_POW
inline vec2 pow(const vec2& a, const vec2& b)
{
	return vec2(std::pow(a[0], b[0]), std::pow(a[1], b[1]));
}
#endif

}