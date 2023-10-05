#pragma once

#include "float.hpp"

namespace rtm {

//vec3------------------------------------------------------------------------
class vec3
{
public:
	union
	{
		float e[3];
		struct
		{
			float x;
			float y;
			float z;
		};
		struct
		{
			float r;
			float g;
			float b;
		};
	};

	vec3() = default;
	vec3(float e0) { e[0] = e0; e[1] = e0; e[2] = e0; }
	vec3(float e0, float e1, float e2) { e[0] = e0; e[1] = e1; e[2] = e2; }

	inline const vec3& operator+() const { return *this; }
	inline vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
	inline float operator[](int i) const { return e[i]; }
	inline float& operator[](int i) { return e[i]; }

	inline vec3& operator=(const vec3& b);
	inline vec3& operator+=(const vec3& b);
	inline vec3& operator-=(const vec3& b);
	inline vec3& operator*=(const vec3& b);
	inline vec3& operator/=(const vec3& b);
};



inline vec3& vec3::operator=(const vec3& v) 
{
	e[0] = v[0];
	e[1] = v[1];
	e[2] = v[2];
	return *this;
}

inline vec3& vec3::operator+=(const vec3& v)
{
	e[0] += v[0];
	e[1] += v[1];
	e[2] += v[2];
	return *this;
}

inline vec3& vec3::operator-=(const vec3& v)
{
	e[0] -= v[0];
	e[1] -= v[1];
	e[2] -= v[2];
	return *this;
}

inline vec3& vec3::operator*=(const vec3& v) 
{
	e[0] *= v[0];
	e[1] *= v[1];
	e[2] *= v[2];
	return *this;
}

inline vec3& vec3::operator/=(const vec3& v) 
{
	e[0] /= v[0];
	e[1] /= v[1];
	e[2] /= v[2];
	return *this;
}



inline vec3 operator+(const vec3  &a, const vec3 &b)
{
	return vec3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

inline vec3 operator-(const vec3 &a, const vec3 &b)
{
	return vec3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

inline vec3 operator*(const vec3 &a, const vec3 &b)
{
	return vec3(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

inline vec3 operator/(const vec3 &a, const vec3 &b)
{
	return vec3(a[0] / b[0], a[1] / b[1], a[2] / b[2]);
}



inline float dot(const vec3 & a, const vec3 &b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline vec3 cross(const vec3 & a, const vec3 &b)
{
	return vec3(a[1] * b[2] - a[2] * b[1],
		(-(a[0] * b[2] - a[2] * b[0])),
		a[0] * b[1] - a[1] * b[0]);
}

inline float length2(const vec3& v)
{
	return rtm::dot(v, v);
}

inline float length(const vec3 &v)
{
	return sqrt(length2(v));
}

inline vec3 normalize(const vec3 &v)
{
	return v / length(v);
}

inline vec3 min(const vec3& a, const vec3& b)
{
	return vec3(rtm::min(a[0], b[0]), rtm::min(a[1], b[1]), rtm::min(a[2], b[2]));
}

inline vec3 max(const vec3& a, const vec3& b)
{
	return vec3(rtm::max(a[0], b[0]), rtm::max(a[1], b[1]), rtm::max(a[2], b[2]));
}

inline vec3 clamp(const vec3& v, const vec3& min, const vec3& max)
{
	return rtm::min(rtm::max(v, min), max);
}

inline vec3 mix(const vec3& a, const vec3& b, float t)
{
	return t * (b - a) + a;
}

inline vec3 reflect(const vec3& i, const vec3 &n)
{
	return 2 * dot(i, i) * n - i;
}

#ifdef RTM_POW
inline vec3 pow(const vec3& a, const vec3& b)
{
	return vec3(std::pow(a[0], b[0]), std::pow(a[1], b[1]), std::pow(a[2], b[2]));
}
#endif

}