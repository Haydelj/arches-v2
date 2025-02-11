#pragma once

#include "vec3.hpp"
#include "quaternion.hpp"

namespace rtm {

//mat3------------------------------------------------------------------------
class mat3
{
public:
	vec3 v[3];

	mat3() = default;
	mat3(vec3 v) { this->v[0] = vec3(v[0], 0.0f, 0.0f); this->v[1] = vec3(0.0f, v[1], 0.0f); this->v[2] = vec3(0.0f, 0.0f, v[2]); }
	mat3(vec3 v0, vec3 v1, vec3 v2) { v[0] = v0; v[1] = v1; v[2] = v2; }

	mat3(const quaternion& q) 
	{
		//v[0] = vec3(1.0f - 2.0f * q.y * q.y - 2.0f * q.z * q.z, 2.0f * q.x * q.y - 2.0f * q.z * q.w,        2.0f * q.x * q.z + 2.0f * q.y * q.w);
		//v[1] = vec3(2.0f * q.x * q.y + 2.0f * q.z * q.w,        1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z, 2.0f * q.y * q.z - 2.0f * q.x * q.w);
		//v[2] = vec3(2.0f * q.x * q.z - 2.0f * q.y * q.w,        2.0f * q.y * q.z + 2.0f * q.x * q.w,        1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y);
		v[0] = vec3(1.0f - 2.0f * q.y * q.y - 2.0f * q.z * q.z, 2.0f * q.x * q.y + 2.0f * q.z * q.w,        2.0f * q.x * q.z - 2.0f * q.y * q.w);
		v[1] = vec3(2.0f * q.x * q.y - 2.0f * q.z * q.w,        1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z, 2.0f * q.y * q.z + 2.0f * q.x * q.w);
		v[2] = vec3(2.0f * q.x * q.z + 2.0f * q.y * q.w,        2.0f * q.y * q.z - 2.0f * q.x * q.w,        1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y);
	}

	inline const mat3& operator+() const { return *this; }
	inline mat3 operator-() const { return mat3(-v[0], -v[1], -v[2]); }
	inline vec3 operator[](int i) const { return v[i]; }
	inline vec3& operator[](int i) { return v[i]; }

	inline mat3& operator=(const mat3& b);
	inline mat3& operator+=(const mat3& b);
	inline mat3& operator-=(const mat3& b);
};

inline mat3& mat3::operator=(const mat3& m)
{
	v[0] = m[0];
	v[1] = m[1];
	v[2] = m[2];
	return *this;
}

inline mat3& mat3::operator+=(const mat3& m)
{
	v[0] += m[0];
	v[1] += m[1];
	v[2] += m[2];
	return *this;
}

inline mat3& mat3::operator-=(const mat3& m)
{
	v[0] -= m[0];
	v[1] -= m[1];
	v[2] -= m[2];
	return *this;
}

inline mat3 operator+(const mat3& a, const mat3& b)
{
	return mat3(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

inline mat3 operator-(const mat3& a, const mat3& b)
{
	return mat3(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

inline mat3 transpose(const mat3& m)
{
	return mat3(
		vec3(m[0][0], m[1][0], m[2][0]),
		vec3(m[0][1], m[1][1], m[2][1]),
		vec3(m[0][2], m[1][2], m[2][2]));
}

inline vec3 operator*(const mat3& m, const vec3& v)
{
	mat3 mt = transpose(m);
	return vec3(dot(mt[0], v), dot(mt[1], v), dot(mt[2], v));
}

inline mat3 operator*(const mat3& a, const mat3& b)
{
	return mat3(a * b[0], a * b[1], a * b[2]);
}

}