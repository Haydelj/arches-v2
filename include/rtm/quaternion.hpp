#pragma once

#include "float.hpp"

namespace rtm {

class quaternion
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
    };

    quaternion() = default;
	quaternion(float e0) { e[0] = e0; e[1] = e0; e[2] = e0; e[3] = e0; }
	quaternion(float e0, float e1, float e2, float e3) { e[0] = e0; e[1] = e1; e[2] = e2; e[3] = e3; }
	
	inline const quaternion& operator+() const { return *this; }
	inline quaternion operator-() const { return quaternion(-e[0], -e[1], -e[2], -e[3]); }
	inline float operator[](int i) const { return e[i]; }
	inline float& operator[](int i) { return e[i]; }

	inline quaternion& operator+=(const quaternion& v);
	inline quaternion& operator-=(const quaternion& v);
	inline quaternion& operator*=(const quaternion& v);
	inline quaternion& operator/=(const quaternion& v);
	inline quaternion& operator=(const quaternion& v);
};

inline quaternion& quaternion::operator=(const quaternion& v) {
	e[0] = v[0];
	e[1] = v[1];
	e[2] = v[2];
	e[3] = v[3];
	return *this;
}

inline quaternion& quaternion::operator+=(const quaternion& v)
{
	e[0] += v[0];
	e[1] += v[1];
	e[2] += v[2];
	e[3] += v[3];
	return *this;
}

inline quaternion& quaternion::operator-=(const quaternion& v)
{
	e[0] -= v[0];
	e[1] -= v[1];
	e[2] -= v[2];
	e[3] -= v[3];
	return *this;
}

inline quaternion& quaternion::operator*=(const quaternion& v) {
	e[0] *= v[0];
	e[1] *= v[1];
	e[2] *= v[2];
	e[3] *= v[3];
	return *this;
}

inline quaternion& quaternion::operator/=(const quaternion& v) {
	e[0] /= v[0];
	e[1] /= v[1];
	e[2] /= v[2];
	e[3] /= v[3];
	return *this;
}



inline quaternion operator+(const quaternion& a, const quaternion& b)
{
	return quaternion(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
}

inline quaternion operator-(const quaternion& a, const quaternion& b)
{
	return quaternion(a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]);
}

inline quaternion operator*(const quaternion& a, const quaternion& b)
{
	return quaternion(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
}

inline quaternion operator/(const quaternion& a, const quaternion& b)
{
	return quaternion(a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]);
}

inline float dot(const quaternion& a, const quaternion& b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

inline float length2(const quaternion& v)
{
	return rtm::dot(v, v);
}

inline float length(const quaternion& v)
{
	return sqrt(length2(v));
}

inline quaternion normalize(const quaternion& v)
{
	return v / length(v);
}

}

