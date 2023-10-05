#pragma once

#include "ray.hpp"
#include "sampling.hpp"

namespace rtm
{

class Camera
{
private:
	float _drdt;
	rtm::vec2 _recip_res;
	rtm::vec3 _position, _x, _y, _z;

public:
	Camera() = default;

	#ifndef __riscv
	Camera(uint width, uint height, float focal_length = 50.0f,
		rtm::vec3 position = rtm::vec3(0.0f, 0.0f, 1.0f),
		rtm::vec3 target = rtm::vec3(0.0f, 0.0f, 0.0f),
		rtm::vec3 up = rtm::vec3(0.0f, 1.0f, 0.0f))
	{
		float aspect = (float)width / height;
		
		_recip_res.x = 1.0f / width;
		_recip_res.y = 1.0f / height;

		_drdt = 24.0f / focal_length / height;
		_drdt = sqrt(_drdt * _drdt / 3.1415926);

		_position = position;
		rtm::vec3 direction = target - position;
		_z = -rtm::normalize(direction);
		_x = rtm::normalize(cross(up, _z));
		_y = rtm::cross(_z, _x);

		_x *= 24.0f / focal_length * aspect;
		_y *= 24.0f / focal_length;
	}
	#endif

	Ray generate_ray_through_pixel(uint i, uint j, rtm::RNG* rng = nullptr) const
	{
		rtm::vec2 uv((float)i, (float)j);
		if(rng) uv += rng->randv2();
		else    uv += rtm::vec2(0.5f);
		uv *= _recip_res;
		uv -= rtm::vec2(0.5f);

		Ray ray;
		ray.d = (_x * uv.x + _y * uv.y - _z);
		ray.t_min = T_MIN;
		ray.o = _position;
		ray.t_max = T_MAX;

		return ray;
	}
};

}