#include "stdafx.hpp"

#include "include.hpp"
#include "intersect.hpp"
#include "custom-instr.hpp"

inline static uint32_t encode_pixel(rtm::vec3 in)
{
	in = rtm::clamp(in, 0.0f, 1.0f);
	uint32_t out = 0u;
	out |= static_cast<uint32_t>(in.r * 255.0f + 0.5f) << 0;
	out |= static_cast<uint32_t>(in.g * 255.0f + 0.5f) << 8;
	out |= static_cast<uint32_t>(in.b * 255.0f + 0.5f) << 16;
	out |= 0xff << 24;
	return out;
}

// Spherical harmonics coefficients
const float SH_C0 = 0.28209479177387814f;
const float SH_C1 = 0.4886025119029199f;
const float SH_C2[] = {
	1.0925484305920792f,
	-1.0925484305920792f,
	0.31539156525252005f,
	-1.0925484305920792f,
	0.5462742152960396f
};
const float SH_C3[] = {
	-0.5900435899266435f,
	2.890611442640554f,
	-0.4570457994644658f,
	0.3731763325901154f,
	-0.4570457994644658f,
	1.445305721320277f,
	-0.5900435899266435f
};

rtm::vec3 spherical_harmonic(const rtm::vec3& d, const rtm::vec3 sh[16])
{
	constexpr uint deg = 0;

	rtm::vec3 result = SH_C0 * sh[0];
	if(deg > 0)
	{
		rtm::vec3 _d = rtm::normalize(d);
		float x = _d.x;
		float y = _d.y;
		float z = _d.z;
		result = result - SH_C1 * y * sh[1] + SH_C1 * z * sh[2] - SH_C1 * x * sh[3];

		if(deg > 1)
		{
			float xx = x * x, yy = y * y, zz = z * z;
			float xy = x * y, yz = y * z, xz = x * z;
			result = result +
				SH_C2[0] * xy * sh[4] +
				SH_C2[1] * yz * sh[5] +
				SH_C2[2] * (2.0f * zz - xx - yy) * sh[6] +
				SH_C2[3] * xz * sh[7] +
				SH_C2[4] * (xx - yy) * sh[8];

			if(deg > 2)
			{
				result = result +
					SH_C3[0] * y * (3.0f * xx - yy) * sh[9] +
					SH_C3[1] * xy * z * sh[10] +
					SH_C3[2] * y * (4.0f * zz - xx - yy) * sh[11] +
					SH_C3[3] * z * (2.0f * zz - 3.0f * xx - 3.0f * yy) * sh[12] +
					SH_C3[4] * x * (4.0f * zz - xx - yy) * sh[13] +
					SH_C3[5] * z * (xx - yy) * sh[14] +
					SH_C3[6] * x * (xx - 3.0f * yy) * sh[15];
			}
		}
	}
	result += 0.5f;

	return rtm::max(result, 0.0f);
}


inline static void kernel(const SGKernel::Args& args)
{
	constexpr uint TILE_X = 4;
	constexpr uint TILE_Y = 8;
	constexpr uint TILE_SIZE = TILE_X * TILE_Y;
	
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint tile_id = index / TILE_SIZE;
		//tile_id = rtm::RNG::hash(tile_id) % (args.framebuffer_size / TILE_SIZE);

		uint32_t tile_x = tile_id % (args.framebuffer_width / TILE_X);
		uint32_t tile_y = tile_id / (args.framebuffer_width / TILE_X);

		uint thread_id = index % TILE_SIZE;
		uint32_t x = tile_x * TILE_X + thread_id % TILE_X;
		uint32_t y = tile_y * TILE_Y + thread_id / TILE_X;

		uint fb_index = y * args.framebuffer_width + x;
		rtm::RNG rng(fb_index);

		rtm::Ray ray = args.pregen_rays ? args.rays[fb_index] : args.camera.generate_ray_through_pixel(x, y);

		ray.t_min = u_to_f((f_to_u(ray.t_min) & 0xffff0000) | (index / 2 & 0xffff));

		uint steps = 0;

		float opacity = 1.0;
		rtm::vec3 color(0.0);
		while(opacity > 0.0001f)
		{
		#ifdef __riscv
			SGKernel::HitPacket hit_packet = _traceray<0x0u>(ray, opacity);
		#else
			SGKernel::HitPacket hit_packet = intersect(args.nodes, args.sgs, ray, opacity, steps);
		#endif
			if(hit_packet.size > 0)
			{
				for(uint i = 0; i < hit_packet.size; ++i)
				{
					SGKernel::Hit hit = hit_packet.hits[i];
					float a = hit.a * (1.0f / 255.0f);
					//rtm::vec3 c = rtm::vec3(rtm::RNG(hit.id).randv3());
					rtm::vec3 c = spherical_harmonic(ray.d, args.shs[hit.id].sh);
					color += opacity * a * c;
					opacity *= (1.0f - a);
				}
				ray.t_min = hit_packet.last_t;
			}
			else break;
		}

		//color += opacity * rtm::vec3(1.0f, 0.0f, 1.0f);
		args.framebuffer[fb_index] = encode_pixel(color);

		//args.framebuffer[fb_index] = encode_pixel((float)steps / (1 << 12));
	}
}

#ifdef __riscv 
int main()
{
	kernel(*(const SGKernel::Args*)KERNEL_ARGS_ADDRESS);
	return 0;
}

#else
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"

int main(int argc, char* argv[])
{
	SGKernel::Args args;
	args.framebuffer_width = 1024;
	args.framebuffer_height = 1024;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.pregen_rays = false;

	//intel sponza camera
	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 40.0f, rtm::vec3(-3, 0.0, -3), rtm::vec3(0, 1.0, 0));
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 400.0f, rtm::vec3(0.0, 0.0, 100), rtm::vec3(0, 0.0, 0));

	// san miguel camera
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));

	// hairball camera
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(0, 10, 10), rtm::vec3(10, 0, 0));

	std::vector<rtm::SphericalGaussian> sgs;
	std::vector<rtm::SphericalHarmonic> shs;
	rtm::PLY ply("../../../datasets/point-cloud.ply");
	ply.read(sgs, shs);
	//sgs.resize(1024);

	//float theta = PI * 0.25;
	//rtm::vec3 axis(0.0, 0.0, 1.0);
	//float half_theta = theta * 0.5f;
	//sgs.emplace_back();
	//sgs.back().position = rtm::vec3(0.0, 0.0, 0.0);
	//sgs.back().scale = rtm::vec3(0.1, 0.01, 0.01);
	//sgs.back().rotation = rtm::normalize(rtm::quaternion(1,1,1,10));
	//sgs.back().opacity = 0.99;
	//sgs.back().sh[0] = rtm::vec3(0.0, 1.0, 0.0);
	//sgs.emplace_back();
	//sgs.back().position = rtm::vec3(-0.5, 0.0, 0.0);
	//sgs.back().scale = rtm::vec3(0.5);
	//sgs.back().rotation = rtm::normalize(rtm::quaternion(0.0, 0.0, 0.0, 1.0));
	//sgs.back().opacity = 0.99;
	//sgs.back().sh[0] = rtm::vec3(1.0, 0.0, 0.0);
	 
	std::vector<rtm::BVH2::BuildObject> build_objects;
	for(uint i = 0; i < sgs.size(); ++i)
	{
		rtm::BVH2::BuildObject obj = sgs[i].build_object();
		if(obj.aabb.surface_area() > 0.0f)
		{
			build_objects.push_back(obj);
			build_objects.back().index = i;
		}
	}

	rtm::BVH2 bvh2("../../../datasets/cache/point-cloud.bvh", build_objects, 2);
	rtm::WideBVH wbvh(bvh2, build_objects);
	rtm::CompressedWideBVH cwbvh(wbvh);

	
	{
		std::vector<rtm::SphericalGaussian> temp_sgs(sgs);
		std::vector<rtm::SphericalHarmonic> temp_shs(shs);
		for(uint i = 0; i < build_objects.size(); ++i)
		{
			sgs[i] = temp_sgs[build_objects[i].index];
			shs[i] = temp_shs[build_objects[i].index];
			build_objects[i].index = i;
		}
	}

	args.nodes = cwbvh.nodes.data();
	args.sgs = sgs.data();
	args.shs = shs.data();
	
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = std::max(std::thread::hardware_concurrency() - 2u, 0u);
	for (uint i = 0; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint i = 0; i < thread_count; ++i) threads[i].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(false);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
