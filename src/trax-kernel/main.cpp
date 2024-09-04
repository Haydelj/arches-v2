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

inline static void kernel(const TRaXKernelArgs& args)
{
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint tile_id = index / 16;
		uint32_t tile_x = tile_id % (args.framebuffer_width / 4);
		uint32_t tile_y = tile_id / (args.framebuffer_width / 4);

		uint thread_id = index % 16;
		uint32_t x = tile_x * 4 + thread_id % 4;
		uint32_t y = tile_y * 4 + thread_id / 4;

		uint fb_index = y * args.framebuffer_width + x;
		rtm::RNG rng(fb_index);

		rtm::Ray ray = args.pregen_rays ? args.rays[fb_index] : args.camera.generate_ray_through_pixel(x, y);

		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
		if(ray.t_min < ray.t_max)
		{
		#if defined(__riscv) &&  defined(USE_RT_CORE)
			_traceray<0x0u>(index, ray, hit);
		#else
			intersect(args.nodes, args.tris, ray, hit);
		#endif
		}

		if(hit.id != ~0u)
		{
			args.framebuffer[fb_index] = rtm::RNG::hash(hit.id) | 0xff000000;
		}
		else
		{
			args.framebuffer[fb_index] = 0xff000000;
		}
	}
}

inline static void kernel2(const TRaXKernelArgs& args)
{
	for(uint index = fchthrd(); index < args.framebuffer_size / PACKET_SIZE; index = fchthrd())
	{
		uint32_t tile_x = (index % (args.framebuffer_width / 4) * 4);
		uint32_t tile_y = (index / (args.framebuffer_width / 4) * 4);

		rtm::Frustum frustrum = args.camera.generate_frustum_for_tile(tile_x, tile_y);

		rtm::Hit hit_buffer[PACKET_SIZE]; 
		for(uint i = 0; i < PACKET_SIZE; ++i)
			hit_buffer[i] = rtm::Hit(frustrum.t_max, rtm::vec2(0.0f), ~0u);

		uint64_t mask = intersect(args.nodes, args.tris, frustrum, hit_buffer);

		for(uint i = 0; i < PACKET_SIZE; ++i)
		{
			uint fb_index = (tile_x + (i % 4)) + (tile_y + (i / 4)) * args.framebuffer_width;
			if(hit_buffer[i].id != ~0u)
			{
				args.framebuffer[fb_index] = rtm::RNG::hash(hit_buffer[i].id) | 0xff000000;
			}
			else
			{
				args.framebuffer[fb_index] = 0xff000000;
			}
		}
	}
}

#ifdef __riscv 
int main()
{
	kernel(*(const TRaXKernelArgs*)KERNEL_ARGS_ADDRESS);
	return 0;
}

#else
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"

int main(int argc, char* argv[])
{
	TRaXKernelArgs args;
	args.framebuffer_width = 1024;
	args.framebuffer_height = 1024;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.pregen_rays = false;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	//args.camera = Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(0.0f, 0.0f, 5.0f));

	rtm::Mesh mesh("../../../datasets/intel-sponza.obj");
	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2("../../../datasets/cache/intel-sponza_bvh.cache", build_objects, 2);
	mesh.reorder(build_objects);

	rtm::PackedBVH2 packed_bvh2(bvh2, build_objects);
	rtm::PackedTreeletBVH treelet_bvh(packed_bvh2, mesh);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);

	args.nodes = packed_bvh2.nodes.data();
	args.treelets = treelet_bvh.treelets.data();
	args.tris = tris.data();

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args, 1, rays);
	args.rays = rays.data();

	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = std::max(std::thread::hardware_concurrency() - 2u, 0u);
	for (uint i = 0; i < thread_count; ++i) threads.emplace_back(kernel2, args);
	kernel2(args);
	for (uint i = 0; i < thread_count; ++i) threads[i].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
