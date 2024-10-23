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
	constexpr uint TILE_X = 4;
	constexpr uint TILE_Y = 4;
	constexpr uint TILE_SIZE = TILE_X * TILE_Y;
	
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint tile_id = index / TILE_SIZE;
		uint32_t tile_x = tile_id % (args.framebuffer_width / TILE_X);
		uint32_t tile_y = tile_id / (args.framebuffer_width / TILE_X);

		uint thread_id = index % TILE_SIZE;
		uint32_t x = tile_x * TILE_X + thread_id % TILE_X;
		uint32_t y = tile_y * TILE_Y + thread_id / TILE_X;

		uint fb_index = y * args.framebuffer_width + x;
		rtm::RNG rng(fb_index);

		rtm::Ray ray = args.pregen_rays ? args.rays[fb_index] : args.camera.generate_ray_through_pixel(x, y);
		//ray.t_max = (1 << 23) - tile_id;
		uint steps = 0;
		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
		if(ray.t_min < ray.t_max)
		{
		#if defined(__riscv) &&  defined(DS_USE_RT_CORE)
			_traceray<0x0u>(index, ray, hit);
		#else
			intersect(args.nodes, args.tris, ray, hit, steps);
			//intersect(args.treelets, ray, hit, steps);
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

		//args.framebuffer[fb_index] = encode_pixel(steps / 64.0f);
	}
}

inline static void mandelbrot(const TRaXKernelArgs& args)
{
	constexpr uint MAX_ITERS = 100;
	for(uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		rtm::Ray ray = args.rays[index];

		// convert 1-d loop in to 2-d loop indices
		const int i = index / args.framebuffer_width;
		const int j = index % args.framebuffer_width;

		float zreal = 0.0f;
		float zimag = 0.0f;
		float creal = (j - args.framebuffer_width / 1.4f) / (args.framebuffer_width / 2.0f);
		float cimag = (i - args.framebuffer_height / 2.0f) / (args.framebuffer_height / 2.0f);

		float lengthsq;
		uint k = 0;
		do
		{
			float temp = (zreal * zreal) - (zimag * zimag) + creal;
			zimag = (2.0f * zreal * zimag) + cimag;
			zreal = temp;
			lengthsq = (zreal * zreal) + (zimag * zimag);
			++k;
		}
		while(lengthsq < 4.f && k < MAX_ITERS);

		if(k == MAX_ITERS)
			k = 0;

		args.framebuffer[index] = encode_pixel(k / (float)MAX_ITERS);
	}
}

#ifdef __riscv 
int main()
{
	kernel(*(const TRaXKernelArgs*)TRAX_KERNEL_ARGS_ADDRESS);
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

	args.pregen_rays = true;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	//args.camera = Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(0.0f, 0.0f, 5.0f));

	rtm::Mesh mesh("../../../datasets/intel-sponza.obj");
	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2("../../../datasets/cache/intel-sponza.bvh", build_objects, 2);
	mesh.reorder(build_objects);

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if (args.pregen_rays)
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, bvh2, mesh, 2, rays);
	args.rays = rays.data();

#if TRAX_USE_COMPRESSED_WIDE_BVH
	rtm::WideBVH wbvh(bvh2, build_objects);
	rtm::CompressedWideBVH cwbvh(wbvh);
	mesh.reorder(build_objects);
	args.nodes = cwbvh.nodes.data();

	rtm::CompressedWideTreeletBVH cwtbvh(cwbvh, mesh);
	args.treelets = cwtbvh.treelets.data();
#else
	rtm::WideBVH wbvh(bvh2, build_objects);
	mesh.reorder(build_objects);
	args.nodes = wbvh.nodes.data();

	rtm::WideTreeletBVH wtbvh(wbvh, mesh);
	args.treelets = wtbvh.treelets.data();
#endif

	//args.nodes = bvh2.nodes.data();

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = tris.data();
	
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = std::max(std::thread::hardware_concurrency() - 2u, 0u);
	for (uint i = 0; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
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
