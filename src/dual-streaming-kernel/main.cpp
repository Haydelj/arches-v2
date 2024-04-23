#include "stdafx.hpp"

//#include "include.hpp"
#include "intersect.hpp"
#include "custom-instr.hpp"

static uint32_t encode_pixel(rtm::vec3 in)
{
	in = rtm::clamp(in, 0.0f, 1.0f);
	uint32_t out = 0u;
	out |= static_cast<uint32_t>(in.r * 255.0f + 0.5f) << 0;
	out |= static_cast<uint32_t>(in.g * 255.0f + 0.5f) << 8;
	out |= static_cast<uint32_t>(in.b * 255.0f + 0.5f) << 16;
	out |= 0xff << 24;
	return out;
}

void inline barrier()
{

}

inline static void kernel(const DualStreamingKernelArgs& args)
{
#if __riscv
	uint index;

	for (index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint x = index % args.framebuffer_width;
		uint y = index / args.framebuffer_width;

		WorkItem wi;
		wi.bray.ray = args.pregen_rays ? args.rays[index] : args.camera.generate_ray_through_pixel(x, y);
		wi.bray.id = index;
		wi.segment_id = 0;
		wi.order_hint = 0;
		_swi(wi); //write root ray to ray bucket
	}

#ifndef USE_RT_CORE
	intersect_buckets(args); //use software traversal
#endif

	for (index = index - args.framebuffer_size; index < args.framebuffer_size; index = fchthrd() - args.framebuffer_size)
	{
		rtm::Hit hit = _lhit(args.hit_records + index);
#else
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint x = index % args.framebuffer_width;
		uint y = index / args.framebuffer_width;

		WorkItem wi;
		rtm::Ray ray = args.pregen_rays ? args.rays[index] : args.camera.generate_ray_through_pixel(x, y);
		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);

		intersect(args.treelets, ray, hit);
#endif
		uint out = 0xff000000;
		if (hit.id != ~0u)
			out |= rtm::RNG::hash(hit.id);
		args.framebuffer[index] = out;
	}
}

#ifdef __riscv 
int main()
{
	kernel(*(const DualStreamingKernelArgs*)KERNEL_ARGS_ADDRESS);
	return 0;
}

#else
#define MULTI_THREADED
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"
int main(int argc, char* argv[])
{
	DualStreamingKernelArgs args;
	args.framebuffer_width = 1024;
	args.framebuffer_height = 1024;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.pregen_rays = false;

	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(6.78f, -5.87, 0.84), rtm::vec3(7.78f, -5.87, 0.84f));
	
	std::string dataset_path = "../../datasets/";
	//std::string scene_name = "sponza";  args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	std::string scene_name = "san-miguel"; args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));
	//std::string scene_name = "san-miguel"; args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(6.37319, -5.62511, 1.53861), rtm::vec3(0.72299, 0.68257, 0.10671));
	args.light_dir = rtm::normalize(rtm::vec3(4.5, 42.5, 5.0));

	rtm::Mesh mesh(dataset_path + scene_name + ".obj"); 
	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2(dataset_path + "cache/" + scene_name + "_bvh.cache", build_objects, 2);
	mesh.reorder(build_objects);

	rtm::PackedBVH2 packed_bvh2(bvh2, build_objects);
	rtm::PackedTreeletBVH treelet_bvh(packed_bvh2, mesh);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);

	std::vector<rtm::Hit> hit_buffer(args.framebuffer_size);

	args.treelets = treelet_bvh.treelets.data();
	args.tris = tris.data();
	args.hit_records = hit_buffer.data();

	for (int i = 0; i < args.framebuffer_size; i++)
		args.hit_records[i].t = T_MAX;

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args, 0, rays);
	args.rays = rays.data();
	
	auto start = std::chrono::high_resolution_clock::now();

#ifdef MULTI_THREADED
	std::vector<std::thread> threads;
	uint thread_count = std::thread::hardware_concurrency() - 1u;
	for (uint i = 0; i < thread_count; ++i) 
		threads.emplace_back(kernel, args);
	kernel(args);

	for (uint i = 0; i < thread_count; ++i)
		threads[i].join();
#else
	kernel(args);
#endif

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
