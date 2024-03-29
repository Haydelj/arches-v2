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
	//if (args.use_secondary_rays)
	//{
	//	uint index;
	//	for (index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	//	{
	//		uint x = index % args.framebuffer_width;
	//		uint y = index / args.framebuffer_width;
	//		rtm::RNG rng(index);
	//		rtm::Ray ray = args.secondary_rays[index];
	//		if (ray.t_max > 0)
	//		{
	//			WorkItem wi;
	//			wi.bray.ray = ray;
	//			wi.bray.id = index;
	//			wi.segment = 0;
	//			_swi(wi);
	//		}
	//	}
	//	intersect_buckets(args);
	//	////spin sleep
	//	for (volatile uint i = 0; i < 1024; i++);
	//	for (index = index - args.framebuffer_size; index < args.framebuffer_size; index = fchthrd() - args.framebuffer_size)
	//	{
	//		rtm::Hit hit = _lhit(args.hit_records + index);
	//		args.framebuffer[index] = encode_pixel(rtm::vec3(hit.bc.x, hit.bc.y, hit.t));
	//		rtm::Ray ray = args.secondary_rays[index];
	//		rtm::vec3 out = 0.0f;
	//		if (hit.t < T_MAX)
	//		{
	//			rtm::vec3 n = args.triangles[hit.id].normal();
	//			out = n * 0.5f + 0.5f;
	//			args.framebuffer[index] = encode_pixel(out);
	//		}
	//	}
	//}
	//else
	{
		for (index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
		{
			uint x = index % args.framebuffer_width;
			uint y = index / args.framebuffer_width;

			WorkItem wi;
			wi.bray.ray = args.camera.generate_ray_through_pixel(x, y);
			wi.bray.id = index;
			wi.segment_id = 0;
			wi.order_hint = 0;
			_swi(wi); //write root ray to ray bucket
		}
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
		rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y);
		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);

		intersect(args.treelets, ray, hit);
#endif
		rtm::vec3 out = 0;
		if (hit.id != ~0u)
		{
			rtm::vec3 n = args.triangles[hit.id].normal();
			out = n * 0.5f + 0.5f;
		}
		args.framebuffer[index] = encode_pixel(out);

		//uint out = 0xff000000;
		//if (hit.id != ~0u)
		//	out |= rtm::RNG::hash(hit.id);
		//args.framebuffer[index] = out;
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
	args.framebuffer_width = 64;
	args.framebuffer_height = 64;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.samples_per_pixel = 1;
	args.inverse_samples_per_pixel = 1.0f / args.samples_per_pixel;
	args.max_path_depth = 1;

	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(6.78f, -5.87, 0.84), rtm::vec3(7.78f, -5.87, 0.84f));
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));
	args.light_dir = rtm::normalize(rtm::vec3(4.5, 42.5, 5.0));
	//rtm::Mesh mesh("../../datasets/sponza.obj");
	rtm::Mesh mesh("../../datasets/san-miguel.obj");
	rtm::BVH bvh;
	std::vector<rtm::Triangle> tris;
	std::vector<rtm::BVH::BuildObject> build_objects;
	for (uint i = 0; i < mesh.size(); ++i)
		build_objects.push_back(mesh.get_build_object(i));
	bvh.build(build_objects);
	mesh.reorder(build_objects);
	mesh.get_triangles(tris);
	std::cout << tris.size() << '\n';
	rtm::PackedTreeletBVH treelet_bvh(bvh, mesh);
	std::vector<rtm::Ray> ray_buffer(args.framebuffer_size);
	std::vector<rtm::Hit> hit_buffer(args.framebuffer_size);

	args.treelets = treelet_bvh.treelets.data();
	args.triangles = tris.data();
	args.hit_records = hit_buffer.data();

	for (int i = 0; i < args.framebuffer_size; i++) {
		(args.hit_records + i)->t = 1e8;
	}
	args.use_secondary_rays = false;
	uint framebuffer_size = args.framebuffer_size;
	std::vector<rtm::Ray> secondary_rays(framebuffer_size);
	std::vector<rtm::Hit> primary_hits(framebuffer_size);
	if (args.use_secondary_rays == 1)
	{
		std::cout << "generating secondray rays..." << '\n';
		// If the secondary hits already exist in the disk, we don't need to generate it agai
		for (int index = 0; index < framebuffer_size; index++)
		{
			uint32_t x = index % args.framebuffer_width;
			uint32_t y = index / args.framebuffer_width;
			rtm::RNG rng(index);
			rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
			rtm::Hit primary_hit;
			primary_hit.t = ray.t_max;
			primary_hit.id = ~0u;
			intersect(args.treelets, ray, primary_hit);
			primary_hits[index] = primary_hit;
			if (primary_hit.id != ~0u)
			{
				rtm::vec3 normal = tris[primary_hit.id].normal();
				ray.o = ray.o + ray.d * primary_hit.t;
				ray.d = cosine_sample_hemisphere(normal, rng); // generate secondray rays
				ray.t_max = 1;
				secondary_rays[index] = ray;
			}
			else
			{
				ray.t_max = -1;
				secondary_rays[index] = ray;
			}
		}
	}
	args.secondary_rays = secondary_rays.data();
	auto start = std::chrono::high_resolution_clock::now();

#ifdef MULTI_THREADED
	uint thread_count = std::max(std::thread::hardware_concurrency() - 1u, 0u);

	std::vector<std::thread> threads;
	for (uint i = 0; i < thread_count; ++i) threads.emplace_back(kernel, args);

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
