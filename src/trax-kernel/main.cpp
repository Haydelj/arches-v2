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
		uint32_t x = index % args.framebuffer_width;
		uint32_t y = index / args.framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = args.pregen_rays ? args.rays[index] : args.camera.generate_ray_through_pixel(x, y);

		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
		if(ray.t_min < ray.t_max)
		{
		#if defined(__riscv) &&  defined(USE_RT_CORE)
			_traceray<0x0u>(index, ray, hit);
		#else
			intersect(args.nodes, args.strips, ray, hit);
		#endif
		}

		if(hit.id != ~0u)
		{
			args.framebuffer[index] = rtm::RNG::hash(hit.id) | 0xff000000;
		}
		else
		{
			args.framebuffer[index] = 0xff000000;
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

	rtm::Mesh mesh("../../datasets/sponza.obj");
	std::vector<rtm::BVH2::BuildObject> build_objects;
	for (uint i = 0; i < mesh.size(); ++i)
		build_objects.push_back(mesh.get_build_object(i));

	rtm::BVH2 bvh2("../../datasets/cache/sponza_bvh.cache", build_objects, 2);
	mesh.reorder(build_objects);

	rtm::PackedBVH2 packed_bvh2(bvh2, mesh);
	rtm::PackedTreeletBVH treelet_bvh(packed_bvh2, mesh);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);

	args.nodes = packed_bvh2.nodes.data();
	args.strips = packed_bvh2.strips.data();
	args.treelets = treelet_bvh.treelets.data();
	args.tris = tris.data();

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args, 1, rays);
	args.rays = rays.data();

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
