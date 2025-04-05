#include "stdafx.hpp"

#include "include.hpp"
#include "intersect.hpp"
#include "custom-instr.hpp"
#include <cstdio>

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

inline static void kernel(const STRaTARTKernel::Args& args)
{
#if defined(__riscv)
    uint32_t index, x, y;
    for (index = fchthrd(); index < args.framebuffer_size + args.max_init_ray; index = fchthrd())
    {
        if(index >= args.max_init_ray)
        {
			STRaTARTKernel::HitReturn hit_return = _lhit(index < args.framebuffer_size ? 0x0ull : 0x10ull);
            uint32_t out = 0xff000000;
            if (hit_return.hit.id != ~0u)
            {
                out |= rtm::RNG::hash(hit_return.hit.id);
            }
            args.framebuffer[hit_return.index] = out;
        }
        // trace a new primary ray
        if(index < args.framebuffer_size)
        {
            x = index % args.framebuffer_width;
            y = index / args.framebuffer_width;
            rtm::Ray ray = args.pregen_rays ? args.rays[index] : args.camera.generate_ray_through_pixel(x, y);
			rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
			STRaTARTKernel::RayData raydata;
            raydata.ray = ray;
			raydata.hit = hit;
            raydata.global_ray_id = index;
            raydata.treelet_id = 0;
			raydata.level = 0;
			raydata.restart_trail = rtm::RestartTrail();
			if(ray.t_min == ray.t_max) raydata.restart_trail.mark_done();
            _swi(raydata);
        }
    } 
#else
	for (uint32_t index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint32_t x = index % args.framebuffer_width;
		uint32_t y = index / args.framebuffer_width;

		uint32_t steps = 0;
		rtm::Ray ray = args.pregen_rays ? args.rays[index] : args.camera.generate_ray_through_pixel(x, y);
		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
		intersect(args.treelets, ray, hit, steps);
		uint32_t out = 0xff000000;
		if (hit.id != ~0u)
			out |= rtm::RNG::hash(hit.id);
		args.framebuffer[index] = out;
	}
#endif
}

#ifdef __riscv 
int main()
{
	kernel(*(const STRaTARTKernel::Args*)KERNEL_ARGS_ADDRESS);
	return 0;
}

#else
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"

int main(int argc, char* argv[])
{
	STRaTARTKernel::Args args;
	args.framebuffer_width = 1024;
	args.framebuffer_height = 1024;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];
	args.raybuffer_size = 4 * 1024 * 1024;	//default 4MB

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
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, bvh2, mesh, 1, rays);
	args.rays = rays.data();

#ifdef USE_COMPRESSED_WIDE_BVH
	rtm::WBVH wbvh(bvh2, build_objects);
	mesh.reorder(build_objects);
	rtm::NVCWBVH cwbvh(wbvh);
	rtm::CompressedWideTreeletBVH cwtbvh(cwbvh, mesh);
	args.treelets = cwtbvh.treelets.data();
#else
	rtm::WideBVHSTRaTA packed_bvh2(bvh2, build_objects);
	rtm::WideTreeletBVHSTRaTA treelet_bvh(packed_bvh2, mesh);
	args.treelets = treelet_bvh.treelets.data();
#endif

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = tris.data();
	std::vector<rtm::Hit> hit_buffer(args.framebuffer_size);
	args.hit_records = hit_buffer.data();

	for (int i = 0; i < args.framebuffer_size; i++)
		args.hit_records[i].t = T_MAX;
	
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint32_t thread_count = 1; // std::max(std::thread::hardware_concurrency() - 1u, 1u);
	for (uint32_t i = 0; i < thread_count - 1; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint32_t i = 0; i < thread_count - 1; ++i) threads[i].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
