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

inline static uint f_to_u(float f)
{
	return *(uint*)&f;
}

inline static float u_to_f(uint u)
{
	return *(float*)&u;
}

inline static void kernel(const TRaXKernelArgs& args)
{
	constexpr uint TILE_X = 32;
	constexpr uint TILE_Y = 1;
	constexpr uint TILE_SIZE = TILE_X * TILE_Y;

	
	uint node_steps = 0, prim_steps = 0;
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint tile_id = index / TILE_SIZE;
		//tile_id = rtm::RNG::fast_hash(tile_id) % (args.framebuffer_size / TILE_SIZE);
		uint32_t tile_x = tile_id % (args.framebuffer_width / TILE_X);
		uint32_t tile_y = tile_id / (args.framebuffer_width / TILE_X);
		uint thread_id = index % TILE_SIZE;
		uint32_t x = tile_x * TILE_X + thread_id % TILE_X;
		uint32_t y = tile_y * TILE_Y + thread_id / TILE_X;
		uint fb_index = y * args.framebuffer_width + x;
		
		//uint fb_index = index, x, y;
		//deinterleave_bits(fb_index, x, y);

		rtm::RNG rng(fb_index);
		rtm::Ray ray = args.pregen_rays ? args.rays[fb_index] : args.camera.generate_ray_through_pixel(x, y);

		//ray.t_min = u_to_f((f_to_u(ray.t_min) & 0xffff0000) | (tile_id & 0xffff));

		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);

		for (uint depth = 0; depth < args.bounce + 1; ++depth)
		{
			hit = rtm::Hit(ray.t_max, rtm::vec2(0.0f), ~0u);
#if defined(__riscv) && (TRAX_USE_RT_CORE)
			_traceray<0x0u>(index, ray, hit);
#else
			intersect(args.nodes, args.tris, ray, hit, fb_index);
			//intersect(args.nodes, (rtm::HECWBVH::Strip*)args.nodes, ray, hit, node_steps, prim_steps);
#endif
			if (hit.id != ~0u)
			{
				rtm::vec3 normal = args.tris[hit.id].normal();
				//ray.d = cosine_sample_hemisphere(normal, rng);

				float costheta = dot(normal, ray.d);
				if (costheta > 0.0f)
				{
					normal = -normal;
				}
				ray.o += ray.d * hit.t + (normal * 0.001f);
				ray.d = randomReflection(normal, rng);


				//ray.d = rtm::vec3(rtm::sqrt(1.0f));
				//ray.d = rtm::cross(normal, normal);
				/*ray.d = rtm::vec3(normal[1] * normal[2] - normal[2] * normal[1],
					(-(normal[0] * normal[2] - normal[2] * normal[0])),
					normal[0] * normal[1] - normal[1] * normal[0]);*/
				//ray.d = rtm::vec3(normal[0]);
			}
			else
				break;
		}

		if(hit.id != ~0u)
		{
			args.framebuffer[fb_index] = rtm::RNG::hash(hit.id) | 0xff000000;
			//args.framebuffer[fb_index] = hit.id | 0xff000000;
			//args.framebuffer[fb_index] = encode_pixel(args.tris[hit.id].normal() * 0.5f + 0.5f);
		}
		else
		{
			args.framebuffer[fb_index] = 0xff000000;
		}

		//args.framebuffer[fb_index] = encode_pixel(steps / 64.0f);
	}

	//printf("Nodes: %.2f\n", (float)node_steps / args.framebuffer_size);
	//printf("Prims: %.2f\n", (float)prim_steps / args.framebuffer_size);
}

inline static void mandelbrot(const TRaXKernelArgs& args)
{
	constexpr uint MAX_ITERS = 512;
	for(uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		//rtm::Ray ray = args.rays[index];

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
		/*uint s = 0;
		for (int j = 0; j < 512; j++)
		{
			uint offset = (j * index + j + index) * index;
			uint addr = (index + offset) % args.framebuffer_size;
			uint a = args.framebuffer[addr];
			s = s + a;
		}
		args.framebuffer[index] = s;*/
	}
}

#ifdef __riscv 
int main()
{
	kernel(*(const TRaXKernelArgs*)TRAX_KERNEL_ARGS_ADDRESS);
	//mandelbrot(*(const TRaXKernelArgs*)TRAX_KERNEL_ARGS_ADDRESS);
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
	args.framebuffer_width = 128;
	args.framebuffer_height = 128;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.pregen_rays = false;
	args.bounce = 1;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	std::string scene_name = "intel-sponza";

	if(scene_name.compare("crytek-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));

	//intel sponza camera
	if(scene_name.compare("intel-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	
	// san miguel camera
	if(scene_name.compare("san-miguel") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));

	// sibenik
	if (scene_name.compare("sibenik") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(3.0, -13.0, 0.0), rtm::vec3(0, -12.0, 0));

	// hairball camera
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(0, 10, 10), rtm::vec3(10, 0, 0));

	//args.camera = Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(0.0f, 0.0f, 5.0f));

	rtm::Mesh mesh("../../../datasets/" + scene_name + ".obj");
	//args.camera._position *= mesh.normalize_verts();
	//mesh.quantize_verts();

#if TRAX_USE_SIM_TRAX_BVH
	std::vector<rtm::BVHSIMTRAX::BuildObject> build_objects;
#else
	 std::vector<rtm::BVH2::BuildObject> build_objects;
#endif
	mesh.get_build_objects(build_objects);

	//rtm::BVH2 bvh2("../../../datasets/cache/" + scene_name + ".bvh", build_objects, 2);
#if TRAX_USE_SIM_TRAX_BVH
	rtm::BVHSIMTRAX bvh2(build_objects, 2);
	args.nodes = bvh2.build_nodes;
#else
	rtm::BVH2 bvh2(build_objects, 2);
	args.nodes = bvh2.nodes.data();
#endif

	//rtm::WBVH wbvh(bvh2, build_objects);
	//args.nodes = wbvh.nodes.data();

	mesh.reorder(build_objects);

	//{
	//	std::vector<rtm::TriangleStrip> temp_strips(strips);
	//	for(uint i = 0; i < build_objects.size(); ++i)
	//	{
	//		strips[i] = temp_strips[build_objects[i].index];
	//		build_objects[i].index = i;
	//	}
	//}
	//args.strips = strips.data();

	//rtm::NVCWBVH cwbvh(wbvh);
	//args.nodes = cwbvh.nodes.data();

	//rtm::HECWBVH hecwbvh(wbvh, strips);
	//args.nodes = hecwbvh.nodes.data();

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = tris.data();

	uint bounce = 0;
	std::string ray_file = scene_name + "-" + std::to_string(args.framebuffer_width) + "-" + std::to_string(bounce) + ".rays";

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, args.nodes, args.tris, args.tris, bounce, rays, ray_file);
	args.rays = rays.data();
	
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = 1;
	//uint thread_count = std::max(std::thread::hardware_concurrency() - 1u, 1u);
	for (uint i = 1; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint i = 1; i < thread_count; ++i) threads[i].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
