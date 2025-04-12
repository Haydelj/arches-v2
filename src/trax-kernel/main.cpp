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
	constexpr uint TILE_Y = 8;
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

		#if defined(__riscv) && (TRAX_USE_RT_CORE)
			_traceray<0x0u>(index, ray, hit);
		#else
			//intersect(args.nodes, args.dgf_blocks, ray, hit, node_steps, prim_steps);
			//intersect(args.nodes, args.qt_blocks, ray, hit, node_steps, prim_steps);
			intersect(args.nodes, args.ft_blocks, ray, hit, node_steps, prim_steps);
		#endif

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

	printf("Nodes: %.2f\n", (float)node_steps / args.framebuffer_size);
	printf("Prims: %.2f\n", (float)prim_steps / args.framebuffer_size);
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
	uint pregen_bounce = 0;
	std::string scene_name = "san-miguel";

	TRaXKernelArgs args;
	args.framebuffer_width = 1024;
	args.framebuffer_height = 1024;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.pregen_rays = true;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	if(scene_name.compare("sibenik") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(3.0, -13.0, 0.0), rtm::vec3(0, -12.0, 0));
	if(scene_name.compare("crytek-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	if(scene_name.compare("intel-sponza") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	if(scene_name.compare("san-miguel") == 0)
		args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));

	std::vector<rtm::BVH2::BuildObject> bos;
	rtm::Mesh mesh("../../../datasets/" + scene_name + ".obj");
	mesh.quantize_verts();
	mesh.get_build_objects(bos);
	rtm::BVH2 bvh2("../../../datasets/cache/" + scene_name + ".bvh", bos, 2);
	mesh.reorder(bos);
	printf("\n");

	std::vector<rtm::BVH2::BuildObject> dgf_bos;
	rtm::DGFMesh dgf_mesh("../../../datasets/" + scene_name + ".dgf");
	dgf_mesh.get_build_objects(dgf_bos);
	rtm::BVH2 dgf_bvh2("../../../datasets/cache/" + scene_name + ".dgf.bvh", dgf_bos, 2);
	dgf_mesh.reorder(dgf_bos);
	printf("\n");

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
	{
		std::string ray_file = scene_name + "-" + std::to_string(args.framebuffer_width) + "-" + std::to_string(pregen_bounce) + ".rays";
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, bvh2, mesh, pregen_bounce, rays);
		args.rays = rays.data();
	}
	printf("\n");

#if USE_HEBVH
	//rtm::WBVH dgf_wbvh(dgf_bvh2, dgf_bos);
	//dgf_mesh.reorder(dgf_bos);
	//rtm::HECWBVH hecwbvh(dgf_wbvh, (uint8_t*)dgf_mesh.blocks.data(), sizeof(rtm::DGFMesh::Block));
	//args.nodes = hecwbvh.nodes.data();
	//args.dgf_blocks = (rtm::DGFMesh::Block*)args.nodes;

	//rtm::WBVH wbvh(bvh2, bos, &mesh, false);
	//mesh.reorder(bos);
	//rtm::HECWBVH hecwbvh(wbvh, (uint8_t*)wbvh.qt_blocks.data(), sizeof(rtm::QTB));
	//args.nodes = hecwbvh.nodes.data();
	//args.qt_blocks = (rtm::QTB*)args.nodes;

	rtm::WBVH wbvh(bvh2, bos, &mesh, false);
	mesh.reorder(bos);
	rtm::HECWBVH hecwbvh(wbvh, (uint8_t*)wbvh.ft_blocks.data(), sizeof(rtm::FTB));
	args.nodes = hecwbvh.nodes.data();
	args.ft_blocks = (rtm::FTB*)args.nodes;
#else
	//rtm::NVCWBVH cwbvh(dgf_wbvh);
	//args.nodes = cwbvh.nodes.data();
	//args.dgf_blocks = dgf_mesh.blocks.data();

	rtm::WBVH dgf_wbvh(dgf_bvh2, dgf_bos);
	dgf_mesh.reorder(dgf_bos);
	rtm::NVCWBVH cwbvh(dgf_wbvh);
	args.nodes = cwbvh.nodes.data();
	args.dgf_blocks = dgf_mesh.blocks.data();
#endif

	printf("\nStarting Taveral\n");
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = 0;
	//uint thread_count = std::max(std::thread::hardware_concurrency() - 1u, 1u);
	for (uint i = 1; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint i = 1; i < thread_count; ++i) threads[i].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("Runtime: %dms\n", (uint)duration.count());

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
