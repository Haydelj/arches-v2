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

inline static void kernel(const KernelArgs& args)
{
#if __riscv
	uint index;
	for(index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint x = index % args.framebuffer_width;
		uint y = index / args.framebuffer_width;	
		rtm::RNG rng(index);

		WorkItem wi;
		wi.bray.ray = args.camera.generate_ray_through_pixel(x, y);
		wi.bray.id = index;
		wi.segment = 0;

		//write root ray to ray bucket
	#if 1
		_swi(wi);
	#else

		rtm::Hit hit; hit.t = wi.bray.ray.t_max;
		uint treelet_stack[32]; uint treelet_stack_size = 0;
		if(intersect_treelet(args.treelets[wi.segment], wi.bray.ray, hit, treelet_stack, treelet_stack_size))
		{
			//update hit record with hit using cshit
			_cshit(hit, args.hit_records + index);
			wi.bray.ray.t_max = hit.t;
		}

		//drain treelet stack
		while(treelet_stack_size)
		{
			uint treelet_index = treelet_stack[--treelet_stack_size];
			wi.segment = treelet_index;
			_swi(wi);
		}
	#endif
	}

	intersect_buckets(args);

	//spin sleep
	for(volatile uint i = 0; i < 1024; i++);

	for(index = index - args.framebuffer_size; index < args.framebuffer_size; index = fchthrd() - args.framebuffer_size)
	{
		rtm::Hit hit = _lhit(args.hit_records + index);
		args.framebuffer[index] = encode_pixel(rtm::vec3(hit.bc.x, hit.bc.y, hit.t));

		rtm::vec3 out = 0.0f;
		if(hit.t < T_MAX)
		{
			rtm::vec3 n = args.triangles[hit.id].normal();
			out = n * 0.5f + 0.5f;
		}

		args.framebuffer[index] = encode_pixel(out);
	}
#else
	for(uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint x = index % args.framebuffer_width;
		uint y = index / args.framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y);
		rtm::Hit hit; hit.t = ray.t_max;

		rtm::vec3 out = 0.0f;
		if (intersect(args.treelets, ray, hit))
		{
			rtm::vec3 n = args.triangles[hit.id].normal();
			out = n * 0.5f + 0.5f;
		}

		args.framebuffer[index] = encode_pixel(out);
	}
#endif
}

#ifdef __riscv 
int main()
{
	kernel(*(const KernelArgs*)KERNEL_ARGS_ADDRESS);
	return 0;
}

#else
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"
int main(int argc, char* argv[])
{
	KernelArgs args;
	args.framebuffer_width = 256;
	args.framebuffer_height = 256;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	args.framebuffer = new uint32_t[args.framebuffer_size];

	args.samples_per_pixel = 1;
	args.inverse_samples_per_pixel = 1.0f / args.samples_per_pixel;
	args.max_path_depth = 1;

	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(6.78f, -5.87, 0.84), rtm::vec3(7.78f, -5.87, 0.84f));
	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(24.4, 16.4, 12.8), rtm::vec3(24.4 - 0.3, 16.4 - 0.6, 12.8 - 0.6));
	args.light_dir = rtm::normalize(rtm::vec3(4.5, 42.5, 5.0));
	rtm::Mesh mesh("../../datasets/sponza.obj");
	//rtm::Mesh mesh("../../datasets/san-miguel.obj");
	rtm::BVH bvh;
	std::vector<rtm::Triangle> tris;
	std::vector<rtm::BVH::BuildObject> build_objects;
	for (uint i = 0; i < mesh.size(); ++i)
		build_objects.push_back(mesh.get_build_object(i));
	bvh.build(build_objects);
	mesh.reorder(build_objects);
	mesh.get_triangles(tris);

	std::cout << tris.size() << '\n';
	TreeletBVH treelet_bvh(bvh, mesh);
	std::vector<rtm::Ray> ray_buffer(args.framebuffer_size);
	std::vector<rtm::Hit> hit_buffer(args.framebuffer_size);

	args.treelets = treelet_bvh.treelets.data();
	args.triangles = tris.data();
	args.hit_records = hit_buffer.data();

	for (int i = 0; i < args.framebuffer_size; i++) {
		(args.hit_records + i)->t = 1e8;
	}

	auto start = std::chrono::high_resolution_clock::now();

	/*uint thread_count = std::max(std::thread::hardware_concurrency() - 1u, 1u);
	std::vector<std::thread> threads;
	for (uint i = 0; i < thread_count; ++i)
		threads.emplace_back(kernel, args);
	for (uint i = 0; i < thread_count; ++i)
		threads[i].join();*/

	kernel(args);

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
