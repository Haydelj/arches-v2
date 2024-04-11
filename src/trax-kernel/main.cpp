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
#if 1
	//if (args.use_secondary_rays)
	//{
	//	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	//	{
	//		uint32_t x = index % args.framebuffer_width;
	//		uint32_t y = index / args.framebuffer_width;
	//		rtm::Ray ray = args.secondary_rays[index];
	//		if (ray.t_max > 0)
	//		{
	//			rtm::Hit hit;
	//			hit.id = ~0u;
	//			hit.t = ray.t_max;
	//			intersect(args.mesh, ray, hit);
	//			if (hit.id != ~0u)
	//			{
	//				rtm::vec3 normal = args.mesh.tris[hit.id].normal();
	//				rtm::vec3 output = normal * 0.5 + 0.5;
	//				args.framebuffer[index] = encode_pixel(output);
	//			}
	//		}
	//	}
	//}
//	else 
	{
		for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
		{
			uint32_t x = index % args.framebuffer_width;
			uint32_t y = index / args.framebuffer_width;

			printf("%d\r", y * args.framebuffer_width + x);

			rtm::RNG rng(index);
			rtm::vec3 output(0.0f);

			for (uint i = 0; i < args.samples_per_pixel; ++i)
			{
				rtm::Ray ray; rtm::Hit hit; rtm::vec3 normal;

				if (args.samples_per_pixel > 1)  ray = args.camera.generate_ray_through_pixel(x, y, &rng);
				else                            ray = args.camera.generate_ray_through_pixel(x, y);

				rtm::vec3 attenuation(1.0f);
				for (uint j = 0; j < args.max_depth; ++j)
				{
					if (j != 0)
					{
						ray.o = ray.o + ray.d * hit.t;
						ray.d = cosine_sample_hemisphere(normal, rng);
						hit.t = ray.t_max;
						attenuation *= 0.8f;
					}

					hit.t = ray.t_max; hit.id = ~0u;
				#if defined(__riscv) &&  defined(USE_RT_CORE)
					_traceray<0x0u>(index, ray, hit);
				#else
					#if defined(WIDE_BVH)
						intersect(args.nodes, args.indices, args.tris, ray, hit);
					#else
						intersect(args.nodes, args.tris, ray, hit);
					#endif
				#endif

					if(hit.id != ~0u)
					{
						
						normal = args.tris[hit.id].normal();
						normal = normal * 0.5f + 0.5f;
						output = normal;
						break;
						float ndotl = rtm::max(0.0f, rtm::dot(normal, args.light_dir));
						if(ndotl > 0.0f)
						{
							rtm::Ray sray = ray;
							sray.o = ray.o + ray.d * hit.t;
							sray.d = args.light_dir;
							rtm::Hit shit;
							shit.t = sray.t_max; shit.id = ~0u;
						#if defined(__riscv) &&  defined(USE_RT_CORE)
							_traceray<0x1u>(index, sray, shit);
						#else
							#if defined(WIDE_BVH)
								intersect(args.nodes, args.indices,args.tris, sray, shit);
							#else
								intersect(args.nodes, args.tris, sray, shit);
							#endif
						#endif
							if(shit.id != ~0u)
								ndotl = 0.0f;
						}
						output += attenuation * ndotl * 0.8f * rtm::vec3(1.0f, 0.9f, 0.8f);
						

					//	output += rtm::vec3(hit.id, hit.id, hit.id);
					}
					else
					{
						output += attenuation * rtm::vec3(0.5f, 0.7f, 0.9f);
					}
				}
				break;
			}

			args.framebuffer[index] = encode_pixel(output * (1.0f / args.samples_per_pixel));
		}

		printf("\n");
	}

#else
	for (uint index = fchthrd(); index < args.framebuffer_size; index = fchthrd())
	{
		uint32_t x = index % args.framebuffer_width;
		uint32_t y = index / args.framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y);

		rtm::Hit hit; hit.t = ray.t_max; hit.id = ~0u;
	#if defined(__riscv) &&  defined(USE_RT_CORE)
		_traceray<0x0u>(index, ray, hit);
	#else
		intersect(args.nodes, args.tris, ray, hit);
	#endif
		if(hit.id != ~0u)
		{
			args.framebuffer[index] = rtm::RNG::hash(hit.id) | 0xff000000;
		}
		else
		{
			args.framebuffer[index] = 0xff000000;
		}
	}
#endif
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

	args.samples_per_pixel = 1;
	args.max_depth = 1;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	//args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 24.0f, rtm::vec3(0.0f, 0.0f, 5.0f));

	args.use_secondary_rays = false;
	uint framebuffer_size = args.framebuffer_size;
	std::vector<rtm::Ray> secondary_rays(framebuffer_size);
	std::vector<rtm::Hit> primary_hits(framebuffer_size);
	rtm::Mesh mesh("../../datasets/sponza.obj");
	rtm::BVH bvh;
	rtm::WideBVH wbvh;

	std::vector<rtm::Triangle> tris;
	std::vector<rtm::BVH::BuildObject> build_objects;
	for (uint i = 0; i < mesh.size(); ++i)
		build_objects.push_back(mesh.get_build_object(i));
	bvh.build(build_objects);
	wbvh.build(bvh);
	
	mesh.reorder(build_objects);
	mesh.reorder(wbvh.indices);
	mesh.get_triangles(tris);
	rtm::PackedBVH2 packed_bvh(bvh);
	rtm::PackedTreeletBVH treelet_bvh(packed_bvh, mesh);

#if defined(WIDE_BVH)
	args.nodes = wbvh.getNodes();
	args.indices = wbvh.getIndices();
#else
	args.nodes = packed_bvh.nodes.data();
#endif
	
	args.tris = tris.data();
	args.treelets = treelet_bvh.treelets.data();
	if (args.use_secondary_rays == 1)
	{
		std::cout << "generating secondray rays..." << '\n';
		// If the secondary hits already exist in the disk, we don't need to generate it 
		// Considering it's running on CPU, it's acceptible

		for (int index = 0; index < framebuffer_size; index++)
		{
			uint32_t x = index % args.framebuffer_width;
			uint32_t y = index / args.framebuffer_width;
			rtm::RNG rng(index);
			rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
			rtm::Hit primary_hit;
			primary_hit.t = ray.t_max;
			primary_hit.id = ~0u;

#if defined(WIDE_BVH)
			intersect(args.nodes, args.indices, args.tris, ray, primary_hit);
#else
			intersect(args.nodes, args.tris, ray, primary_hit);
#endif
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

	std::cout << "Launching TraX kernel now..." << std::endl;
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<std::thread> threads;
	uint thread_count = std::max(std::thread::hardware_concurrency() - 2u, 0u);
	//uint thread_count = 1;

	for (uint i = 0; i < thread_count; ++i) threads.emplace_back(kernel, args);
	kernel(args);
	for (uint i = 0; i < thread_count; ++i) threads[i].join();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Kernel Execution Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", args.framebuffer_width, args.framebuffer_height, 4, args.framebuffer, 0);

	delete[] args.framebuffer;
	return 0;
}
#endif
