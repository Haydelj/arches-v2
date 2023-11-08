#include "stdafx.hpp"

#include "custom-instr.hpp"
#include "ray-tracing-include.hpp"

//static uint64_t encode_pixel(rtm::vec3 in)
//{
//	in = rtm::clamp(in, 0.0f, 1.0f);
//	uint64_t out = 0u;
//	out |= static_cast<uint64_t>(in.r * 65535.0f + 0.5f) << 0;
//	out |= static_cast<uint64_t>(in.g * 65535.0f + 0.5f) << 16;
//	out |= static_cast<uint64_t>(in.b * 65535.0f + 0.5f) << 32;
//	out |= 0xffff                                        << 48;
//	return out;
//}
static uint32_t encode_pixel(rtm::vec3 in)
{
	in = rtm::clamp(in, 0.0f, 1.0f);
	uint32_t out = 0u;
	out |= static_cast<uint32_t>(in.r * 255.0f + 0.5f) << 0;
	out |= static_cast<uint32_t>(in.g * 255.0f + 0.5f) << 8;
	out |= static_cast<uint32_t>(in.b * 255.0f + 0.5f) << 16;
	out |= 0xff                                        << 24;
	return out;
}

static void fix_normal(const glm::vec3& d, glm::vec3& n)
{
	constexpr float EPS_NORMAL_CORRECTION = 0.01f;
	float dot = glm::dot(d, n);
	if(dot <= -EPS_NORMAL_CORRECTION)
	{
	}
	else
	{
		//shading normal is close to or below the surface.
		glm::vec3 tmp = n - glm::dot(d, n) * d;
		float numer = fast_sqrtf(1.0f - EPS_NORMAL_CORRECTION * EPS_NORMAL_CORRECTION);
		float denom = fast_sqrtf(glm::dot(tmp, tmp));
		n = EPS_NORMAL_CORRECTION * d;
		n += (numer * denom) * tmp;
		n = glm::normalize(n);
	}
}

void static inline path_tracer(const GlobalData& global_data)
{
	for(uint32_t index = fchthrd(); index < global_data.framebuffer_size; index = fchthrd())
	{
		uint32_t x = index % global_data.framebuffer_width;
		uint32_t y = index / global_data.framebuffer_width;	
		RNG rng(index);

		uint samples = 1024;
		rtm::vec3 out(0.0f);
		for(uint j = 0; j < samples; ++j)
		{
			Hit hit;
			Ray ray = global_data.camera.generate_ray_through_pixel(x, y, &rng);
			ray.d = rtm::normalize(ray.d);
			ray.rcp_max_error = 1.0f;

			Ray last_ray = ray;
			uint last_patch_index = ~0u;

			rtm::vec3 attenuation(1.0f);
			for(uint i = 0; true; ++i)
			{
				 hit.t = ray.t_max;
				if(intersect(global_data, last_patch_index, last_ray, ray, hit))
				{
					if(i >= global_data.bounces) break;

					rtm::uvec3 ni = global_data.ni[hit.id];
					rtm::vec3 n = rtm::normalize(global_data.normals[ni[0]] * hit.bc[0] + global_data.normals[ni[1]] * hit.bc[1] + global_data.normals[ni[2]] * (1.0f - hit.bc[0] - hit.bc[1])); 
					fix_normal(ray.d, n);

					last_ray = ray;
					last_patch_index = hit.patch_index;

					ray.o += ray.d * hit.t;
					ray.d = cosine_sample_hemisphere(n, rng);

					ray.radius += ray.drdt * hit.t;
					ray.drdt = fast_sqrtf(3.1415926f * 2.0f / (glm::dot(ray.d, n) * samples));

					attenuation *= 0.8f;
				}
				else
				{
					out += attenuation * 1.0f;
					break;
				}
			}
		}

		out /= samples;
		#ifdef ARCH_X86
		out = rtm::pow(out, 1.0f / 2.2f);
		#endif
		global_data.framebuffer[index] = encode_pixel(out);
	}
}

#ifdef ARCH_RISCV
//gcc will only set main as entry point so we'll do this for now but we could theortically use path_tracer as entry
int main()
{
	path_tracer(*(GlobalData*)GLOBAL_DATA_ADDRESS);
	return 0;
}
#endif

#ifdef ARCH_X86
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"

int main(int argc, char* argv[])
{	
	GlobalData global_data;
	global_data.framebuffer_width = 1024;
	global_data.framebuffer_height = 1024;
	global_data.framebuffer_size = global_data.framebuffer_width * global_data.framebuffer_height;
	global_data.framebuffer = new uint32_t[global_data.framebuffer_size];

	global_data.camera = Camera(global_data.framebuffer_width, global_data.framebuffer_height, 35.0f, rtm::vec3(0.0f, 0.0f, 1.5f));

	Mesh mesh(std::string(argv[2]) + ".obj");
	TesselationTree1 tt1(std::string(argv[2]) + ".tt1");
	TesselationTree4 tt4(std::string(argv[2]) + ".tt4");
	
	BVH blas;
	std::vector<BVH::CompressedNode4> cblas;

	global_data.config = atoi(argv[1]);
	printf("Config: %d\n", global_data.config);

	if(global_data.config == 0) //BVHC
	{
		std::vector<BuildObject> build_objects;
		for(uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));

		blas.build(build_objects);
		mesh.reorder(build_objects);
		blas.compress_and_pack(cblas);

		global_data.cblas = cblas.data();
		global_data.mesh.vertex_indices = mesh.vertex_indices.data();
		global_data.mesh.vertices = mesh.vertices.data();
	}
	else if(global_data.config == 1) //TTC
	{
		std::vector<BuildObject> build_objects;
		for(uint i = 0; i < tt4.size(); ++i)
			build_objects.push_back(tt4.get_build_object(i));

		blas.build(build_objects);
		tt4.reorder(build_objects);
		blas.compress_and_pack(cblas);

		global_data.cblas = cblas.data();
		global_data.tt4.headers = tt4.headers.data();
		global_data.tt4.nodes = tt4.nodes.data();
		global_data.tt4.triangles = tt4.triangles.data();
		global_data.tt4.vertex_indices = tt4.vertex_indices.data();
	}
	else if(global_data.config == 2) //BVH
	{
		std::vector<BuildObject> build_objects;
		for(uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));

		blas.build(build_objects);
		mesh.reorder(build_objects);

		global_data.blas = blas.nodes.data();
		global_data.mesh.vertex_indices = mesh.vertex_indices.data();
		global_data.mesh.vertices = mesh.vertices.data();
	}
	else if(global_data.config == 3) //TT
	{
		std::vector<BuildObject> build_objects;
		for(uint i = 0; i < tt1.size(); ++i)
			build_objects.push_back(tt1.get_build_object(i));

		blas.build(build_objects);
		tt1.reorder(build_objects);

		global_data.blas = blas.nodes.data();
		global_data.tt1.headers = tt1.headers.data();
		global_data.tt1.nodes = tt1.nodes.data();
		global_data.tt1.vertices = tt1.vertices.data();
		global_data.tt1.triangles = tt1.triangles.data();
	}

	global_data.ni = mesh.normal_indices.data();
	global_data.normals = mesh.normals.data();

	global_data.bounces = atoi(argv[3]);

	auto start = std::chrono::high_resolution_clock::now();
	std::vector<std::thread> threads;
	for(uint i = 0; i < std::thread::hardware_concurrency(); ++i) threads.emplace_back(path_tracer, global_data);
	for(uint i = 0; i < std::thread::hardware_concurrency(); ++i) threads[i].join();
	//path_tracer(global_data);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", global_data.framebuffer_width, global_data.framebuffer_height, 4, global_data.framebuffer, 0);

	delete[] global_data.framebuffer;
	return 0;
}
#endif
