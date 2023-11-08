#include "stdafx.hpp"

#include "custom-instr.hpp"
#include "ray-tracing-include.hpp"

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

void static inline ray_caster(const GlobalData& global_data)
{
	for(uint32_t index = fchthrd(); index < global_data.framebuffer_size; index = fchthrd())
	{
		uint32_t x = index % global_data.framebuffer_width;
		uint32_t y = index / global_data.framebuffer_width;	

		Ray ray = global_data.camera.generate_ray_through_pixel(x, y);
		Hit hit; hit.t = ray.t_max;
		ray.rcp_max_error =  4.0f * 0.5f; //1 pixel = 2 radius of error so 1 / 2

		uint32_t out = 0xffffffff;
		if(intersect(global_data, ray, hit)) 
			out = RNG::fast_hash(hit.id) | 0xff000000;

		global_data.framebuffer[index] = out;
	}
}

#ifdef ARCH_RISCV
//gcc will only set main as entry point so we'll do this for now but we could theortically use path_tracer as entry
int main()
{
	ray_caster(*(GlobalData*)GLOBAL_DATA_ADDRESS);
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

	global_data.camera = Camera(global_data.framebuffer_width, global_data.framebuffer_height, 35.0f, rtm::vec3(0.0f, 0.0f, 1.0f));

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
		global_data.tt1.
		triangles = tt1.triangles.data();
	}

	auto start = std::chrono::high_resolution_clock::now();
	ray_caster(global_data);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	stbi_flip_vertically_on_write(true);
	stbi_write_png("./out.png", global_data.framebuffer_width, global_data.framebuffer_height, 4, global_data.framebuffer, 0);

	delete[] global_data.framebuffer;

	return 0;
}
#endif
