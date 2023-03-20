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
	for(uint32_t index = atomicinc(); index < global_data.framebuffer_size; index = atomicinc())
	{
		uint32_t x = index % global_data.framebuffer_width;
		uint32_t y = index / global_data.framebuffer_width;	

		Ray ray = global_data.camera.generate_ray_through_pixel(x, y);
		Hit hit; hit.t = ray.t_max;

		uint32_t out = 0xffffffff;
		if(intersect(global_data.mesh, ray, hit)) 
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
int main(int argc, char* argv[])
{	
	GlobalData global_data;
	global_data.framebuffer_width = 1024;
	global_data.framebuffer_height = 1024;
	global_data.framebuffer_size = global_data.framebuffer_width * global_data.framebuffer_height;
	global_data.framebuffer = new uint32_t[global_data.framebuffer_size];

	global_data.camera = Camera(global_data.framebuffer_width, global_data.framebuffer_height, 35.0f, rtm::vec3(0.0f, 0.0f, 6.0f));

	Mesh mesh(std::string(argv[1]) + ".obj");
	BVH mesh_blas;
	std::vector<BuildObject> build_objects;
	for(uint i = 0; i < mesh.size(); ++i)
		build_objects.push_back(mesh.get_build_object(i));
	mesh_blas.build(build_objects);
	mesh.reorder(build_objects);

	global_data.mesh.blas = mesh_blas.nodes.data();
	global_data.mesh.vertex_indices = mesh.vertex_indices.data();
	global_data.mesh.vertices = mesh.vertices.data();

	auto start = std::chrono::high_resolution_clock::now();
	ray_caster(global_data);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	std::cout << "Runtime: " << duration.count() << " ms\n\n";

	dump_framebuffer(global_data.framebuffer, "./out.ppm", global_data.framebuffer_width, global_data.framebuffer_height);
	delete[] global_data.framebuffer;

	return 0;
}
#endif
