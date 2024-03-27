#include "stdafx.hpp"

#include "trax.hpp"
#include "dual-streaming.hpp"
#include "shared-utils.hpp"
#include "trax-kernel/include.hpp"

//global verbosity flag
int arches_verbosity = 1;

Arches::GlobalConfig parse_config(int argc, char* argv[])
{
	// parse configs for simulators (trax, dual-streaming, etc.)
	Arches::GlobalConfig global_config;
	std::cout << argc << '\n';
	auto ParseCommand = [&](char* argv)
	{
		std::string s(argv);
		size_t pos = s.find("=");
		if (pos == std::string::npos)
		{
			return;
		}
		// -Dxxx=yyy
		auto key = s.substr(2, pos - 2);
		auto value = s.substr(pos + 1, s.size() - (pos + 1));

		if (key == "simulator")
		{
			global_config.simulator = std::stoi(value);
		}
		if (key == "scene_name")
		{
			for (int i = 0; i < Arches::scene_names.size(); i++)
			{
				if (Arches::scene_names[i] == value)
				{
					global_config.scene_id = i;
				}
			}
		}
		if (key == "framebuffer_width")
		{
			global_config.framebuffer_width = std::stoi(value);
		}
		if (key == "framebuffer_height")
		{
			global_config.framebuffer_height = std::stoi(value);
		}
		if (key == "traversal_scheme")
		{
			global_config.traversal_scheme = std::stoi(value);
		}
		if (key == "hit_buuffer_size")
		{
			global_config.hit_buffer_size = std::stoi(value);
		}
		if (key == "use_early")
		{
			global_config.use_early = std::stoi(value);
		}
		if (key == "hit_delay")
		{
			global_config.hit_delay = std::stoi(value);
		}
		if (key == "secondary_rays")
		{
			global_config.use_secondary_rays = std::stoi(value);
		}
		if (key == "weight_scheme")
		{
			global_config.weight_scheme = std::stoi(value);
		}
		std::cout << key << ' ' << value << '\n';
	};

	// 0 is .exe
	for (int i = 1; i < argc; i++)
	{
		if (Arches::readCmd) ParseCommand(argv[i]);
	}
	Arches::scene_configs[Arches::SCENES::SPONZA].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	Arches::scene_configs[Arches::SCENES::SAN_MIGUEL].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0f, rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794));
	Arches::scene_configs[Arches::SCENES::HAIRBALL].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0, rtm::vec3(0, 0, 10), rtm::vec3(0, 0, -1));
	Arches::scene_configs[Arches::SCENES::LIVING_ROOM].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0, rtm::vec3(-1.15, 2.13, 7.72), rtm::vec3(-1.15 + 0.3, 2.13 - 0.2, 7.72 - 0.92));
	global_config.scene_config = Arches::scene_configs[global_config.scene_id];
	return global_config;
}

int main(int argc, char* argv[])
{
	Arches::GlobalConfig global_config = parse_config(argc, argv);

	if (global_config.use_secondary_rays == 1)
	{
		std::cout << "generating secondray rays..." << '\n';

		uint framebuffer_size = global_config.framebuffer_height * global_config.framebuffer_width;
		// we need to pre-generate secondary rays and feed them into the simulator
		Arches::secondary_rays.resize(framebuffer_size);
		Arches::primary_hits.resize(framebuffer_size);
		// If the secondary hits already exist in the disk, we don't need to generate it again

		// build BVH
		std::string s = Arches::scene_names[global_config.scene_id];
		TCHAR exePath[MAX_PATH];
		GetModuleFileName(NULL, exePath, MAX_PATH);
		std::wstring fullPath(exePath);
		std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
		std::string current_folder_path(exeFolder.begin(), exeFolder.end());
		std::string filename = current_folder_path + "../../datasets/" + s + ".obj";
		rtm::Mesh mesh(filename);
		std::vector<rtm::BVH::BuildObject> build_objects;
		for (uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));
		rtm::BVH blas;
		std::vector<rtm::Triangle> tris;
		blas.build(build_objects);
		mesh.reorder(build_objects);
		mesh.get_triangles(tris);
		MeshPointers mesh_pointers;
		mesh_pointers.blas = blas.nodes.data();
		mesh_pointers.tris = tris.data();
		for (int index = 0; index < framebuffer_size; index++)
		{
			uint32_t x = index % global_config.framebuffer_width;
			uint32_t y = index / global_config.framebuffer_width;
			rtm::RNG rng(index);
			rtm::Ray ray = global_config.scene_config.camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
			rtm::Hit primary_hit;
			primary_hit.t = ray.t_max;
			primary_hit.id = ~0u;
			intersect(mesh_pointers, ray, primary_hit);
			Arches::primary_hits[index] = primary_hit;
			if (primary_hit.id != ~0u)
			{
				rtm::vec3 normal = tris[primary_hit.id].normal();
				ray.o = ray.o + ray.d * primary_hit.t;
				ray.d = cosine_sample_hemisphere(normal, rng); // generate secondray rays
				ray.t_max = T_MAX;
				Arches::secondary_rays[index] = ray;
				global_config.valid_secondary_rays++;
			}
			else
			{
				ray.t_max = -1;
				Arches::secondary_rays[index] = ray;
			}
		}
		std::cout << "Secondary rays generated. Total valid rays: " << global_config.valid_secondary_rays << '\n';
	}

	if (global_config.simulator == 0)
	{
		Arches::TRaX::run_sim_trax(global_config);
	}
	else if (global_config.simulator == 1)
	{
		Arches::DualStreaming::run_sim_dual_streaming(global_config);
	}
	
	return 0;
}