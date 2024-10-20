#pragma once
#include "simulator/simulator.hpp"

#include "units/unit-dram.hpp"
#include "units/unit-dram-ramulator.hpp"
#include "units/unit-blocking-cache.hpp"
#include "units/unit-non-blocking-cache.hpp"
#include "units/unit-buffer.hpp"
#include "units/unit-atomic-reg-file.hpp"
#include "units/unit-tile-scheduler.hpp"
#include "units/unit-sfu.hpp"
#include "units/unit-tp.hpp"

#include "util/elf.hpp"
#include "isa/riscv.hpp"

#include "stdafx.hpp"
#include "rtm/packed-bvh.hpp"
#include "rtm/packed-treelet-bvh.hpp"
#include <Windows.h>

namespace Arches {

template <typename RET>
static RET* write_array(Units::UnitMainMemoryBase* main_memory, size_t alignment, RET* data, size_t size, paddr_t& heap_address)
{
	paddr_t array_address = align_to(alignment, heap_address);
	heap_address = array_address + size * sizeof(RET);
	main_memory->direct_write(data, size * sizeof(RET), array_address);
	return reinterpret_cast<RET*>(array_address);
}

template <typename RET>
static RET* write_vector(Units::UnitMainMemoryBase* main_memory, size_t alignment, std::vector<RET> v, paddr_t& heap_address)
{
	return write_array(main_memory, alignment, v.data(), v.size(), heap_address);
}

template <class T, class L>
inline static L delta_log(L& master_log, std::vector<T*> units)
{
	L delta_log;
	for(uint i = 0; i < units.size(); ++i)
	{
		delta_log.accumulate(units[i]->log);
		units[i]->log.reset();
	}
	master_log.accumulate(delta_log);
	return delta_log;
}

template <class T, class L>
inline static L delta_log(L& master_log, T& unit)
{
	L delta_log = unit.log;
	master_log.accumulate(unit.log);
	unit.log.reset();
	return delta_log;
}

enum SCENES
{
	SPONZA = 0,
	INTEL_SPONZA,
	SAN_MIGUEL,
	HAIRBALL,
	LIVING_ROOM,
	NUMBER
};

std::vector<std::string> scene_names = { "sponza", "intel-sponza", "san-miguel", "hairball", "living_room" };

struct CameraConfig
{
	rtm::vec3 position;
	rtm::vec3 target;
	float focal_length;
};

static const CameraConfig camera_configs[SCENES::NUMBER] =
{
	{rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f), 12.0f}, //CRYTEC SPONZA
	{rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f), 12.0f}, //INTEL SPONZA
	{rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794), 12.0f}, //SAN_MIGUEL
	{rtm::vec3(0, 0, 10), rtm::vec3(0, 0, 0), 24.0f}, //HAIRBALL
	{rtm::vec3(-1.15, 2.13, 7.72), rtm::vec3(-1.15 + 0.3, 2.13 - 0.2, 7.72 - 0.92), 24.0f}, //LIVING_ROOM
};

class GlobalConfig
{
public:
	//simulator config
	uint simulator = 0; //0-trax, 1-dual-streaming
	uint logging_interval = 32 * 1024;

	//workload config
	uint scene_id = 0; 
	uint framebuffer_width = 256;
	uint framebuffer_height = 256;
	CameraConfig camera_config;
	bool pregen_rays = 0;
	uint pregen_bounce = 0; //0-primary, 1-secondary, etc.

	//dual streaming
	bool use_scene_buffer = 0;
	bool rays_on_chip = 0;
	bool hits_on_chip = 1;
	bool use_early = 0;
	bool hit_delay = 0;
	uint hit_buffer_size = 64 * 1024; // number of hits, assuming 128 * 16 * 1024 B = 2MB
	uint traversal_scheme = 1; // 0-BFS, 1-DFS
	uint weight_scheme = 1; // 0 total, 1 average, 2 none

public:
	GlobalConfig(int argc, char* argv[])
	{
		auto ParseCommand = [&](char* argv)
		{
			std::string s(argv);
			size_t pos = s.find("=");
			if(pos == std::string::npos)
			{
				return;
			}
			// -Dxxx=yyy
			auto key = s.substr(2, pos - 2);
			auto value = s.substr(pos + 1, s.size() - (pos + 1));

			if(key == "simulator")
			{
				simulator = std::stoi(value);
			}
			if(key == "scene_name")
			{
				for(int i = 0; i < Arches::scene_names.size(); i++)
				{
					if(Arches::scene_names[i] == value)
					{
						scene_id = i;
					}
				}
			}
			if(key == "framebuffer_width")
			{
				framebuffer_width = std::stoi(value);
			}
			if(key == "framebuffer_height")
			{
				framebuffer_height = std::stoi(value);
			}
			if(key == "traversal_scheme")
			{
				traversal_scheme = std::stoi(value);
			}
			if(key == "hit_buuffer_size")
			{
				hit_buffer_size = std::stoi(value);
			}
			if(key == "use_scene_buffer")
			{
				use_scene_buffer = std::stoi(value);
			}
			if(key == "rays_on_chip")
			{
				rays_on_chip = std::stoi(value);
			}
			if(key == "hits_on_chip")
			{
				hits_on_chip = std::stoi(value);
			}
			if(key == "use_early")
			{
				use_early = std::stoi(value);
			}
			if(key == "hit_delay")
			{
				hit_delay = std::stoi(value);
			}
			if(key == "pregen_rays")
			{
				pregen_rays = std::stoi(value);
			}
			if(key == "pregen_bounce")
			{
				pregen_bounce = std::stoi(value);
			}
			if(key == "weight_scheme")
			{
				weight_scheme = std::stoi(value);
			}
			if(key == "logging_interval")
			{
				logging_interval = std::stoi(value);
			}
			if(key == "buffer")
			{
				if(std::stoi(value) == 0)
					setvbuf(stdout, (char*)NULL, _IONBF, 0);
			}

			std::cout << key << ' ' << value << '\n';
		};

		// 0 is .exe
		for(int i = 1; i < argc; i++)
			ParseCommand(argv[i]);

		camera_config = camera_configs[scene_id];
	}
};
}
