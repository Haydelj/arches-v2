#pragma once
#include "simulator/simulator.hpp"

#include "units/unit-dram.hpp"
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

#include "units/unit-rt-core.hpp"

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

enum SCENES
{
	SPONZA = 0,
	SAN_MIGUEL,
	HAIRBALL,
	LIVING_ROOM,
	NUMBER
};
std::vector<std::string> scene_names = { "sponza", "san-miguel", "hairball", "living_room" };
struct SceneConfig
{
	rtm::Camera camera;
}scene_configs[SCENES::NUMBER];
struct GlobalConfig
{
	uint simulator = 1; // 0 - trax, 1-dual-streaming
	uint scene_id = 0;
	uint framebuffer_width = 256;
	uint framebuffer_height = 256;
	uint traversal_scheme = 1; // 0 - BFS, 1 - DFS
	uint hit_buffer_size = 128 * 1024; // number of hits, assuming 128 * 16 * 1024 B = 2MB
	bool use_early = 1;
	bool hit_delay = 0;
	bool use_secondary_rays = false; // 0-primary ray, 1-secondary ray
	uint valid_secondary_rays = 0;
	uint weight_scheme = 1;
	SceneConfig scene_config;
};

bool readCmd = true;
std::vector<rtm::Ray> secondary_rays;
std::vector<rtm::Hit> primary_hits;
}
