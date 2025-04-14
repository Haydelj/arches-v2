#pragma once
#include "stdafx.hpp"

#include "simulator/simulator.hpp"

#include "units/unit-dram.hpp"
#include "units/unit-dram-ramulator.hpp"
#include "units/unit-cache.hpp"
#include "units/unit-crossbar.hpp"
#include "units/unit-buffer.hpp"
#include "units/unit-atomic-reg-file.hpp"
#include "units/unit-tile-scheduler.hpp"
#include "units/unit-sfu.hpp"
#include "units/unit-tp.hpp"

#include "util/elf.hpp"
#include "isa/riscv.hpp"
#include "rtm/rtm.hpp"

#include <Windows.h>

namespace Arches {

std::string get_project_folder_path()
{
	CHAR path[MAX_PATH];
	GetModuleFileNameA(NULL, path, MAX_PATH);
	std::string executable_path(path);
	return executable_path.substr(0, executable_path.rfind("build"));
}

template <typename T>
static T* write_array(Units::UnitMainMemoryBase* main_memory, size_t alignment, T* data, size_t size, paddr_t& heap_address)
{
	paddr_t array_address = align_to(alignment, heap_address);
	heap_address = array_address + size * sizeof(T);
	main_memory->direct_write(data, size * sizeof(T), array_address);
	return reinterpret_cast<T*>(array_address);
}

template <typename T>
static T* write_vector(Units::UnitMainMemoryBase* main_memory, size_t alignment, std::vector<T> v, paddr_t& heap_address)
{
	return write_array(main_memory, alignment, v.data(), v.size(), heap_address);
}

template <typename T>
static T* write_array(uint8_t* main_memory, size_t alignment, T* data, size_t size, paddr_t& heap_address)
{
	paddr_t array_address = align_to(alignment, heap_address);
	heap_address = array_address + size * sizeof(T);
	memcpy(main_memory + array_address, data, size * sizeof(T));
	return reinterpret_cast<T*>(array_address);
}

template <typename T>
static T* write_vector(uint8_t* main_memory, size_t alignment, std::vector<T> v, paddr_t& heap_address)
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

void print_header(std::string string, uint header_length = 80)
{
	uint spacers = string.length() < header_length ? header_length - string.length() : 0;
	printf("\n");
	for(uint i = 0; i < spacers / 2; ++i)
		printf("-");
	printf("%s", string.c_str());
	for(uint i = 0; i < (spacers + 1) / 2; ++i)
		printf("-");
	printf("\n");
}

const static std::vector<std::string> arch_names = {"TRaX","STRaTA", "STRaTA-RT", "Dual-Streaming", "RIC"};

struct SceneConfig
{
	std::string name;
	rtm::vec3 cam_pos;
	rtm::vec3 cam_target;
	float focal_length;
};

const static std::vector<SceneConfig> scene_configs =
{
	{"crytek-sponza", rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f), 12.0f}, //CRYTEC SPONZA

	{"intel-sponza", rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f), 12.0f}, //INTEL SPONZA
	
	{"san-miguel", rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(7.448 + 0.608, 1.014 + 0.026, 12.357 - 0.794), 12.0f}, //SAN_MIGUEL
	
	{"hairball", rtm::vec3(0, 0, 10), rtm::vec3(0, 0, 0), 24.0f}, //HAIRBALL

	{"cornellbox", rtm::vec3(0, 2, 4), rtm::vec3(0, 0, 0), 24.0f}, //CORNELLBOX

	{"sibenik", rtm::vec3(3.0, -13.0, 0.0), rtm::vec3(0, -12.0, 0), 12.0f},
};


class SimulationConfig
{
public:
	struct Param
	{
		enum Type
		{
			INT,
			FLOAT,
			STRING,
		}
		type;

		union
		{
			int i;
			float f;
		};

		std::string s;

		Param() {};

		Param(const Param& other)
		{
			*this = other;
		}


		Param& operator=(const Param& other)
		{
			type = other.type;
			if(type == Param::Type::STRING)
			{
				s = std::string(other.s);
			}
			else
			{
				i = other.i;
			}
			return *this;
		}

		~Param()
		{

		}
	};

	rtm::Camera camera;

private:
	std::map<std::string, Param> _params;

public:
	SimulationConfig(int argc, char* argv[])
	{
		//Simulation
		set_param("logging_interval", 100000);


		//Arch
		set_param("arch_name", "TRaX");
		set_param("num_threads", 4);
		set_param("num_tms", 128);
		set_param("num_tps", 128);
		set_param("num_rt_cores", 1);
		set_param("max_rays", 128);

		set_param("l2_size", 72 << 20);
		set_param("l2_associativity", 18);
		set_param("l2_in_order", 0);

		set_param("l1_size", 128 << 10);
		set_param("l1_associativity", 16);
		set_param("l1_in_order", 0);

		//Workload
		set_param("scene_name", "intel-sponza");

		set_param("framebuffer_width", 1024);
		set_param("framebuffer_height", 1024);

		//DS
		set_param("warm_l2", 0);
		set_param("pregen_rays", 0);
		set_param("pregen_bounce", 0);


		set_param("use_scene_buffer", 0);
		set_param("rays_on_chip", 0);
		set_param("hits_on_chip", 0);
		set_param("use_early", 0);
		set_param("hit_delay", 0);
		set_param("hit_buffer_size", 1024 * 1024);
		set_param("traversal_scheme", 0);
		set_param("weight_scheme", 0);

		//RIC


		set_param("max_active_set_size", 20 << 20);

		for(uint i = 1; i < argc; ++i)
		{
			std::string arg(argv[i]);
			size_t start_pos = arg.find("--");
			size_t split_pos = arg.find("=");
			if(start_pos == std::string::npos || split_pos == std::string::npos) continue;

			//--xxx=yyy
			start_pos += 2;
			std::string key = arg.substr(start_pos, split_pos - start_pos);
			split_pos++;
			std::string value = arg.substr(split_pos, arg.size() - split_pos);

			parse_param(key, value);
		}

		set_param("arch_id", -1);
		for(int i = 0; i < arch_names.size(); ++i)
			if(get_string("arch_name").compare(arch_names[i]) == 0)
				set_param("arch_id", i);
		_assert(get_int("arch_id") != -1);

		set_param("scene_id", -1);
		for(int i = 0; i < scene_configs.size(); ++i)
			if(get_string("scene_name").compare(scene_configs[i].name) == 0)
				set_param("scene_id", i);
		_assert(get_int("scene_id") != -1);

		uint scene_id = get_int("scene_id");
		camera = rtm::Camera(get_int("framebuffer_width"), get_int("framebuffer_height"), scene_configs[scene_id].focal_length, scene_configs[scene_id].cam_pos, scene_configs[scene_id].cam_target);
	}

	int get_int(const std::string& key) const
	{

		const auto& a = _params.find(key);
		_assert(a != _params.end());
		_assert(a->second.type == Param::Type::INT);
		return a->second.i;
	}

	float get_float(const std::string& key) const
	{
		const auto& a = _params.find(key);
		_assert(a != _params.end());
		_assert(a->second.type == Param::Type::FLOAT);
		return a->second.f;
	}

	std::string get_string(const std::string& key) const
	{
		const auto& a = _params.find(key);
		_assert(a != _params.end());
		_assert(a->second.type == Param::Type::STRING);
		return a->second.s;
	}


	void set_param(const std::string& key, int value)
	{
		_params[key].type = Param::Type::INT;
		_params[key].i = value;
	}

	void set_param(const std::string& key, float value)
	{
		_params[key].type = Param::Type::FLOAT;
		_params[key].f = value;
	}

	void set_param(const std::string& key, const std::string& value)
	{
		_params[key].type = Param::Type::STRING;
		_params[key].s = std::string(value);
	}

	void parse_param(const std::string& key, const std::string& str)
	{
		if(_params.count(key))
		{
			if(_params[key].type == Param::Type::INT)
			{
				_params[key].i = std::stoi(str);
			}
			else if(_params[key].type == Param::Type::FLOAT)
			{
				_params[key].f = std::stof(str);
			}
			else if(_params[key].type == Param::Type::STRING)
			{
				_params[key].s = std::string(str);
			}
			else _assert(false);
		}
		else
		{
			printf("Invalid Param!: %s\n", key.c_str());
			_assert(false);
		}
	}

	void print()
	{
		printf("Simulation Parameters\n");
		for(std::pair<const std::string, Param>& a : _params)
		{
			if(a.second.type == Param::Type::INT)
			{
				printf("%s: %d\n", a.first.c_str(), a.second.i);
			}
			else if(a.second.type == Param::Type::FLOAT)
			{
				printf("%s: %f\n", a.first.c_str(), a.second.f);
			}
			else if(a.second.type == Param::Type::STRING)
			{
				printf("%s: %s\n", a.first.c_str(), a.second.s.c_str());
			}
			else _assert(false);
		}
		printf("\n");
	}
};

}
