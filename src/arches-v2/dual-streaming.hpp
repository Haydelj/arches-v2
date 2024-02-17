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

#include "units/dual-streaming/unit-stream-scheduler-dfs.hpp"
//#include "units/dual-streaming/unit-stream-scheduler.hpp"
#include "units/dual-streaming/unit-ray-staging-buffer.hpp"
#include "units/dual-streaming/unit-ds-tp.hpp"
#include "units/dual-streaming/unit-hit-record-updater.hpp"

#include "util/elf.hpp"
#include "isa/riscv.hpp"

#include "dual-streaming-kernel/include.hpp"
#include <Windows.h>

namespace Arches {

namespace ISA { namespace RISCV {

//see the opcode map for details
const static InstructionInfo isa_custom0_000_imm[8] =
{
	InstructionInfo(0x0, "fchthrd", InstrType::CUSTOM0, Encoding::U, RegType::INT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::INT;
		reg_addr.sign_ext = false;

		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = sizeof(uint32_t);
		req.dst = reg_addr.u8;
		req.vaddr = 0x0ull;

		return req;
	}),
	InstructionInfo(0x1, "boxisect", InstrType::CUSTOM1, Encoding::U, RegType::FLOAT, EXEC_DECL
	{
		Register32 * fr = unit->float_regs->registers;

		rtm::vec3 inv_d;

		rtm::Ray ray;
		ray.o.x = fr[0].f32;
		ray.o.y = fr[1].f32;
		ray.o.z = fr[2].f32;
		inv_d.x = fr[3].f32;
		inv_d.y = fr[4].f32;
		inv_d.z = fr[5].f32;

		rtm::AABB aabb;
		aabb.min.x = fr[6].f32;
		aabb.min.y = fr[7].f32;
		aabb.min.z = fr[8].f32;
		aabb.max.x = fr[9].f32;
		aabb.max.y = fr[10].f32;
		aabb.max.z = fr[11].f32;

		unit->float_regs->registers[instr.u.rd].f32 = rtm::intersect(aabb, ray, inv_d);
	}),
	InstructionInfo(0x2, "triisect", InstrType::CUSTOM2, Encoding::U, RegType::INT, RegType::FLOAT, EXEC_DECL
	{
		Register32 * fr = unit->float_regs->registers;

		rtm::Ray ray;
		ray.o.x = fr[0].f32;
		ray.o.y = fr[1].f32;
		ray.o.z = fr[2].f32;
		ray.d.x = fr[3].f32;
		ray.d.y = fr[4].f32;
		ray.d.z = fr[5].f32;

		rtm::Triangle tri;
		tri.vrts[0].x = fr[6].f32;
		tri.vrts[0].y = fr[7].f32;
		tri.vrts[0].z = fr[8].f32;
		tri.vrts[1].x = fr[9].f32;
		tri.vrts[1].y = fr[10].f32;
		tri.vrts[1].z = fr[11].f32;
		tri.vrts[2].x = fr[12].f32;
		tri.vrts[2].y = fr[13].f32;
		tri.vrts[2].z = fr[14].f32;

		rtm::Hit hit;
		hit.t = fr[15].f32;
		hit.bc[0] = fr[16].f32;
		hit.bc[1] = fr[17].f32;

		unit->int_regs->registers[instr.u.rd].u32 = rtm::intersect(tri, ray, hit);

		fr[15].f32 = hit.t;
		fr[16].f32 = hit.bc[0];
		fr[17].f32 = hit.bc[1];
	}),
};

const static InstructionInfo isa_custom0_funct3[8] =
{
	InstructionInfo(0x0, META_DECL{return isa_custom0_000_imm[instr.u.imm_31_12 >> 3]; }),
	InstructionInfo(0x1, "lwi", InstrType::CUSTOM3, Encoding::I, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{	
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		//load bucket ray into registers [rd - (rd + N)]
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::LOAD;
		mem_req.size = sizeof(WorkItem);
		mem_req.dst = reg_addr.u8;
		mem_req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

		return mem_req;
	}),
	InstructionInfo(0x2, "swi", InstrType::CUSTOM4, Encoding::S, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		//store bucket ray to hit record updater
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(WorkItem);
		mem_req.vaddr = unit->int_regs->registers[instr.s.rs1].u64 + s_imm(instr);
		
		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(WorkItem) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.s.rs2 + i].f32;

		return mem_req;
	}),
	InstructionInfo(0x3, "cshit", InstrType::CUSTOM5, Encoding::S, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{	
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(rtm::Hit);
		mem_req.vaddr = unit->int_regs->registers[instr.s.rs1].u64 + s_imm(instr);

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.s.rs2 + i].f32;

		return mem_req;
	}),
	InstructionInfo(0x4, "lhit", InstrType::CUSTOM6, Encoding::I, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		//load hit record into registers [rd - (rd + N)]
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::LOAD;
		mem_req.size = sizeof(rtm::Hit);
		mem_req.dst = reg_addr.u8;
		mem_req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

		return mem_req;
	}),
	InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::I, RegType::FLOAT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		//load hit record into registers [rd - (rd + N)]
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::LOAD;
		mem_req.size = sizeof(rtm::Hit);
		mem_req.dst = reg_addr.u8;
		mem_req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

		return mem_req;
	}),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3]; });

}}

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
std::vector<std::string> scene_names = { "sponza", "san-miguel", "hairball", "living_room"};
struct SceneConfig
{
	rtm::Camera camera;
}scene_configs[SCENES::NUMBER];
struct GlobalConfig
{
	uint scene_id = 0;
	uint framebuffer_width = 256;
	uint framebuffer_height = 256;
	uint traversal_scheme = 0; // 0 - BFS, 1 - DFS
	uint hit_buffer_size = 128 * 1024; // number of hits, assuming 128 * 16 * 1024 B = 2MB
	bool use_early = 0;
	bool hit_delay = 0; // hit delay sucks usually
	SceneConfig scene_config;
}global_config;
bool readCmd = true;

static KernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address)
{
	//std::string s = "san-miguel";
	//std::string s = "sponza";
	std::string s = scene_names[global_config.scene_id];
	//std::cout << s << '\n';
	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());
	std::string filename = current_folder_path + "../../datasets/" + s + ".obj";

	std::ifstream inputTreelets(s + "_treelets.dat", std::ios::binary);
	std::ifstream inputTriangles(s + "_triangles.dat", std::ios::binary);
	TreeletBVH treelet_bvh;
	std::vector<rtm::Triangle> tris;
	if (inputTreelets.is_open() && inputTriangles.is_open())
	{
		// Do not need to rebuild treelets every time
		std::cout << "Loading treelets from file...\n";
		Treelet t;
		rtm::Triangle tt;
		while (inputTreelets.read(reinterpret_cast<char*>(&t), TREELET_SIZE))
		{
			treelet_bvh.treelets.push_back(t);
		}
		while (inputTriangles.read(reinterpret_cast<char*>(&tt), sizeof(rtm::Triangle)))
		{
			tris.push_back(tt);
		}
		std::cout << "Treelet size: " << treelet_bvh.treelets.size() << '\n';
	}
	else
	{
		rtm::Mesh mesh(filename);
		rtm::BVH blas;
		std::vector<rtm::BVH::BuildObject> build_objects;
		for (uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));
		blas.build(build_objects);
		mesh.reorder(build_objects);
		mesh.get_triangles(tris);
		treelet_bvh = TreeletBVH(blas, mesh);
		std::ofstream outputTreelets(s + "_treelets.dat", std::ios::binary);
		std::ofstream outputTriangles(s + "_triangles.dat", std::ios::binary);
		std::cout << "Writing treelets to disk...\n";
		std::cout << treelet_bvh.treelets.size() * TREELET_SIZE << '\n';
		for (auto& t : treelet_bvh.treelets)
		{
			outputTreelets.write(reinterpret_cast<const char*>(&t), TREELET_SIZE);
		}
		for(auto& tt: tris)
		{
			outputTriangles.write(reinterpret_cast<const char*>(&tt), sizeof(rtm::Triangle));
		}
	}
	KernelArgs args;
	args.framebuffer_width = global_config.framebuffer_width;
	args.framebuffer_height = global_config.framebuffer_height;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;

	args.samples_per_pixel = 1;
	args.max_path_depth = 1;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	
	args.camera = global_config.scene_config.camera;

	args.use_early = global_config.use_early;
	args.hit_delay = global_config.hit_delay;

	heap_address = align_to(ROW_BUFFER_SIZE, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address); heap_address += args.framebuffer_size * sizeof(uint32_t);

	std::vector<rtm::Hit> hits(args.framebuffer_size);
	for(auto& hit : hits) hit.t = T_MAX;
	args.hit_records = write_vector(main_memory, ROW_BUFFER_SIZE, hits, heap_address);
	args.treelets = write_vector(main_memory, ROW_BUFFER_SIZE, treelet_bvh.treelets, heap_address);

	// Do not need to write tris ^_^
	args.triangles = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);

	main_memory->direct_write(&args, sizeof(KernelArgs), KERNEL_ARGS_ADDRESS);

	return args;
}

static void run_sim_dual_streaming(int argc, char* argv[])
{
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

		if (key == "scene_name")
		{
			for (int i = 0; i < scene_names.size(); i++)
			{
				if (scene_names[i] == value)
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
		std::cout << key << ' ' << value << '\n';
	};

	// 0 is .exe
	for (int i = 1; i < argc; i++)
	{
		if(readCmd) ParseCommand(argv[i]);
	}

	scene_configs[SCENES::SPONZA].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	scene_configs[SCENES::SAN_MIGUEL].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 24.0f, rtm::vec3(24.4, 16.4, 12.8), rtm::vec3(24.4 - 0.3, 16.4 - 0.6, 12.8 - 0.6));
	scene_configs[SCENES::HAIRBALL].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0, rtm::vec3(0, 0, 10), rtm::vec3(0, 0, -1));
	scene_configs[SCENES::LIVING_ROOM].camera = rtm::Camera(global_config.framebuffer_width, global_config.framebuffer_height, 12.0, rtm::vec3(-1.15, 2.13, 7.72), rtm::vec3(-1.15 + 0.3, 2.13 - 0.2, 7.72 - 0.92));
	global_config.scene_config = scene_configs[global_config.scene_id];


	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM3] = "LWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM4] = "SWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM5] = "CSHIT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM6] = "LHIT";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::custom0;

	uint64_t num_tps_per_tm = 64;
	uint64_t num_tms = 64;

	uint64_t num_tps = num_tps_per_tm * num_tms;
	uint64_t num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;

	//hardware spec
	uint64_t mem_size = 4ull * 1024ull * 1024ull * 1024ull; //4GB
	uint64_t stack_size = 4096; //1KB

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::DualStreaming::UnitRayStagingBuffer*> rsbs;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<Units::UnitNonBlockingCache*> l1s;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());

	Units::UnitDRAM dram(64, mem_size, &simulator); dram.clear();
	simulator.register_unit(&dram);

	simulator.new_unit_group();

	ELF elf(current_folder_path + "../dual-streaming-kernel/riscv/kernel");
	paddr_t heap_address = dram.write_elf(elf);

	KernelArgs kernel_args = initilize_buffers(&dram, heap_address);

	Units::DualStreaming::UnitStreamSchedulerDFS::Configuration stream_scheduler_config;
	stream_scheduler_config.treelet_addr = *(paddr_t*)&kernel_args.treelets;
	stream_scheduler_config.heap_addr = *(paddr_t*)&heap_address;
	stream_scheduler_config.num_tms = num_tms;
	stream_scheduler_config.num_banks = 16;
	stream_scheduler_config.cheat_treelets = (Treelet*)&dram._data_u8[(size_t)kernel_args.treelets];
	stream_scheduler_config.main_mem = &dram;
	stream_scheduler_config.main_mem_port_offset = 1;
	stream_scheduler_config.main_mem_port_stride = 4;
	stream_scheduler_config.traversal_scheme = global_config.traversal_scheme;
	stream_scheduler_config.num_root_rays = kernel_args.framebuffer_size;

	Units::DualStreaming::UnitStreamSchedulerDFS stream_scheduler(stream_scheduler_config);
	simulator.register_unit(&stream_scheduler);

	Units::DualStreaming::UnitHitRecordUpdater::Configuration hit_record_updater_config;
	hit_record_updater_config.num_tms = num_tms;
	hit_record_updater_config.main_mem = &dram;
	hit_record_updater_config.main_mem_port_offset = 3;
	hit_record_updater_config.main_mem_port_stride = 4;
	hit_record_updater_config.hit_record_start = *(paddr_t*)&kernel_args.hit_records;
	hit_record_updater_config.cache_size = global_config.hit_buffer_size; // 128 * 16 = 2048B = 2KB
	hit_record_updater_config.associativity = 4;
	Units::DualStreaming::UnitHitRecordUpdater hit_record_updater(hit_record_updater_config);
	simulator.register_unit(&hit_record_updater);

	/*
	Units::UnitBuffer::Configuration scene_buffer_config;
	scene_buffer_config.size = scene_buffer_size;
	scene_buffer_config.num_banks = 32;
	scene_buffer_config.num_ports = num_tms + 1;
	scene_buffer_config.latency = 3;

	Units::UnitBuffer scene_buffer(scene_buffer_config);
	simulator.register_unit(&scene_buffer);
	*/

	simulator.new_unit_group();

	Units::UnitBlockingCache::Configuration l2_config;
	l2_config.size = 32 * 1024 * 1024;
	l2_config.associativity = 8;
	l2_config.num_ports = num_tms * 8;
	l2_config.num_banks = 32;
	l2_config.cross_bar_width = 32;
	l2_config.bank_select_mask = 0b0001'1110'0000'0100'0000ull; //The high order bits need to match the channel assignment bits
	l2_config.latency = 10;
	l2_config.cycle_time = 1;
	l2_config.mem_higher = &dram;
	l2_config.mem_higher_port_offset = 0;
	l2_config.mem_higher_port_stride = 2;

	Units::UnitBlockingCache l2(l2_config);
	simulator.register_unit(&l2);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);

	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		simulator.new_unit_group();
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);

		std::vector<Units::UnitMemoryBase*> mem_list;

		Units::UnitNonBlockingCache::Configuration l1_config;
		l1_config.size = 32 * 1024;
		l1_config.associativity = 4;
		l1_config.num_ports = num_tps_per_tm;
		l1_config.num_banks = 8;
		l1_config.cross_bar_width = 8;
		l1_config.bank_select_mask = 0b0000'0101'0100'0000ull;
		l1_config.latency = 1;
		l1_config.num_lfb = 8;
		l1_config.mem_higher = &l2;
		l1_config.mem_higher_port_offset = l1_config.num_banks * tm_index;

		l1s.push_back(new Units::UnitNonBlockingCache(l1_config));
		mem_list.push_back(l1s.back());
		simulator.register_unit(l1s.back());

		unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1s.back();
		unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1s.back();

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, kernel_args.framebuffer_width, kernel_args.framebuffer_height));
		mem_list.push_back(thread_schedulers.back());
		simulator.register_unit(thread_schedulers.back());

		unit_table[(uint)ISA::RISCV::InstrType::ATOMIC] = thread_schedulers.back();
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

		rsbs.push_back(_new Units::DualStreaming::UnitRayStagingBuffer(num_tps_per_tm, tm_index, &stream_scheduler, &hit_record_updater));
		mem_list.push_back(rsbs.back());
		simulator.register_unit(rsbs.back());

		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM3] = rsbs.back(); //LWI
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM4] = rsbs.back(); //SWI
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM5] = rsbs.back(); //CSHIT
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM6] = rsbs.back(); //LHIT



		std::vector<Units::UnitSFU*> sfu_list;

		sfu_list.push_back(_new Units::UnitSFU(16, 2, 1, num_tps_per_tm));
		//sfu_list.push_back(_new Units::UnitSFU(50, 1, 2, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(2, 1, 1, num_tps_per_tm));
		//sfu_list.push_back(_new Units::UnitSFU(20, 1, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

		//sfu_list.push_back(_new Units::UnitSFU(20, 1, 16, num_tps_per_tm));
		sfu_list.push_back(_new Units::UnitSFU(1, 1, 16, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

		//sfu_list.push_back(_new Units::UnitSFU(20, 1, 4, num_tps_per_tm));
		sfu_list.push_back(_new Units::UnitSFU(2, 4, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

		//sfu_list.push_back(_new Units::UnitSFU(20, 18, 31, num_tps_per_tm));
		sfu_list.push_back(_new Units::UnitSFU(1, 31, 18, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM2] = sfu_list.back();

		for(auto& sfu : sfu_list)
			sfus.push_back(sfu);

		unit_tables.emplace_back(unit_table);
		sfu_lists.emplace_back(sfu_list);
		mem_lists.emplace_back(mem_list);

		for(uint tp_index = 0; tp_index < num_tps_per_tm; ++tp_index)
		{
			Units::UnitTP::Configuration tp_config;
			tp_config.num_threads = 1;
			tp_config.tp_index = tp_index;
			tp_config.tm_index = tm_index;
			tp_config.pc = elf.elf_header->e_entry.u64;
			tp_config.sp = 0x0;
			tp_config.gp = 0x0000000000012c34;
			tp_config.stack_size = stack_size;
			tp_config.cheat_memory = dram._data_u8;
			tp_config.unit_table = &unit_tables.back();
			tp_config.unique_mems = &mem_lists.back();
			tp_config.unique_sfus = &sfu_lists.back();

			tps.push_back(new Units::DualStreaming::UnitTP(tp_config));
			simulator.register_unit(tps.back());
			simulator.units_executing++;
		}
	}

	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute();
	auto stop = std::chrono::high_resolution_clock::now();

	dram.print_usimm_stats(CACHE_BLOCK_SIZE, 4, simulator.current_cycle);

	printf("\nL2\n");
	l2.log.print_log();

	printf("\nL1\n");
	Units::UnitNonBlockingCache::Log l1_log;
	for(auto& l1 : l1s)
		l1_log.accumulate(l1->log);
	l1_log.print_log();

	printf("\nTP\n");
	Units::UnitTP::Log tp_log(0x10000);
	for(auto& tp : tps)
		tp_log.accumulate(tp->log);
	tp_log.print_log();

	stream_scheduler.log.print();

	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("\nSummary\n");
	printf("Runtime: %lldms\n", duration.count());
	printf("Cycles: %lld\n", simulator.current_cycle);
	printf("MRays/s: %.2f\n", (float)kernel_args.framebuffer_size / (simulator.current_cycle / (2 * 1024)));

	paddr_t paddr_frame_buffer = reinterpret_cast<paddr_t>(kernel_args.framebuffer);

	std::string scene_name = scene_names[global_config.scene_id];
	dram.dump_as_png_uint8(paddr_frame_buffer, kernel_args.framebuffer_width, kernel_args.framebuffer_height, scene_name + "_out.png");

	for(auto& tp : tps) delete tp;
	for(auto& sfu : sfus) delete sfu;
	for(auto& rsb : rsbs) delete rsb;
	for(auto& ts : thread_schedulers) delete ts;
	for(auto& l1 : l1s) delete l1;
}

}