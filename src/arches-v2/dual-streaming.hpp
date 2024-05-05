#pragma once

#include "shared-utils.hpp"
#include "units/dual-streaming/unit-treelet-rt-core.hpp"
#include "units/dual-streaming/unit-stream-scheduler.hpp"
#include "units/dual-streaming/unit-ray-staging-buffer.hpp"
#include "units/dual-streaming/unit-tp.hpp"
#include "units/dual-streaming/unit-hit-record-updater.hpp"
#include "units/dual-streaming/unit-scene-buffer.hpp"
#include "units/dual-streaming/unit-treelet-rt-core.hpp"

namespace Arches {

namespace ISA {
namespace RISCV {
namespace DualStreaming {

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

		rtm::Ray ray;
		rtm::vec3 inv_d;
		ray.o.x = fr[0].f32;
		ray.o.y = fr[1].f32;
		ray.o.z = fr[2].f32;
		ray.t_min = fr[3].f32;
		inv_d.x = fr[4].f32;
		inv_d.y = fr[5].f32;
		inv_d.z = fr[6].f32;
		ray.t_max = fr[7].f32;

		rtm::AABB aabb;
		aabb.min.x = fr[8].f32;
		aabb.min.y = fr[9].f32;
		aabb.min.z = fr[10].f32;
		aabb.max.x = fr[11].f32;
		aabb.max.y = fr[12].f32;
		aabb.max.z = fr[13].f32;

		unit->float_regs->registers[instr.u.rd].f32 = rtm::intersect(aabb, ray, inv_d);
	}),
	InstructionInfo(0x2, "triisect", InstrType::CUSTOM2, Encoding::U, RegType::FLOAT, EXEC_DECL
	{
		Register32 * fr = unit->float_regs->registers;

		rtm::Ray ray;
		ray.o.x = fr[0].f32;
		ray.o.y = fr[1].f32;
		ray.o.z = fr[2].f32;
		ray.t_min = fr[3].f32;
		ray.d.x = fr[4].f32;
		ray.d.y = fr[5].f32;
		ray.d.z = fr[6].f32;
		ray.t_max = fr[7].f32;

		rtm::Triangle tri;
		tri.vrts[0].x = fr[8].f32;
		tri.vrts[0].y = fr[9].f32;
		tri.vrts[0].z = fr[10].f32;
		tri.vrts[1].x = fr[11].f32;
		tri.vrts[1].y = fr[12].f32;
		tri.vrts[1].z = fr[13].f32;
		tri.vrts[2].x = fr[14].f32;
		tri.vrts[2].y = fr[15].f32;
		tri.vrts[2].z = fr[16].f32;

		rtm::Hit hit;
		hit.t = fr[17].f32;
		hit.bc[0] = fr[18].f32;
		hit.bc[1] = fr[19].f32;
		hit.id = fr[20].u32;

		rtm::intersect(tri, ray, hit);

		fr[17].f32 = hit.t;
		fr[18].f32 = hit.bc[0];
		fr[19].f32 = hit.bc[1];
		fr[20].u32 = hit.id;
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

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3];});

}
}
}

namespace Units {
namespace DualStreaming {

class L1Cache : public UnitNonBlockingCache
{
private:
	std::pair<paddr_t, paddr_t> _treelet_range;

public:
	L1Cache(const UnitNonBlockingCache::Configuration& config, const std::pair<paddr_t, paddr_t>& treelet_range) : 
		UnitNonBlockingCache(config), _treelet_range(treelet_range)
	{}

private:
	UnitMemoryBase* _get_mem_higher(paddr_t addr) override
	{
		if(addr >= _treelet_range.first && addr < _treelet_range.second)
			return _mem_highers[1];

		return _mem_highers[0];
	}
};

}
}

namespace DualStreaming {

#include "dual-streaming-kernel/include.hpp"
#include "dual-streaming-kernel/intersect.hpp"
static DualStreamingKernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, GlobalConfig global_config)
{
	std::cerr << "Dual Streaming:: Initializing buffers...\n";
	//std::string s = "san-miguel";
	//std::string s = "sponza";
	std::string scene_name = scene_names[global_config.scene_id];

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);

	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());

	std::string filename = current_folder_path + "../../../../datasets/" + scene_name + ".obj";
	std::string bvh_filename = current_folder_path + "../../../../datasets/cache/" + scene_name + "_bvh.cache";
	std::string triangle_filename = current_folder_path + "../../../../datasets/cache/" + scene_name + "_triangles.cache";

	std::ifstream inputBVH(bvh_filename, std::ios::binary);
	std::ifstream inputTriangles(triangle_filename, std::ios::binary);
	rtm::BVH2 bvh;
	std::vector<rtm::Triangle> tris;
	if(inputBVH.is_open() && inputTriangles.is_open())
	{
		// Do not need to rebuild treelets every time
		printf("Loading BVH from %s\n", bvh_filename.c_str());
		rtm::BVH2::Node node;
		while (inputBVH.read(reinterpret_cast<char*>(&node), sizeof(node)))
			bvh.nodes.emplace_back(node);
		printf("Loaded %zd BVH nodes\n", bvh.nodes.size());

		printf("Loading triangles from %s\n", triangle_filename.c_str());
		rtm::Triangle cur_tri;
		while(inputTriangles.read(reinterpret_cast<char*>(&cur_tri), sizeof(rtm::Triangle)))
			tris.push_back(cur_tri);
		printf("Loaded %zd triangles\n", tris.size());
	}
	else
	{
		rtm::Mesh mesh(filename);
		std::vector<rtm::BVH2::BuildObject> build_objects;
		for(uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));
		bvh.build(build_objects);
		mesh.reorder(build_objects);
		mesh.get_triangles(tris);

		std::ofstream outputTreelets(bvh_filename, std::ios::binary);
		std::ofstream outputTriangles(triangle_filename, std::ios::binary);
		printf("Writing %zd bvh nodes to %s\n", bvh.nodes.size(), bvh_filename.c_str());
		for(auto& t : bvh.nodes)
			outputTreelets.write(reinterpret_cast<const char*>(&t), sizeof(t));

		printf("Writing %zd triangles to %s\n", tris.size(), triangle_filename.c_str());
		for(auto& tt : tris)
			outputTriangles.write(reinterpret_cast<const char*>(&tt), sizeof(tt));
	}
	rtm::PackedBVH2 packed_bvh(bvh);
	rtm::PackedTreeletBVH treelet_bvh(packed_bvh, tris);

	DualStreamingKernelArgs args;
	args.framebuffer_width = global_config.framebuffer_width;
	args.framebuffer_height = global_config.framebuffer_height;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, global_config.camera_config.focal_length, global_config.camera_config.position, global_config.camera_config.target);

	args.use_early = global_config.use_early;
	args.hit_delay = global_config.hit_delay;
	args.pregen_rays = global_config.pregen_rays;

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
	{
		args.treelets = treelet_bvh.treelets.data();
		args.tris = tris.data();
		pregen_rays(args, global_config.pregen_bounce, rays);
	}

	std::vector<rtm::Hit> hits(args.framebuffer_size);
	for(auto& hit : hits)
		hit.t = T_MAX;

	heap_address = align_to(ROW_BUFFER_SIZE, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.hit_records = write_vector(main_memory, ROW_BUFFER_SIZE, hits, heap_address);
	args.treelets = write_vector(main_memory, ROW_BUFFER_SIZE, treelet_bvh.treelets, heap_address);
	args.tris = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);
	args.rays = write_vector(main_memory, CACHE_BLOCK_SIZE, rays, heap_address);

	args.num_treelets = treelet_bvh.treelets.size();

	main_memory->direct_write(&args, sizeof(DualStreamingKernelArgs), KERNEL_ARGS_ADDRESS);
	return args;
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

static void run_sim_dual_streaming(const GlobalConfig& global_config)
{
	//hardware spec
	double clock_rate = 2'000'000'000.0;

	uint64_t mem_size = 1ull << 32; //4GB
	uint64_t stack_size = 1ull << 10; //1KB

#if 1 //Modern config

	//Compute
	uint num_threads_per_tp = 4;
	uint num_tps_per_tm = 64;
	uint num_tms = 64;

	//Scene buffer
	Units::DualStreaming::UnitSceneBuffer::Configuration scene_buffer_config;
	scene_buffer_config.size = 4 * 1024 * 1024; // 4MB
	scene_buffer_config.latency = 4;
	scene_buffer_config.num_banks = 32;
	scene_buffer_config.bank_select_mask = generate_nbit_mask(log2i(scene_buffer_config.num_banks)) << log2i(CACHE_BLOCK_SIZE);

	Units::DualStreaming::UnitSceneBuffer::PowerConfig scene_buffer_power_config;
	scene_buffer_power_config.leakage_power = 53.7192e-3f * scene_buffer_config.num_banks;
	scene_buffer_power_config.read_energy = 0.118977e-9f;
	scene_buffer_power_config.write_energy = 0.118977e-9f;

	//L2$
	Units::UnitBlockingCache::Configuration l2_config;
	l2_config.size = 32ull * 1024 * 1024; //32MB
	l2_config.associativity = 8;
	l2_config.latency = 10;
	l2_config.cycle_time = 4;
	l2_config.num_banks = 64;
	l2_config.cross_bar_width = 16;
	//l2_config.bank_select_mask = 0b0001'1110'0000'0100'0000ull;
	l2_config.bank_select_mask = (generate_nbit_mask(log2i(NUM_DRAM_CHANNELS)) << log2i(ROW_BUFFER_SIZE))  //The high order bits need to match the channel assignment bits
		| (generate_nbit_mask(log2i(l2_config.num_banks / NUM_DRAM_CHANNELS)) << log2i(CACHE_BLOCK_SIZE));

	Units::UnitBlockingCache::PowerConfig l2_power_config;
	l2_power_config.leakage_power = 184.55e-3f * l2_config.num_banks;
	l2_power_config.tag_energy = 0.00756563e-9f;
	l2_power_config.read_energy = 0.378808e-9f - l2_power_config.tag_energy;
	l2_power_config.write_energy = 0.365393e-9f - l2_power_config.tag_energy;

	//L1d$
	uint num_mshr = 256;
	Units::UnitNonBlockingCache::Configuration l1d_config;
	l1d_config.size = 128ull * 1024;
	l1d_config.associativity = 4;
	l1d_config.latency = 1;
	l1d_config.num_banks = 8;
	l1d_config.cross_bar_width = l1d_config.num_banks;
	l1d_config.bank_select_mask = generate_nbit_mask(log2i(l1d_config.num_banks)) << log2i(CACHE_BLOCK_SIZE);
	l1d_config.num_mshr = num_mshr / l1d_config.num_banks;
	l1d_config.use_lfb = false;

	Units::UnitNonBlockingCache::PowerConfig l1d_power_config;
	l1d_power_config.leakage_power = 7.19746e-3f * l1d_config.num_banks * num_tms;
	l1d_power_config.tag_energy = 0.000663943e-9f;
	l1d_power_config.read_energy = 0.0310981e-9f - l1d_power_config.tag_energy;
	l1d_power_config.write_energy = 0.031744e-9f - l1d_power_config.tag_energy;

	//L1i$
	uint num_icache_per_tm = l1d_config.num_banks;
	Units::UnitBlockingCache::Configuration l1i_config;
	l1i_config.size = 4 * 1024;
	l1i_config.associativity = 4;
	l1i_config.latency = 1;
	l1i_config.cycle_time = 1;
	l1i_config.num_banks = 1;
	l1i_config.cross_bar_width = 1;
	l1i_config.bank_select_mask = 0;

	Units::UnitBlockingCache::PowerConfig l1i_power_config;
	l1i_power_config.leakage_power = 1.72364e-3f * l1i_config.num_banks * num_icache_per_tm * num_tms;
	l1i_power_config.tag_energy = 0.000215067e-9f;
	l1i_power_config.read_energy = 0.00924837e-9f - l1i_power_config.tag_energy;
	l1i_power_config.write_energy = 0.00850041e-9f - l1i_power_config.tag_energy;

#else //Legacy config


	uint num_threads_per_tp = 1;
	uint num_tps_per_tm = 16;
	uint num_tms = 64;

	uint64_t l2_size = 4ull * 1024 * 1024; //4MB
	uint num_l2_banks = 32;
	uint l2_latency = 4;


	uint64_t l1_size = 16ull * 1024; //16KB
	uint num_l1_banks = 8;
#endif

	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM3] = "LWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM4] = "SWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM5] = "CSHIT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM6] = "LHIT";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::DualStreaming::custom0;

	uint num_tps = num_tps_per_tm * num_tms;
	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;
	uint num_tps_per_i_cache = num_tps_per_tm / num_icache_per_tm;
	uint num_l2_ports_per_tm = l1d_config.num_banks * 2;

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;

	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<Units::DualStreaming::UnitTreeletRTCore*> rtcs;
	std::vector<Units::DualStreaming::UnitRayStagingBuffer*> rsbs;

	std::vector<Units::UnitNonBlockingCache*> l1ds;
	std::vector<Units::UnitBlockingCache*> l1is;

	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

	uint dram_ports_per_channel = 8;
	std::set<uint> unused_dram_ports;
	for(uint i = 0; i < dram_ports_per_channel; ++i)
		unused_dram_ports.insert(i);

	Units::UnitBuffer::Configuration sram_config;
	sram_config.bank_select_mask = 0b0001'1110'0000'0000'0000ull;
	sram_config.cross_bar_width = NUM_DRAM_CHANNELS;
	sram_config.latency = 1;
	sram_config.size = 1 << 30;
	sram_config.num_banks = NUM_DRAM_CHANNELS;
	sram_config.num_ports = dram_ports_per_channel * NUM_DRAM_CHANNELS;

	Units::UnitBuffer sram(sram_config);
	simulator.register_unit(&sram);
	simulator.new_unit_group();

	Units::UnitDRAM dram(dram_ports_per_channel * NUM_DRAM_CHANNELS, mem_size, &simulator);
	simulator.register_unit(&dram);
	simulator.new_unit_group();

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());
	ELF elf(current_folder_path + "../../dual-streaming-kernel/riscv/kernel");
	paddr_t heap_address = dram.write_elf(elf);

	DualStreamingKernelArgs kernel_args = DualStreaming::initilize_buffers(&dram, heap_address, global_config);
	std::pair<paddr_t, paddr_t> treelet_range = {0, 0};
	if(global_config.use_scene_buffer)
		treelet_range = {(paddr_t)kernel_args.treelets, (paddr_t)kernel_args.treelets + kernel_args.num_treelets * sizeof(rtm::PackedTreelet)};

	l2_config.num_ports = num_tms * num_l2_ports_per_tm;
	l2_config.mem_higher = &dram;
	l2_config.mem_higher_port_offset = 0;
	l2_config.mem_higher_port_stride = 2;
	for(uint i = l2_config.mem_higher_port_offset; i < dram_ports_per_channel; i += l2_config.mem_higher_port_stride)
		unused_dram_ports.erase(i);

	Units::UnitBlockingCache l2(l2_config);
	simulator.register_unit(&l2);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);
	simulator.new_unit_group();

	scene_buffer_config.segment_start = (paddr_t)kernel_args.treelets;
	scene_buffer_config.segment_size = sizeof(rtm::PackedTreelet);
	scene_buffer_config.num_ports = num_tms * num_l2_ports_per_tm;
	scene_buffer_config.main_mem = &dram;
	scene_buffer_config.main_mem_port_stride = dram_ports_per_channel;
	scene_buffer_config.main_mem_port_offset = *unused_dram_ports.begin();
	unused_dram_ports.erase(*unused_dram_ports.begin());

	Units::DualStreaming::UnitSceneBuffer scene_buffer(scene_buffer_config);
	simulator.register_unit(&scene_buffer);

	Units::DualStreaming::UnitStreamScheduler::Configuration stream_scheduler_config;
	stream_scheduler_config.num_banks = 32;
	stream_scheduler_config.traversal_scheme = global_config.traversal_scheme;
	stream_scheduler_config.weight_scheme = global_config.weight_scheme;
	stream_scheduler_config.num_tms = num_tms;
	stream_scheduler_config.num_root_rays = kernel_args.framebuffer_size;
	stream_scheduler_config.treelet_addr = *(paddr_t*)&kernel_args.treelets;
	stream_scheduler_config.heap_addr = *(paddr_t*)&heap_address;
	stream_scheduler_config.cheat_treelets = (rtm::PackedTreelet*)&dram._data_u8[(size_t)kernel_args.treelets];
	stream_scheduler_config.main_mem = &dram;
	stream_scheduler_config.main_mem_port_stride = dram_ports_per_channel;
	stream_scheduler_config.main_mem_port_offset = *unused_dram_ports.begin();
	unused_dram_ports.erase(*unused_dram_ports.begin());
	if(global_config.use_scene_buffer)
	{
		stream_scheduler_config.max_active_segments = scene_buffer_config.size / sizeof(rtm::PackedTreelet);
		stream_scheduler_config.scene_buffer = &scene_buffer;
	}
	if(global_config.rays_on_chip)
	{
		stream_scheduler_config.main_mem = &sram;
		stream_scheduler_config.heap_addr = 0;
	}

	Units::DualStreaming::UnitStreamScheduler stream_scheduler(stream_scheduler_config);
	simulator.register_unit(&stream_scheduler);

	Units::DualStreaming::UnitHitRecordUpdater::Configuration hit_record_updater_config;
	hit_record_updater_config.num_tms = num_tms;
	hit_record_updater_config.hit_record_start = *(paddr_t*)&kernel_args.hit_records;
	hit_record_updater_config.cache_size = global_config.hit_buffer_size; // 128 * 16 = 2048B = 2KB
	hit_record_updater_config.associativity = 8;
	hit_record_updater_config.main_mem = &dram;
	hit_record_updater_config.main_mem_port_stride = dram_ports_per_channel;
	hit_record_updater_config.main_mem_port_offset = *unused_dram_ports.begin();
	unused_dram_ports.erase(*unused_dram_ports.begin());
	Units::DualStreaming::UnitHitRecordUpdater hit_record_updater(hit_record_updater_config);
	simulator.register_unit(&hit_record_updater);

	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		simulator.new_unit_group();
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);

		std::vector<Units::UnitMemoryBase*> mem_list;
		l1d_config.num_ports = num_tps_per_tm;
	#ifdef USE_RT_CORE
		l1d_config.num_ports += 1; //add extra port for RT core
	#endif
		l1d_config.mem_highers = {&l2, &scene_buffer};
		l1d_config.mem_higher_port_offset = num_l2_ports_per_tm * tm_index;
		l1d_config.mem_higher_port_stride = 2;

		l1ds.push_back(_new Units::DualStreaming::L1Cache(l1d_config, treelet_range));
		simulator.register_unit(l1ds.back());
		mem_list.push_back(l1ds.back());
		unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1ds.back();
		unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1ds.back();

		// L1 instruction cache
		for(uint i_cache_index = 0; i_cache_index < num_icache_per_tm; ++i_cache_index)
		{
			l1i_config.num_ports = num_tps_per_i_cache;
			l1i_config.mem_higher = &l2;
			l1i_config.mem_higher_port_offset = num_l2_ports_per_tm * tm_index + i_cache_index * 2 + 1;
			Units::UnitBlockingCache* i_l1 = _new Units::UnitBlockingCache(l1i_config);
			l1is.push_back(i_l1);
			simulator.register_unit(l1is.back());
		}

	#ifdef USE_RT_CORE
		rsbs.push_back(_new Units::DualStreaming::UnitRayStagingBuffer(1, tm_index, &stream_scheduler, &hit_record_updater));
		simulator.register_unit(rsbs.back());

		Units::DualStreaming::UnitTreeletRTCore::Configuration rtc_config;
		rtc_config.max_rays = 64;
		rtc_config.num_tp = num_tps_per_tm;
		rtc_config.treelet_base_addr = (paddr_t)kernel_args.treelets;
		rtc_config.hit_record_base_addr = (paddr_t)kernel_args.hit_records;
		rtc_config.use_early_termination = global_config.use_early;
		rtc_config.cache = l1ds.back();
		rtc_config.rsb = rsbs.back();

		rtcs.push_back(_new Units::DualStreaming::UnitTreeletRTCore(rtc_config));
		simulator.register_unit(rtcs.back());
		mem_list.push_back(rtcs.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM4] = rtcs.back(); //SWI
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM6] = rtcs.back(); //LHIT
	#else
		rsbs.push_back(_new Units::DualStreaming::UnitRayStagingBuffer(num_tps_per_tm, tm_index, &stream_scheduler, &hit_record_updater));
		simulator.register_unit(rsbs.back());
		mem_list.push_back(rsbs.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM3] = rsbs.back(); //LWI
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM4] = rsbs.back(); //SWI
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM5] = rsbs.back(); //CSHIT
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM6] = rsbs.back(); //LHIT
	#endif

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, kernel_args.framebuffer_width, kernel_args.framebuffer_height));
		simulator.register_unit(thread_schedulers.back());
		mem_list.push_back(thread_schedulers.back());
		unit_table[(uint)ISA::RISCV::InstrType::ATOMIC] = thread_schedulers.back();
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

		std::vector<Units::UnitSFU*> sfu_list;

		sfu_list.push_back(_new Units::UnitSFU(num_tps_per_tm, 2, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(num_tps_per_tm / 8, 1, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(num_tps_per_tm / 16, 6, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(2, 3, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(1, 22, 8, num_tps_per_tm));
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
			tp_config.tp_index = tp_index;
			tp_config.tm_index = tm_index;
			tp_config.stack_size = stack_size;
			tp_config.cheat_memory = dram._data_u8;
			tp_config.inst_cache = l1is[uint(tm_index * num_icache_per_tm + tp_index / num_tps_per_i_cache)];
			tp_config.num_tps_per_i_cache = num_tps_per_i_cache;
			tp_config.unit_table = &unit_tables.back();
			tp_config.unique_mems = &mem_lists.back();
			tp_config.unique_sfus = &sfu_lists.back();
			tp_config.num_threads = num_threads_per_tp;

			tps.push_back(new Units::DualStreaming::UnitTP(tp_config));
			simulator.register_unit(tps.back());
		}
	}

	for(auto& tp : tps)
		tp->set_entry_point(elf.elf_header->e_entry.u64);

	//master logs
	Units::UnitBuffer::Log sram_log;
	Units::UnitDRAM::Log dram_log;
	Units::UnitBlockingCache::Log l2_log;
	Units::UnitNonBlockingCache::Log l1d_log;
	Units::UnitBlockingCache::Log l1i_log;
	Units::UnitTP::Log tp_log;

	Units::DualStreaming::UnitTreeletRTCore::Log rtc_log;
	Units::DualStreaming::UnitSceneBuffer::Log sb_log;
	Units::DualStreaming::UnitStreamScheduler::Log ss_log;

	uint delta = global_config.logging_interval;
	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute(delta, [&]() -> void
	{
		float delta_us = delta / (clock_rate / 1'000'000);

		Units::UnitBuffer::Log sram_delta_log = delta_log(sram_log, sram);
		Units::UnitDRAM::Log dram_delta_log = delta_log(dram_log, dram);
		Units::UnitBlockingCache::Log l2_delta_log = delta_log(l2_log, l2);
		Units::UnitNonBlockingCache::Log l1d_delta_log = delta_log(l1d_log, l1ds);
		Units::DualStreaming::UnitSceneBuffer::Log sb_delta_log = delta_log(sb_log, scene_buffer);
		Units::DualStreaming::UnitStreamScheduler::Log ss_delta_log = delta_log(ss_log, stream_scheduler);

		printf("                             \n");
		printf("Cycle: %lld                  \n", simulator.current_cycle);
		printf("Threads Launched: %d         \n", atomic_regs.iregs[0] * 64);
		printf("Buckets Launched: %lld       \n", ss_log.buckets_launched);
		printf("Segments Launched: %lld      \n", ss_log.segments_launched);
		printf("                             \n");
		printf(" Ray Total: %8.1f bytes/cycle\n", (float)(ss_delta_log.buckets_generated + ss_delta_log.buckets_launched) * RAY_BUCKET_SIZE / delta);
		printf(" Ray Write: %8.1f bytes/cycle\n", (float)ss_delta_log.buckets_generated * RAY_BUCKET_SIZE / delta);
		printf("  Ray Read: %8.1f bytes/cycle\n", (float)ss_delta_log.buckets_launched * RAY_BUCKET_SIZE / delta);
		printf("  Ray Rate: %8.1f Mrays/s    \n", (float)ss_delta_log.buckets_launched * MAX_RAYS_PER_BUCKET / delta_us);
		printf("                             \n");
		printf("Scene Fill: %8.1f bytes/cycle\n", (float)sb_delta_log.bytes_written / delta);
		printf("Scene Read: %8.1f bytes/cycle\n", (float)sb_delta_log.bytes_read / delta);
		printf("                             \n");
		printf("DRAM Total: %8.1f bytes/cycle\n", (float)(dram_delta_log.bytes_read + dram_delta_log.bytes_written) / delta);
		printf("DRAM Write: %8.1f bytes/cycle\n", (float)dram_delta_log.bytes_written / delta);
		printf("                             \n");
		printf(" DRAM Read: %8.1f bytes/cycle\n", (float)dram_delta_log.bytes_read / delta);
		printf("  L2$ Read: %8.1f bytes/cycle\n", (float)l2_delta_log.bytes_read / delta);
		printf(" L1d$ Read: %8.1f bytes/cycle\n", (float)l1d_delta_log.bytes_read / delta);
		printf("                             \n");
	});
	auto stop = std::chrono::high_resolution_clock::now();

	cycles_t frame_cycles = simulator.current_cycle;
	double frame_time = frame_cycles / clock_rate;
	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
	float total_power = dram.total_power();

	tp_log.print_profile(dram._data_u8);

	dram.print_usimm_stats(CACHE_BLOCK_SIZE, 4, frame_cycles);
	print_header("DRAM");
	delta_log(dram_log, dram);
	dram_log.print(frame_cycles);

	print_header("SRAM");
	delta_log(sram_log, sram);
	sram_log.print(frame_cycles);

	print_header("Stream Scheduler");
	delta_log(ss_log, stream_scheduler);
	ss_log.print();

	print_header("Scene Buffer");
	delta_log(sb_log, scene_buffer);
	sb_log.print(frame_cycles);
	total_power += sb_log.print_power(scene_buffer_power_config, frame_time);

	print_header("L2$");
	delta_log(l2_log, l2);
	l2_log.print(frame_cycles);
	total_power += l2_log.print_power(l2_power_config, frame_time);

	print_header("L1d$");
	delta_log(l1d_log, l1ds);
	l1d_log.print(frame_cycles);
	total_power += l1d_log.print_power(l1d_power_config, frame_time);

	print_header("L1i$");
	delta_log(l1i_log, l1is);
	l1i_log.print(frame_cycles);
	total_power += l1i_log.print_power(l1i_power_config, frame_time);

	print_header("TP");
	delta_log(tp_log, tps);
	tp_log.print(frame_cycles, tps.size());

	if(!rtcs.empty())
	{
		print_header("RT Core");
		delta_log(rtc_log, rtcs);
		rtc_log.print(frame_cycles, rtcs.size());
	}

	float total_energy = total_power * frame_time;

	print_header("Performance Summary");
	printf("Cycles: %lld\n", frame_cycles);
	printf("Clock rate: %.0f MHz\n", clock_rate / 1'000'000.0);
	printf("Frame time: %.3g ms\n", frame_time * 1000.0);
	printf("Mrays/s: %.0f\n", (float)ss_log.rays / frame_time / 1'000'000);

	print_header("Power Summary");
	printf("Energy: %.2f mJ\n", total_energy * 1000.0);
	printf("Power: %.2f W\n", total_power);
	printf("Mrays/J: %.2f\n", (float)ss_log.rays / total_energy / 1'000'000);

	print_header("Simulation Summary");
	printf("Simulation rate: %.2f KHz\n", frame_cycles / simulation_time / 1000.0);
	printf("Simulation time: %.0f s\n", simulation_time);

	stbi_flip_vertically_on_write(true);
	dram.dump_as_png_uint8((paddr_t)kernel_args.framebuffer, kernel_args.framebuffer_width, kernel_args.framebuffer_height, "out.png");

	for(auto& tp : tps) delete tp;
	for(auto& sfu : sfus) delete sfu;
	for(auto& l1d : l1ds) delete l1d;
	for(auto& l1i : l1is) delete l1i;
	for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
	for(auto& rtc : rtcs) delete rtc;
	for(auto& rsb : rsbs) delete rsb;
}

}
}