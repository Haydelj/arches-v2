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

typedef Units::UnitNonBlockingCache UnitL1Cache;
typedef Units::UnitNonBlockingCache UnitL2Cache;

std::vector<MemoryRange> memory_ranges;
std::shared_ptr<std::vector<MemoryRange>> memory_ranges_ptr;

#include "dual-streaming-kernel/include.hpp"
#include "dual-streaming-kernel/intersect.hpp"
static DualStreamingKernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, GlobalConfig global_config, uint page_size)
{
	memory_ranges.push_back({ heap_address, "Instructions" });

	std::string scene_name = scene_names[global_config.scene_id];

	TCHAR tc_exe_path[MAX_PATH];
	GetModuleFileName(NULL, tc_exe_path, MAX_PATH);
	std::wstring w_exe_path(tc_exe_path);
	std::string exe_path(w_exe_path.begin(), w_exe_path.end());

	std::string poject_folder = exe_path.substr(0, exe_path.rfind("build"));
	std::string data_folder = poject_folder + "datasets/";

	std::string filename = data_folder + scene_name + ".obj";
	std::string bvh_cache_filename = data_folder + "cache/" + scene_name + "_bvh.cache";

	rtm::Mesh mesh(filename);
	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2(bvh_cache_filename, build_objects);
	rtm::PackedBVH2 packed_bvh2(bvh2, build_objects);
	mesh.reorder(build_objects);

	rtm::PackedTreeletBVH treelet_bvh(packed_bvh2, mesh);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);

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

	heap_address = align_to(page_size, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);
	memory_ranges.push_back({ heap_address, "Frame Buffer" });

	args.hit_records = write_vector(main_memory, page_size, hits, heap_address);
	memory_ranges.push_back({ heap_address, "Hit Records" });

	args.treelets = write_vector(main_memory, page_size, treelet_bvh.treelets, heap_address);
	memory_ranges.push_back({ heap_address, "Scene Data" });

	args.tris = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);
	memory_ranges.push_back({ heap_address, "Triangles" });

	args.rays = write_vector(main_memory, CACHE_BLOCK_SIZE, rays, heap_address);
	memory_ranges.push_back({ heap_address, "Pregenerated Rays" });

	args.num_treelets = treelet_bvh.treelets.size();

	main_memory->direct_write(&args, sizeof(DualStreamingKernelArgs), KERNEL_ARGS_ADDRESS);
	memory_ranges.push_back({ KERNEL_ARGS_ADDRESS + sizeof(DualStreamingKernelArgs), "ARGS" });

	// range for ray buckets
	memory_ranges.push_back({ (paddr_t)1e15, "Ray Buckets"});

	for (auto& v : memory_ranges)
	{
		printf("Name %s, Memory end: %lld\n", v.data_type, v.paddr);
	}
	std::sort(memory_ranges.begin(), memory_ranges.end());
	assert(std::is_sorted(memory_ranges.begin(), memory_ranges.end()));
	memory_ranges_ptr = std::make_shared<std::vector<MemoryRange>>(memory_ranges);
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
	double clock_rate = 2.0e9;
	
#if 1 //Modern config
	uint num_threads_per_tp = 4;
	uint num_tps_per_tm = 64;
	uint num_tms = 64;
	uint64_t stack_size = 1ull << 10; //1KB

	//DRAM
	uint dram_ports_per_channel = 16;
	uint64_t mem_size =1ull << 32; //4GB

#if 1
	typedef Units::UnitDRAMRamulator UnitDRAM;
	UnitDRAM dram(dram_ports_per_channel * NUM_DRAM_CHANNELS, mem_size); dram.clear();
#else
	typedef Units::UnitDRAM UnitDRAM;
	UnitDRAM::init_usimm("gddr5_16ch.cfg", "1Gb_x16_amd2GHz.vi");
	UnitDRAM dram(dram_ports_per_channel * NUM_DRAM_CHANNELS, mem_size);
#endif

	uint num_channels = 8;// dram.num_channels();
	uint64_t row_size = 8 * 1024; // dram.row_size();
	uint64_t block_size = CACHE_BLOCK_SIZE; // dram.block_size();

	_assert(block_size <= MemoryRequest::MAX_SIZE);
	_assert(block_size == CACHE_BLOCK_SIZE);
	//_assert(row_size == DRAM_ROW_SIZE);

	//Scene buffer
	Units::DualStreaming::UnitSceneBuffer::Configuration scene_buffer_config;
	scene_buffer_config.size = 4 * 1024 * 1024; // 4MB
	scene_buffer_config.latency = 4;
	scene_buffer_config.num_banks = 32;
	scene_buffer_config.bank_select_mask = generate_nbit_mask(log2i(scene_buffer_config.num_banks)) << log2i(block_size);

	Units::DualStreaming::UnitSceneBuffer::PowerConfig scene_buffer_power_config;
	scene_buffer_power_config.leakage_power = 53.7192e-3f * scene_buffer_config.num_banks;
	scene_buffer_power_config.read_energy = 0.118977e-9f;
	scene_buffer_power_config.write_energy = 0.118977e-9f;

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.size = 32ull * 1024 * 1024; //32MB
	l2_config.block_size = block_size;
	l2_config.associativity = 8;
	l2_config.num_mshr = 16;
	l2_config.latency = 10;
	l2_config.cycle_time = 4;
	l2_config.num_banks = 64;
	l2_config.bank_select_mask = (generate_nbit_mask(log2i(num_channels)) << log2i(row_size))  //The high order bits need to match the channel assignment bits
		| (generate_nbit_mask(log2i(l2_config.num_banks / num_channels)) << log2i(block_size));

	UnitL2Cache::PowerConfig l2_power_config;
	l2_power_config.leakage_power = 184.55e-3f * l2_config.num_banks;
	l2_power_config.tag_energy = 0.00756563e-9f;
	l2_power_config.read_energy = 0.378808e-9f - l2_power_config.tag_energy;
	l2_power_config.write_energy = 0.365393e-9f - l2_power_config.tag_energy;

	//L1d$
	uint num_mshr = 256;
	UnitL1Cache::Configuration l1d_config;
	l1d_config.size = 128ull * 1024;
	l1d_config.block_size = block_size;
	l1d_config.associativity = 4;
	l1d_config.latency = 1;
	l1d_config.num_banks = 8;
	l1d_config.bank_select_mask = generate_nbit_mask(log2i(l1d_config.num_banks)) << log2i(block_size);
	l1d_config.num_mshr = num_mshr / l1d_config.num_banks;
	l1d_config.use_lfb = false;
	l1d_config.num_ports = num_tps_per_tm;
	uint8_t l1_weight_table[128];
	l1d_config.weight_table = l1_weight_table;
	for(uint i = 0; i < l1d_config.num_ports; ++i)
		l1_weight_table[i] = 1;

#ifdef USE_RT_CORE
	l1_weight_table[l1d_config.num_ports] = l1d_config.num_ports;
	l1d_config.num_ports += 1; //add extra port for RT core
#endif

	UnitL1Cache::PowerConfig l1d_power_config;
	l1d_power_config.leakage_power = 7.19746e-3f * l1d_config.num_banks * num_tms;
	l1d_power_config.tag_energy = 0.000663943e-9f;
	l1d_power_config.read_energy = 0.0310981e-9f - l1d_power_config.tag_energy;
	l1d_power_config.write_energy = 0.031744e-9f - l1d_power_config.tag_energy;

	//L1i$
	uint num_icache_per_tm = l1d_config.num_banks;
	Units::UnitBlockingCache::Configuration l1i_config;
	l1i_config.size = 4 * 1024;
	l1i_config.block_size = block_size;
	l1i_config.associativity = 4;
	l1i_config.latency = 1;
	l1i_config.cycle_time = 1;
	l1i_config.num_banks = 1;
	l1i_config.bank_select_mask = 0;

	Units::UnitBlockingCache::PowerConfig l1i_power_config;
	l1i_power_config.leakage_power = 1.72364e-3f * l1i_config.num_banks * num_icache_per_tm * num_tms;
	l1i_power_config.tag_energy = 0.000215067e-9f;
	l1i_power_config.read_energy = 0.00924837e-9f - l1i_power_config.tag_energy;
	l1i_power_config.write_energy = 0.00850041e-9f - l1i_power_config.tag_energy;

#else //Legacy config

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

	simulator.register_unit(&dram);
	simulator.new_unit_group();

	Units::UnitBuffer::Configuration sram_config;
	sram_config.latency = 1;
	sram_config.size = 1 << 30;
	sram_config.num_banks = num_channels;
	sram_config.num_ports = dram_ports_per_channel * num_channels;

	Units::UnitBuffer sram(sram_config);
	simulator.register_unit(&sram);
	simulator.new_unit_group();

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());
	ELF elf(current_folder_path + "../../dual-streaming-kernel/riscv/kernel");
	paddr_t heap_address = dram.write_elf(elf);

	DualStreamingKernelArgs kernel_args = DualStreaming::initilize_buffers(&dram, heap_address, global_config, row_size);
	dram.log.memory_ranges = memory_ranges_ptr;

	heap_address = align_to(row_size * num_channels, heap_address);

	std::pair<paddr_t, paddr_t> treelet_range = {(paddr_t)kernel_args.treelets, (paddr_t)kernel_args.treelets + kernel_args.num_treelets * sizeof(rtm::PackedTreelet)};

	std::set<uint> unused_dram_ports;
	for(uint i = 0; i < dram_ports_per_channel; ++i)
		unused_dram_ports.insert(i);

	l2_config.num_ports = num_tms * num_l2_ports_per_tm + 1;
	//l2_config.mem_higher = &dram;
	l2_config.mem_highers = {&dram};
	l2_config.mem_higher_port_offset = 0;
	l2_config.mem_higher_port_stride = 2;
	l2_config.unit_name = "L2 Cache";
	l2_config.memory_ranges = memory_ranges_ptr;
	for(uint i = l2_config.mem_higher_port_offset; i < dram_ports_per_channel; i += l2_config.mem_higher_port_stride)
		unused_dram_ports.erase(i);

	UnitL2Cache l2(l2_config);
	simulator.register_unit(&l2);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);
	simulator.new_unit_group();

	scene_buffer_config.segment_start = (paddr_t)kernel_args.treelets;
	scene_buffer_config.segment_size = sizeof(rtm::PackedTreelet);
	scene_buffer_config.num_ports = num_tms * num_l2_ports_per_tm;
	scene_buffer_config.row_size = row_size;
	scene_buffer_config.block_size = block_size;
	scene_buffer_config.num_channels = num_channels;
	scene_buffer_config.main_mem = &dram;
	scene_buffer_config.main_mem_port_stride = dram_ports_per_channel;
	scene_buffer_config.main_mem_port_offset = *unused_dram_ports.begin();
	unused_dram_ports.erase(*unused_dram_ports.begin());

	Units::DualStreaming::UnitSceneBuffer scene_buffer(scene_buffer_config);
	simulator.register_unit(&scene_buffer);

	Units::DualStreaming::UnitStreamScheduler::Configuration stream_scheduler_config;
	stream_scheduler_config.num_banks = 32;
	stream_scheduler_config.num_channels = num_channels;
	stream_scheduler_config.traversal_scheme = global_config.traversal_scheme;
	stream_scheduler_config.weight_scheme = global_config.weight_scheme;
	stream_scheduler_config.num_tms = num_tms;
	stream_scheduler_config.block_size = block_size;
	stream_scheduler_config.row_size = row_size;
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
	else
	{
		stream_scheduler_config.l2_cache_port = l2_config.num_ports - 1;
		stream_scheduler_config.l2_cache = &l2; // &l2;
		stream_scheduler_config.max_active_segments = num_tms * 2;
	}
	if(global_config.rays_on_chip)
	{
		stream_scheduler_config.main_mem = &sram;
	}

	Units::DualStreaming::UnitStreamScheduler stream_scheduler(stream_scheduler_config);
	simulator.register_unit(&stream_scheduler);

	Units::DualStreaming::UnitHitRecordUpdater::Configuration hit_record_updater_config;
	hit_record_updater_config.num_tms = num_tms;
	hit_record_updater_config.hit_record_start = *(paddr_t*)&kernel_args.hit_records;
	hit_record_updater_config.cache_size = global_config.hit_buffer_size; // 128 * 16 = 2048B = 2KB
	hit_record_updater_config.associativity = 8;
	hit_record_updater_config.row_size = row_size;
	hit_record_updater_config.num_channels = num_channels;
	hit_record_updater_config.main_mem = &dram;
	hit_record_updater_config.main_mem_port_stride = dram_ports_per_channel;
	hit_record_updater_config.main_mem_port_offset = *unused_dram_ports.begin();
	unused_dram_ports.erase(*unused_dram_ports.begin());

	if(global_config.hits_on_chip)
	{
		std::vector<rtm::Hit> hits(kernel_args.framebuffer_size);
		for(auto& hit : hits) hit.t = T_MAX;
		paddr_t address = *(paddr_t*)&kernel_args.hit_records;
		write_vector(&sram, row_size, hits, address);
		hit_record_updater_config.main_mem = &sram;
	}

	Units::DualStreaming::UnitHitRecordUpdater hit_record_updater(hit_record_updater_config);
	simulator.register_unit(&hit_record_updater);

	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		simulator.new_unit_group();
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);

		std::vector<Units::UnitMemoryBase*> mem_list;

		l1d_config.mem_highers = {&l2, &scene_buffer};
		l1d_config.mem_higher_port_offset = num_l2_ports_per_tm * tm_index;
		l1d_config.mem_higher_port_stride = 2;
		l1d_config.unit_name = "L1 Data Cache";
		l1d_config.memory_ranges = memory_ranges_ptr;

		l1ds.push_back(_new Units::DualStreaming::L1Cache(l1d_config, global_config.use_scene_buffer ? treelet_range : std::pair<paddr_t, paddr_t>(0ull, 0ull)));
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
			l1i_config.unit_name = "L1 Instruction Cache";
			l1i_config.memory_ranges = memory_ranges_ptr;
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

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, 64));
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
	UnitDRAM::Log dram_log;
	UnitL2Cache::Log l2_log;
	UnitL1Cache::Log l1d_log;
	Units::UnitBlockingCache::Log l1i_log;
	Units::UnitTP::Log tp_log;

	Units::DualStreaming::UnitTreeletRTCore::Log rtc_log;
	Units::DualStreaming::UnitSceneBuffer::Log sb_log;
	Units::DualStreaming::UnitStreamScheduler::Log ss_log;

	float dram_peak_bandwidth = std::min(NUM_DRAM_CHANNELS, (int)dram_ports_per_channel) * CACHE_BLOCK_SIZE;
	float l2_peak_bandwidth = std::min(num_l2_ports_per_tm * num_tms, 64u) * CACHE_BLOCK_SIZE;
	float l1_peak_bandwidth = std::min(num_tps_per_tm, 64u) * 4 * num_tms; // 4 bytes
	printf("DRAM peak bandwidth: %8.1f bytes/cycle\n", dram_peak_bandwidth);
	printf("L2 Cache peak bandwidth: %8.1f bytes/cycle\n", l2_peak_bandwidth);
	printf("L1d$ peak bandwidth: %8.1f bytes/cycle\n", l1_peak_bandwidth);
	uint delta = global_config.logging_interval;
	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute(delta, [&]() -> void
	{
		float delta_us = delta / (clock_rate / 1'000'000);

		Units::UnitBuffer::Log sram_delta_log = delta_log(sram_log, sram);
		UnitDRAM::Log dram_delta_log = delta_log(dram_log, dram);
		UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2);
		UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);

		Units::DualStreaming::UnitTreeletRTCore::Log rtc_delta_log = delta_log(rtc_log, rtcs);
		Units::DualStreaming::UnitSceneBuffer::Log sb_delta_log = delta_log(sb_log, scene_buffer);
		Units::DualStreaming::UnitStreamScheduler::Log ss_delta_log = delta_log(ss_log, stream_scheduler);

		printf("                             \n");
		printf("Cycle: %lld                  \n", simulator.current_cycle);
		printf("Threads Launched: %d         \n", atomic_regs.iregs[0] * 64);
		printf("Buckets Launched: %lld       \n", ss_log.buckets_launched);
		printf("Segments Launched: %lld      \n", ss_log.segments_launched);
		printf("Prefetch queue size: %lld    \n", stream_scheduler._l2_cache_prefetch_queue.size());
		printf("                             \n");
		printf(" Ray Total: %8.1f bytes/cycle\n", (float)(ss_delta_log.buckets_generated + ss_delta_log.buckets_launched) * RAY_BUCKET_SIZE / delta);
		printf(" Ray Write: %8.1f bytes/cycle\n", (float)ss_delta_log.buckets_generated * RAY_BUCKET_SIZE / delta);
		printf("  Ray Read: %8.1f bytes/cycle\n", (float)ss_delta_log.buckets_launched * RAY_BUCKET_SIZE / delta);
		printf("  Ray Rate: %8.1f Mrays/s    \n", (float)ss_delta_log.buckets_launched * Arches::Units::DualStreaming::RayBucket::MAX_RAYS / delta_us);
		printf("                             \n");
		printf("Scene Fill: %8.1f bytes/cycle\n", (float)sb_delta_log.bytes_written / delta);
		printf("Scene Read: %8.1f bytes/cycle\n", (float)sb_delta_log.bytes_read / delta);
		printf("--------------------------------------------------------DRAM---------------------------------------------------------\n");
		printf("DRAM Total: %8.1f bytes/cycle\n", (float)(dram_delta_log.bytes_read + dram_delta_log.bytes_written) / delta);
		printf(" DRAM Read: %8.1f bytes/cycle\n", (float)dram_delta_log.bytes_read / delta);
		printf("DRAM Write: %8.1f bytes/cycle\n", (float)dram_delta_log.bytes_written / delta);
		dram_delta_log.print_request_logs(delta);

		printf("--------------------------------------------------------SRAM---------------------------------------------------------\n");
		printf("SRAM Total: %8.1f bytes/cycle\n", (float)(sram_delta_log.bytes_read + sram_delta_log.bytes_written) / delta);
		printf("SRAM Write: %8.1f bytes/cycle\n", (float)sram_delta_log.bytes_written / delta);
		printf("SRAM  Read: %8.1f bytes/cycle\n", (float)sram_delta_log.bytes_read / delta);

		printf("--------------------------------------------------------L2 Cache-----------------------------------------------------\n");
		printf("  L2$ Read: %8.1f bytes/cycle\n", (float)l2_delta_log.bytes_read / delta);
		printf("        L2$ Hit Rate: %8.1f%%\n", 100.0 * l2_delta_log.hits / l2_delta_log.get_total());
		l2_delta_log.print_request_logs(delta);

		printf("--------------------------------------------------------L1d$-----------------------------------------------------\n");
		printf(" L1d$ Read: %8.1f bytes/cycle\n", (float)l1d_delta_log.bytes_read / delta);
		printf("L1d$ Hit Rate: %8.1f%%\n", 100.0 * l1d_delta_log.hits / l1d_delta_log.get_total());
		l1d_delta_log.print_request_logs(delta);
		printf("                             \n");
	});
	auto stop = std::chrono::high_resolution_clock::now();

	cycles_t frame_cycles = simulator.current_cycle;
	double frame_time = frame_cycles / clock_rate;
	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
	float total_power = dram.total_power();

	tp_log.print_profile(dram._data_u8);

	dram.print_stats(4, frame_cycles);
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

	print_header("Treelet Histogram");

#if 0
	uint treelet_counts[16];
	std::map<uint, uint64_t> treelet_histos[16];

	for(uint i = 0; i < 16; ++i)
		treelet_counts[i] = 0;

	for(auto& a : l1d_log.profile_counters)
	{
		if(a.first >= treelet_range.first && a.first < treelet_range.second)
		{
			uint treelet_id = (a.first - (paddr_t)kernel_args.treelets) / rtm::PackedTreelet::SIZE;
			paddr_t treelet_addr = (paddr_t)kernel_args.treelets + treelet_id * rtm::PackedTreelet::SIZE;
			rtm::PackedTreelet::Header header;
			dram.direct_read(&header, sizeof(rtm::PackedTreelet::Header), treelet_addr);
			uint offset = a.first - treelet_addr;

			treelet_histos[header.depth][offset] += a.second;

			if(offset == 64)
				treelet_counts[header.depth]++;
		}
	}

	for(uint i = 0; i < 16; ++i)
	{
		if(treelet_counts[i])
		{
			printf("Depth %d (%d)\n", i, treelet_counts[i]);
			uint64_t total = 0;
			total += treelet_histos[i][64];
			for(auto& a : treelet_histos[i])
			{
				printf("\t%6d:%.2f(%.2f%%)\n", a.first / 64, (double)a.second / treelet_counts[i], 100.0 * a.second / total);
			}
			printf("\n");
		}
	}
#endif
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
