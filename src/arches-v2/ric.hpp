#pragma once

#include "shared-utils.hpp"
#include "units/ric/unit-tp.hpp"
#include "units/ric/unit-treelet-rt-core.hpp"
#include "units/ric/unit-ray-coalescer.hpp"
#include "units/dual-streaming/unit-hit-record-updater.hpp"


namespace Arches {

namespace ISA { namespace RISCV { namespace RIC {

//see the opcode map for details
const static InstructionInfo isa_custom0_000_imm[8] =
{
	InstructionInfo(0x0, "fchthrd", InstrType::CUSTOM0, Encoding::U, RegFile::INT, MEM_REQ_DECL
	{
		MemoryRequest req;
		req.type = MemoryRequest::Type::FCHTHRD;
		req.size = sizeof(uint32_t);
		req.dst.push(DstReg(instr.rd, RegType::UINT32).u9, 9);
		req.vaddr = 0x0ull;
		return req;
	}),
	InstructionInfo(0x1, IMPL_NONE),
	InstructionInfo(0x2, IMPL_NONE),
};

const static InstructionInfo isa_custom0_funct3[8] =
{
	InstructionInfo(0x0, META_DECL{return isa_custom0_000_imm[instr.u.imm_31_12 >> 3]; }),
	InstructionInfo(0x1, IMPL_NONE),
	InstructionInfo(0x2, "swi", InstrType::CUSTOM4, Encoding::S, RegFile::FLOAT, RegFile::INT, MEM_REQ_DECL
	{
		//store bucket ray to hit record updater
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(uint32_t);
		mem_req.vaddr = unit->int_regs->registers[instr.s.rs1].u64 + s_imm(instr);

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(uint32_t) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.s.rs2 + i].f32;

		return mem_req;
	}),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, "lhit", InstrType::CUSTOM6, Encoding::I, RegFile::FLOAT, RegFile::INT, MEM_REQ_DECL
	{
		//load hit record into registers [rd - (rd + N)]
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::LOAD;
		mem_req.size = sizeof(rtm::Hit);
		mem_req.dst.push(DstReg(instr.rd, RegType::FLOAT32).u9, 9);
		mem_req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

		return mem_req;
	}),
	InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::I, RegFile::FLOAT, MEM_REQ_DECL
	{
		//load hit record into registers [rd - (rd + N)]
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::LOAD;
		mem_req.size = sizeof(rtm::Hit);
		mem_req.dst.push(DstReg(instr.rd, RegType::FLOAT32).u9, 9);
		mem_req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

		return mem_req;
	}),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3];});

}}}

namespace RIC {

#include "ric-kernel/include.hpp"
#include "ric-kernel/intersect.hpp"

typedef Units::UnitCache UnitL1Cache;
typedef Units::UnitCache UnitL2Cache;
typedef Units::UnitDRAMRamulator UnitDRAM;
#if	DS_USE_COMPRESSED_WIDE_BVH
typedef rtm::CompressedWideTreeletBVH::Treelet SceneSegment;
#else
typedef rtm::WideTreeletBVH::Treelet SceneSegment;
#endif

static RICKernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, const SimulationConfig& sim_config, uint page_size)
{
	std::string scene_name = sim_config.get_string("scene_name");
	std::string project_folder = get_project_folder_path();
	std::string scene_file = project_folder + "datasets\\" + scene_name + ".obj";
	std::string bvh_cache_filename = project_folder + "datasets\\cache\\" + scene_name + ".bvh";

	RICKernelArgs args;
	args.framebuffer_width = sim_config.get_int("framebuffer_width");
	args.framebuffer_height = sim_config.get_int("framebuffer_height");
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	heap_address = align_to(page_size, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	args.camera = sim_config.camera;

	rtm::Mesh mesh(scene_file);
	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2(bvh_cache_filename, build_objects);
	mesh.reorder(build_objects);

	args.pregen_rays = sim_config.get_int("pregen_rays");

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, bvh2, mesh, sim_config.get_int("pregen_bounce"), rays);
	
	std::vector<MinRayState> ray_states(args.framebuffer_size);
	for(uint i = 0; i < ray_states.size(); ++i)
	{
		ray_states[i].ray = rays[i];
		ray_states[i].hit.bc = rtm::vec2(0.0f);
		ray_states[i].hit.t = T_MAX;
		ray_states[i].hit.id = ~0u;
	}

	args.ray_states = write_vector(main_memory, CACHE_BLOCK_SIZE, ray_states, heap_address);

	rtm::WBVH wbvh(bvh2, build_objects);
	mesh.reorder(build_objects);

	rtm::NVCWBVH cwbvh(wbvh);

	rtm::CompressedWideTreeletBVH cwtbvh(cwbvh, mesh);
	args.treelets = write_vector(main_memory, page_size, cwtbvh.treelets, heap_address);
	args.num_treelets = cwtbvh.treelets.size();

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);

	main_memory->direct_write(&args, sizeof(RICKernelArgs), RIC_KERNEL_ARGS_ADDRESS);
	return args;
}

static void run_sim_ric(const SimulationConfig& sim_config)
{
	std::string project_folder = get_project_folder_path();
	
#if 1 //Modern config
	double clock_rate = 2.0e9;
	uint num_threads = sim_config.get_int("num_threads");
	uint num_tps = sim_config.get_int("num_tps");
	uint num_tms = sim_config.get_int("num_tms");
	uint num_rtc = sim_config.get_int("num_rt_cores");
	uint64_t stack_size = 512;

	//Memory
	uint64_t block_size = CACHE_BLOCK_SIZE;
	uint num_partitions = 16;
	uint partition_stride = 1 << 10; 
	uint64_t partition_mask = generate_nbit_mask(log2i(num_partitions)) << log2i(partition_stride);

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = project_folder + "build\\src\\arches-v2\\config-files\\gddr6_pch_config.yaml";
	dram_config.size = 4ull << 30; //4GB
	dram_config.num_controllers = num_partitions;
	dram_config.partition_stride = partition_mask;

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.in_order = sim_config.get_int("l2_in_order");
	l2_config.level = 2;
	l2_config.block_size = block_size;
	l2_config.size = sim_config.get_int("l2_size");
	l2_config.associativity = sim_config.get_int("l2_associativity");
	l2_config.num_slices = num_partitions;
	l2_config.slice_select_mask = partition_mask;
	l2_config.num_banks = 2;
	l2_config.bank_select_mask = generate_nbit_mask(log2i(l2_config.num_banks)) << log2i(block_size);
	l2_config.crossbar_width = 32;
	l2_config.num_mshr = 192;
	l2_config.rob_size = 4 * l2_config.num_mshr / l2_config.num_banks;
	l2_config.latency = 170;

	UnitL2Cache::PowerConfig l2_power_config;
	l2_power_config.leakage_power = 184.55e-3f * l2_config.num_banks;
	l2_power_config.tag_energy = 0.00756563e-9f;
	l2_power_config.read_energy = 0.378808e-9f - l2_power_config.tag_energy;
	l2_power_config.write_energy = 0.365393e-9f - l2_power_config.tag_energy;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.in_order = sim_config.get_int("l1_in_order");
	l1d_config.level = 1;
	l1d_config.block_size = block_size;
	l1d_config.size = sim_config.get_int("l1_size");
	l1d_config.associativity = sim_config.get_int("l1_associativity");
	l1d_config.num_banks = 4;
	l1d_config.bank_select_mask = generate_nbit_mask(log2i(l1d_config.num_banks)) << log2i(block_size);
	l1d_config.crossbar_width = 4;
	l1d_config.num_mshr = 256;
	l1d_config.rob_size = 8 * l1d_config.num_mshr / l1d_config.num_banks;
	l1d_config.latency = 30;

	UnitL1Cache::PowerConfig l1d_power_config;
	l1d_power_config.leakage_power = 7.19746e-3f * l1d_config.num_banks * num_tms;
	l1d_power_config.tag_energy = 0.000663943e-9f;
	l1d_power_config.read_energy = 0.0310981e-9f - l1d_power_config.tag_energy;
	l1d_power_config.write_energy = 0.031744e-9f - l1d_power_config.tag_energy;
#else //Legacy config

#endif

	_assert(block_size <= MemoryRequest::MAX_SIZE);
	_assert(block_size == CACHE_BLOCK_SIZE);

	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM3] = "LWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM4] = "SWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM5] = "CSHIT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM6] = "LHIT";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::RIC::custom0;

	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<Units::RIC::UnitTreeletRTCore<SceneSegment>*> rtcs;
	std::vector<UnitL1Cache*> l1ds;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms * num_rtc);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

	uint dram_ports_per_controller = 4;
	dram_config.num_ports = dram_ports_per_controller * dram_config.num_controllers;
	UnitDRAM dram(dram_config); 

	simulator.register_unit(&dram);
	simulator.new_unit_group();

	Units::UnitBuffer::Configuration sram_config;
	sram_config.latency = 1;
	sram_config.size = 1 << 30;
	sram_config.num_banks = num_partitions;
	sram_config.num_ports = dram_ports_per_controller * num_partitions;

	Units::UnitBuffer sram(sram_config);
	simulator.register_unit(&sram);
	simulator.new_unit_group();

	ELF elf(project_folder + "src\\ric-kernel\\riscv\\kernel");

	dram.clear();
	paddr_t heap_address = elf.load(dram._data_u8);
	RICKernelArgs kernel_args = initilize_buffers(&dram, heap_address, sim_config, partition_stride);
	heap_address = align_to(partition_stride * num_partitions, heap_address);
	std::pair<paddr_t, paddr_t> treelet_range = {(paddr_t)kernel_args.treelets, (paddr_t)kernel_args.treelets + kernel_args.num_treelets * sizeof(SceneSegment)};

	std::set<uint> unused_dram_ports;
	for(uint i = 0; i < dram_ports_per_controller; ++i)
		unused_dram_ports.insert(i);

	l2_config.num_ports = num_tms;
	l2_config.num_ports += 16 * l2_config.num_ports / l2_config.crossbar_width;
	l2_config.crossbar_width += 16;
	l2_config.mem_highers = {&dram};
	l2_config.mem_higher_port = *unused_dram_ports.begin();
	l2_config.mem_higher_port_stride = dram_ports_per_controller;
	unused_dram_ports.erase(*unused_dram_ports.begin());

	UnitL2Cache l2(l2_config);
	simulator.register_unit(&l2);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);
	simulator.new_unit_group();

	Units::RIC::UnitRayCoalescer::Configuration ray_coalescer_config;
	ray_coalescer_config.num_banks = 32;
	ray_coalescer_config.num_channels = num_partitions;
	ray_coalescer_config.traversal_scheme = sim_config.get_int("traversal_scheme");
	ray_coalescer_config.weight_scheme = sim_config.get_int("weight_scheme");
	ray_coalescer_config.num_tms = num_tms * num_rtc;
	ray_coalescer_config.block_size = block_size;
	ray_coalescer_config.row_size = partition_stride;
	ray_coalescer_config.num_root_rays = kernel_args.framebuffer_size;
	ray_coalescer_config.treelet_addr = *(paddr_t*)&kernel_args.treelets;
	ray_coalescer_config.heap_addr = *(paddr_t*)&heap_address;
	ray_coalescer_config.cheat_treelets = nullptr;// (rtm::WideTreeletBVH::Treelet*)&dram._data_u8[(size_t)kernel_args.treelets];
	ray_coalescer_config.max_active_segments_size = sim_config.get_int("max_active_set_size");
	ray_coalescer_config.l2_cache = &l2;
	ray_coalescer_config.main_mem = &dram;
	ray_coalescer_config.main_mem_port_stride = dram_ports_per_controller;
	ray_coalescer_config.main_mem_port_offset = *unused_dram_ports.begin();
	unused_dram_ports.erase(*unused_dram_ports.begin());

	if(sim_config.get_int("rays_on_chip"))
	{
		ray_coalescer_config.main_mem = &sram;
	}

	Units::RIC::UnitRayCoalescer ray_coalescer(ray_coalescer_config);
	simulator.register_unit(&ray_coalescer);

	l1d_config.num_ports = num_tps;
	//l1d_config.num_ports += 1 * l1d_config.num_ports / l1d_config.crossbar_width; //add extra port for RT core
	//l1d_config.crossbar_width += 1;
	l1d_config.num_ports += num_rtc * l1d_config.num_ports / l1d_config.crossbar_width; //add extra port for RT core
	l1d_config.crossbar_width += num_rtc;
	l1d_config.mem_highers = {&l2};
	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);
		std::vector<Units::UnitMemoryBase*> mem_list;
		std::vector<Units::UnitSFU*> sfu_list;

		l1d_config.mem_higher_port = tm_index;
		l1ds.push_back(_new UnitL1Cache(l1d_config));
		simulator.register_unit(l1ds.back());
		mem_list.push_back(l1ds.back());
		unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1ds.back();
		unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1ds.back();

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps, tm_index, &atomic_regs, 64));
		simulator.register_unit(thread_schedulers.back());
		mem_list.push_back(thread_schedulers.back());
		unit_table[(uint)ISA::RISCV::InstrType::ATOMIC] = thread_schedulers.back();
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

		//sfu_list.push_back(_new Units::UnitSFU(num_tps, 2, 1, num_tps));
		//simulator.register_unit(sfu_list.back());
		//unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
		//unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
		//unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(num_tps / 8, 1, 1, num_tps));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(num_tps / 16, 6, 1, num_tps));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

	#if DS_USE_HARDWARE_INTERSECTORS
		sfu_list.push_back(_new Units::UnitSFU(2, 3, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(1, 22, 8, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM2] = sfu_list.back();
	#endif

		for(auto& sfu : sfu_list)
			sfus.push_back(sfu);

		Units::RIC::UnitTreeletRTCore<SceneSegment>::Configuration rtc_config;
		rtc_config.early_t = sim_config.get_int("use_early");
		rtc_config.max_rays = 256 / num_rtc;
		rtc_config.num_tp = num_tps;
		rtc_config.treelet_base_addr = (paddr_t)kernel_args.treelets;
		rtc_config.ray_state_base_addr = (paddr_t)kernel_args.ray_states;
		rtc_config.cache = l1ds.back();
		rtc_config.ray_coalescer = &ray_coalescer;
		for(uint i = 0; i < num_rtc; ++i)
		{
			rtc_config.rtc_index = tm_index * num_rtc + i;
			rtc_config.cache_port = num_tps + i * 32;
			rtcs.push_back(_new Units::RIC::UnitTreeletRTCore<SceneSegment>(rtc_config));
			simulator.register_unit(rtcs.back());
			mem_list.push_back(rtcs.back());
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM4] = rtcs.back(); //SWI
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM6] = rtcs.back(); //LHIT
			unit_tables.emplace_back(unit_table);
		}

		sfu_lists.emplace_back(sfu_list);
		mem_lists.emplace_back(mem_list);

		Units::UnitTP::Configuration tp_config;
		tp_config.tm_index = tm_index;
		tp_config.stack_size = stack_size;
		tp_config.cheat_memory = dram._data_u8;
		tp_config.unique_mems = &mem_lists.back();
		tp_config.unique_sfus = &sfu_lists.back();
		for(uint tp_index = 0; tp_index < num_tps; ++tp_index)
		{
			tp_config.tp_index = tp_index;
			tp_config.num_threads = num_threads;
			tp_config.unit_table = &unit_tables[num_rtc * tm_index + tp_index * num_rtc / num_tps];
			tps.push_back(new Units::RIC::UnitTP(tp_config));
			simulator.register_unit(tps.back());
		}

		simulator.new_unit_group();
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

	Units::RIC::UnitTreeletRTCore<SceneSegment>::Log rtc_log;
	Units::RIC::UnitRayCoalescer::Log rc_log;

	uint delta = sim_config.get_int("logging_interval");
	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute(delta, [&]() -> void
	{
		float delta_us = delta / (clock_rate / 1'000'000);
		float delta_ns = delta / (clock_rate / 1'000'000'000);

		Units::UnitTP::Log tp_delta_log = delta_log(tp_log, tps);

		Units::UnitBuffer::Log sram_delta_log = delta_log(sram_log, sram);
		UnitDRAM::Log dram_delta_log = delta_log(dram_log, dram);
		UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2);
		UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);

		Units::RIC::UnitTreeletRTCore<SceneSegment>::Log rtc_delta_log = delta_log(rtc_log, rtcs);
		Units::RIC::UnitRayCoalescer::Log rc_delta_log = delta_log(rc_log, ray_coalescer);

		printf("                               \n");
		printf("Cycle: %lld                    \n", simulator.current_cycle);
		printf(" Threads Launched: %8d         \n", atomic_regs.iregs[0]);
		printf(" Buckets Launched: %8lld       \n", rc_log.buckets_launched);
		printf("Segments Launched: %8lld       \n", rc_log.segments_launched);
		printf("                               \n");
		printf(" Ray Total: %8.1f bytes/cycle  \n", (float)(rc_delta_log.buckets_generated + rc_delta_log.buckets_launched) * RAY_BUCKET_SIZE / delta);
		printf(" Ray Write: %8.1f bytes/cycle  \n", (float)rc_delta_log.buckets_generated * RAY_BUCKET_SIZE / delta);
		printf("  Ray Read: %8.1f bytes/cycle  \n", (float)rc_delta_log.buckets_launched * RAY_BUCKET_SIZE / delta);
		printf(" Ray Write: %8.1f Grays/s      \n", (float)(rc_delta_log.buckets_generated) * Arches::Units::RIC::RayBucket::MAX_RAYS / delta_ns);
		printf("  Ray Read: %8.1f Grays/s      \n", (float)(rc_delta_log.buckets_launched) * Arches::Units::RIC::RayBucket::MAX_RAYS / delta_ns);
		printf("                               \n");
		printf("DRAM Total: %8.1f bytes/cycle  \n", (float)(dram_delta_log.bytes_read + dram_delta_log.bytes_written) / delta);
		printf("DRAM Write: %8.1f bytes/cycle  \n", (float)dram_delta_log.bytes_written / delta);
		printf("DRAM  Read: %8.1f bytes/cycle  \n", (float)dram_delta_log.bytes_read / delta);
		printf("                               \n");
		printf("  L2$ Read: %8.1f bytes/cycle  \n", (float)l2_delta_log.bytes_read / delta);
		printf(" L1d$ Read: %8.1f bytes/cycle  \n", (float)l1d_delta_log.bytes_read / delta);
		printf("                               \n");
		printf(" L2$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l2_delta_log.hits / l2_delta_log.get_total(), 100.0 * l2_delta_log.half_misses / l2_delta_log.get_total(), 100.0 * l2_delta_log.misses / l2_delta_log.get_total());
		printf("L1d$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l1d_delta_log.hits / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.half_misses / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.misses / l1d_delta_log.get_total());
		printf("                               \n");
		printf("GRays/s: %2.1f\n", rtc_delta_log.rays / delta_ns);
		printf("                               \n");
		tp_delta_log.print(tps.size());
		printf("                               \n");
		rtc_delta_log.print(rtcs.size());
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

	print_header("L2$");
	delta_log(l2_log, l2);
	l2_log.print(frame_cycles);
	total_power += l2_log.print_power(l2_power_config, frame_time);

	print_header("L1d$");
	delta_log(l1d_log, l1ds);
	l1d_log.print(frame_cycles);
	total_power += l1d_log.print_power(l1d_power_config, frame_time);

	print_header("TP");
	delta_log(tp_log, tps);
	tp_log.print(tps.size());

	print_header("Ray Coalescer");
	delta_log(rc_log, ray_coalescer);
	rc_log.print();

	if(!rtcs.empty())
	{
		print_header("RT Core");
		float duplication = (float)rtc_log.rays / kernel_args.framebuffer_size;
		printf("Ray duplication: %.2f\n", duplication);
		rtc_log.rays = kernel_args.framebuffer_size;
		delta_log(rtc_log, rtcs);
		rtc_log.print(rtcs.size());
	}

	float total_energy = total_power * frame_time;

	print_header("Performance Summary");
	printf("Cycles: %lld\n", frame_cycles);
	printf("Clock rate: %.0f MHz\n", clock_rate / 1'000'000.0);
	printf("Frame time: %.3g ms\n", frame_time * 1000.0);
	printf("Mrays/s: %.0f\n", (float)rc_log.rays / frame_time / 1'000'000);

	print_header("Power Summary");
	printf("Energy: %.2f mJ\n", total_energy * 1000.0);
	printf("Power: %.2f W\n", total_power);
	printf("Mrays/J: %.2f\n", (float)rc_log.rays / total_energy / 1'000'000);

	print_header("Simulation Summary");
	printf("Simulation rate: %.2f KHz\n", frame_cycles / simulation_time / 1000.0);
	printf("Simulation time: %.0f s\n", simulation_time);

	print_header("Treelet Histogram");

#if 1
	uint treelet_counts[16];
	std::map<uint, uint64_t> treelet_histos[16];

	for(uint i = 0; i < 16; ++i)
		treelet_counts[i] = 0;

	for(auto& a : l1d_log.profile_counters)
	{
		if(a.first >= treelet_range.first && a.first < treelet_range.second)
		{
			uint treelet_id = (a.first - (paddr_t)kernel_args.treelets) / rtm::CompressedWideTreeletBVH::Treelet::SIZE;
			paddr_t treelet_addr = (paddr_t)kernel_args.treelets + treelet_id * rtm::CompressedWideTreeletBVH::Treelet::SIZE;
			rtm::WideTreeletBVH::Treelet::Header header;
			dram.direct_read(&header, sizeof(rtm::WideTreeletBVH::Treelet::Header), treelet_addr);
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
	for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
	for(auto& rtc : rtcs) delete rtc;
}

}}
