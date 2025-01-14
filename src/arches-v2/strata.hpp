#pragma once
#include "shared-utils.hpp"
#include "units/strata/unit-tp.hpp"
#include "units/strata/unit-treelet-rt-core.hpp"

namespace Arches {

namespace ISA { namespace RISCV { namespace STRaTA {

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
	InstructionInfo(0x1, IMPL_NONE),
	InstructionInfo(0x2, "swi", InstrType::CUSTOM4, Encoding::S, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		// store ray buffer
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(RayData);
		mem_req.vaddr = unit->int_regs->registers[instr.s.rs1].u64 + s_imm(instr);

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(RayData) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.s.rs2 + i].f32;

		return mem_req;
	}),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, "lhit", InstrType::CUSTOM6, Encoding::I, RegType::FLOAT, RegType::INT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		//load hit record into registers [rd - (rd + N)]
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::LOAD;
		mem_req.size = sizeof(STRaTAHitReturn);
		mem_req.dst = reg_addr.u8;
		mem_req.vaddr = unit->int_regs->registers[instr.i.rs1].u64 + i_imm(instr);

		return mem_req;
	}),
	InstructionInfo(0x5, IMPL_NONE),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3];});

}
}
}

namespace STRaTA {

typedef Units::UnitNonBlockingCache UnitL1Cache;
typedef Units::UnitNonBlockingCache UnitL2Cache;

#include "strata-kernel/include.hpp"
#include "strata-kernel/intersect.hpp"
static STRaTAKernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, GlobalConfig global_config, uint page_size, uint raybuffer_size)
{
	std::string scene_name = scene_names[global_config.scene_id];

	TCHAR tc_exe_path[MAX_PATH];
	GetModuleFileName(NULL, tc_exe_path, MAX_PATH);
	std::wstring w_exe_path(tc_exe_path);
	std::string exe_path(w_exe_path.begin(), w_exe_path.end());

	std::string poject_folder = exe_path.substr(0, exe_path.rfind("build"));
	std::string data_folder = poject_folder + "datasets/";

	printf("%s\n", poject_folder.c_str());

	std::string filename = data_folder + scene_name + ".obj";
	std::string bvh_cache_filename = data_folder + "cache/" + scene_name + "_bvh.cache";

	STRaTAKernelArgs args;
	args.framebuffer_width = global_config.framebuffer_width;
	args.framebuffer_height = global_config.framebuffer_height;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	heap_address = align_to(page_size, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	std::vector<rtm::Hit> hits(args.framebuffer_size, {T_MAX, rtm::vec2(0.0), ~0u});
	args.hit_records = write_vector(main_memory, page_size, hits, heap_address);

	args.raybuffer_size = raybuffer_size;
	args.max_init_ray = std::min(args.raybuffer_size / (uint32_t)sizeof(RayData), args.framebuffer_size);

	// secondary rays only
	args.pregen_rays = global_config.pregen_rays;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, global_config.camera_config.focal_length, global_config.camera_config.position, global_config.camera_config.target);

	rtm::Mesh mesh(filename);
	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2(bvh_cache_filename, build_objects);
	mesh.reorder(build_objects);

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, bvh2, mesh, global_config.pregen_bounce, rays);

#ifdef USE_COMPRESSED_WIDE_BVH
	rtm::CompressedWideBVH cwbvh(bvh2);
	mesh.reorder(cwbvh.indices);
	args.nodes = write_vector(main_memory, CACHE_BLOCK_SIZE, cwbvh.nodes, heap_address);
#else
	rtm::WideBVH wbvh(bvh2, build_objects);
	mesh.reorder(build_objects);
	rtm::WideTreeletSTRaTABVH wtbvh(wbvh, mesh);
	args.treelets = write_vector(main_memory, page_size, wtbvh.treelets, heap_address);
#endif

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);
	args.rays = write_vector(main_memory, CACHE_BLOCK_SIZE, rays, heap_address);
	main_memory->direct_write(&args, sizeof(STRaTAKernelArgs), KERNEL_ARGS_ADDRESS);
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

static void run_sim_strata(GlobalConfig global_config)
{
	//hardware spec
	double clock_rate = 2'000'000'000.0;

#if 1 //Modern config

	//Compute
	uint64_t stack_size = 1ull << 10; //1KB
	uint num_threads_per_tp = 4;
	uint num_tps_per_tm = 128;
	uint num_tms = 128;

	//DRAM 
	uint64_t mem_size = 1ull << 32; //4GB
	uint num_channels = 16;// dram.num_channels();
	uint64_t row_size = 8 * 1024; // dram.row_size();
	uint64_t block_size = 64; // dram.block_size();

#if 1
	typedef Units::UnitDRAMRamulator UnitDRAM;
	UnitDRAM dram(32, num_channels, mem_size); dram.clear();
#else
	typedef Units::UnitDRAM UnitDRAM;
	UnitDRAM::init_usimm("gddr5_16ch.cfg", "1Gb_x16_amd2GHz.vi");
	UnitDRAM dram(64, mem_size);
#endif

	_assert(block_size <= MemoryRequest::MAX_SIZE);
	_assert(block_size == CACHE_BLOCK_SIZE);
	//_assert(row_size == DRAM_ROW_SIZE);

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.size = 512ull * 1024; //512KB for treelet data
	l2_config.block_size = block_size;
	l2_config.num_mshr = 256;
	l2_config.associativity = 8;
	l2_config.latency = 200;
	l2_config.cycle_time = 1;
	l2_config.num_banks = 32;
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
	l1d_config.size = 128ull * 1024; //128KB
	l1d_config.block_size = block_size;
	l1d_config.associativity = 4;
	l1d_config.latency = 30;
	l1d_config.num_banks = 8;
	l1d_config.bank_select_mask = generate_nbit_mask(log2i(l1d_config.num_banks)) << log2i(block_size);
	l1d_config.num_mshr = num_mshr / l1d_config.num_banks;

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
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM4] = "SWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM6] = "LHIT";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::STRaTA::custom0;

	uint num_tps = num_tps_per_tm * num_tms;
	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;
	uint num_tps_per_i_cache = num_tps_per_tm / num_icache_per_tm;
	uint num_l2_ports_per_tm = l1d_config.num_banks * 2;

	Simulator simulator;
	std::vector<Units::STRaTA::UnitTP*> tps;

	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;

#ifdef USE_COMPRESSED_WIDE_BVH
	typedef Units::STRaTA::UnitTreeletRTCore<rtm::CompressedWideBVH> UnitRTCore;
#else
	typedef Units::STRaTA::UnitTreeletRTCore UnitRTCore;
#endif
	std::vector<UnitRTCore*> rtcs;

	std::vector<UnitL1Cache*> l1ds;
	std::vector<Units::UnitBlockingCache*> l1is;

	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

	simulator.register_unit(&dram);
	simulator.new_unit_group();

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());
	ELF elf(current_folder_path + "../../strata-kernel/riscv/kernel");
	paddr_t heap_address = dram.write_elf(elf);

	STRaTAKernelArgs kernel_args = initilize_buffers(&dram, heap_address, global_config, row_size, 4ull * 1024 * 1024);

	l2_config.num_ports = num_tms * num_l2_ports_per_tm;
	l2_config.mem_highers = {&dram};
	l2_config.mem_higher_port_offset = 0;
	l2_config.mem_higher_port_stride = 1;

	UnitL2Cache l2(l2_config);
	simulator.register_unit(&l2);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);

	Units::STRaTA::UnitRayStreamBuffer::Configuration ray_stream_buffer_config;
	ray_stream_buffer_config.latency = l2_config.latency + l1d_config.latency;
	ray_stream_buffer_config.num_banks = 64;
	ray_stream_buffer_config.num_tm = num_tms;
	ray_stream_buffer_config.size = 4ull * 1024 * 1024; //4MB
	Units::STRaTA::UnitRayStreamBuffer ray_stream_buffer(ray_stream_buffer_config);
	simulator.register_unit(&ray_stream_buffer);

	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		simulator.new_unit_group();
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);

		std::vector<Units::UnitMemoryBase*> mem_list;

		l1d_config.num_ports = num_tps_per_tm;
	#ifdef USE_RT_CORE
		l1d_config.num_ports += 1; //add extra port for RT core
	#endif
		l1d_config.mem_highers = {&l2};
		l1d_config.mem_higher_port_offset = num_l2_ports_per_tm * tm_index;
		l1d_config.mem_higher_port_stride = 2;

		l1ds.push_back(new UnitL1Cache(l1d_config));
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
			Units::UnitBlockingCache* i_l1 = new Units::UnitBlockingCache(l1i_config);
			l1is.push_back(i_l1);
			simulator.register_unit(l1is.back());
		}

	#ifdef USE_RT_CORE
		UnitRTCore::Configuration rtc_config;
		rtc_config.max_rays = 256;
		rtc_config.num_tp = num_tps_per_tm;
		rtc_config.tm_index = tm_index;
		rtc_config.treelet_base_addr = (paddr_t)kernel_args.treelets;
		rtc_config.hit_record_base_addr = (paddr_t)kernel_args.hit_records;
		rtc_config.cache = l1ds.back();
		rtc_config.ray_stream_buffer = &ray_stream_buffer;

		rtcs.push_back(_new  UnitRTCore(rtc_config));
		simulator.register_unit(rtcs.back());
		mem_list.push_back(rtcs.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM4] = rtcs.back(); //SWI
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM6] = rtcs.back(); //LHIT
	#endif

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, 64));
		simulator.register_unit(thread_schedulers.back());
		mem_list.push_back(thread_schedulers.back());
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

			tps.push_back(new Units::STRaTA::UnitTP(tp_config));
			simulator.register_unit(tps.back());
		}
	}

	for(auto& tp : tps)
		tp->set_entry_point(elf.elf_header->e_entry.u64);

	//master logs
	UnitDRAM::Log dram_log;
	UnitL2Cache::Log l2_log;
	UnitL1Cache::Log l1d_log;
	Units::UnitBlockingCache::Log l1i_log;
	Units::UnitTP::Log tp_log;

	UnitRTCore::Log rtc_log;
	Units::STRaTA::UnitRayStreamBuffer::Log rsb_log;

	uint delta = global_config.logging_interval;
	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute(delta, [&]() -> void
	{
		float epsilon_ns = delta / (clock_rate / 1'000'000'000);

		UnitDRAM::Log dram_delta_log = delta_log(dram_log, dram);
		UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2);
		UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);
		Units::STRaTA::UnitRayStreamBuffer::Log rsb_delta_log = delta_log(rsb_log, ray_stream_buffer);
		UnitRTCore::Log rtc_delta_log = delta_log(rtc_log, rtcs);

		printf("                            \n");
		printf("Cycle: %lld                 \n", simulator.current_cycle);
		printf("Threads Launched: %d        \n", atomic_regs.iregs[0] * 64);
		printf("                            \n");
		printf("DRAM Read: %8.1f bytes/cycle\n", (float)dram_delta_log.bytes_read / delta);
		printf(" L2$ Read: %8.1f bytes/cycle\n", (float)l2_delta_log.bytes_read / delta);
		printf("L1d$ Read: %8.1f bytes/cycle\n", (float)l1d_delta_log.bytes_read / delta);
		printf("RSB$ Read: %8.1f bytes/cycle\n", (float)rsb_delta_log.bytes_read / delta);
		printf("RSB$ Write: %7.1f bytes/cycle\n", (float)rsb_delta_log.bytes_written / delta);
		printf("RSB$ Req Push: %d\n", rsb_delta_log.ray_request_push_count);
		printf("RSB$ Req Pop: %d\n", rsb_delta_log.ray_request_pop_count);
		printf("RTC Rays: %d\n", rtc_delta_log.rays);
		printf("RTC Store Rays: %d\n", rtc_delta_log.store_rays);
		printf("RTC Hits: %d\n", rtc_delta_log.hits);
		printf("RTC Get Hits: %d\n", rtc_delta_log.get_hits);
		printf("RTC Load Hits: %d\n", rtc_delta_log.load_hits);
		printf("RTC Return Hits: %d\n", rtc_delta_log.return_hits);
		printf("                            \n");
		printf(" L2$ Hit Rate: %8.1f%%\n", 100.0 * (l2_delta_log.hits + l2_delta_log.half_misses) / l2_delta_log.get_total());
		printf("L1d$ Hit Rate: %8.1f%%\n", 100.0 * (l1d_delta_log.hits + l1d_delta_log.half_misses) / l1d_delta_log.get_total());
		printf("                             \n");
	});
	auto stop = std::chrono::high_resolution_clock::now();

	cycles_t frame_cycles = simulator.current_cycle;
	double frame_time = frame_cycles / clock_rate;
	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;

	tp_log.print_profile(dram._data_u8);

	dram.print_stats(4, frame_cycles);
	print_header("DRAM");
	delta_log(dram_log, dram);
	dram_log.print(frame_cycles);
	float total_power = dram.total_power();

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
	printf("Cycles: %lld\n", simulator.current_cycle);
	printf("Clock rate: %.0f MHz\n", clock_rate / 1'000'000.0);
	printf("Frame time: %.3g ms\n", frame_time * 1000.0);
	if(!rtcs.empty()) printf("MRays/s: %.0f\n", rtc_log.rays / frame_time / 1'000'000.0);
	else              printf("MRays/s: %.0f\n", kernel_args.framebuffer_size / frame_time / 1'000'000.0);

	print_header("Power Summary");
	printf("Energy: %.2f mJ\n", total_power * frame_time * 1000.0);
	printf("Power: %.2f W\n", total_power);
	if(!rtcs.empty()) printf("MRays/J: %.2f\n", rtc_log.rays / total_energy / 1'000'000.0);
	else              printf("MRays/J: %.2f\n", kernel_args.framebuffer_size / total_energy / 1'000'000.0);

	print_header("Simulation Summary");
	printf("Simulation rate: %.2f KHz\n", simulator.current_cycle / simulation_time / 1000.0);
	printf("Simulation time: %.0f s\n", simulation_time);
	printf("MSIPS: %.2f\n", simulator.current_cycle * tps.size() / simulation_time / 1'000'000.0);

	stbi_flip_vertically_on_write(true);
	dram.dump_as_png_uint8((paddr_t)kernel_args.framebuffer, kernel_args.framebuffer_width, kernel_args.framebuffer_height, "out.png");

	for(auto& tp : tps) delete tp;
	for(auto& sfu : sfus) delete sfu;
	for(auto& l1d : l1ds) delete l1d;
	for(auto& l1i : l1is) delete l1i;
	for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
	for(auto& rtc : rtcs) delete rtc;
}
}
}