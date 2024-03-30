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

namespace ISA { namespace RISCV { namespace DualStreaming {

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
		for (uint i = 0; i < sizeof(WorkItem) / sizeof(float); ++i)
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
		for (uint i = 0; i < sizeof(rtm::Hit) / sizeof(float); ++i)
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

namespace DualStreaming {
#include "dual-streaming-kernel/include.hpp"
std::pair<paddr_t, paddr_t> treelet_range;
static DualStreamingKernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, GlobalConfig global_config)
{
	std::cerr << "Dual Streaming:: Initializing buffers...\n";
	//std::string s = "san-miguel";
	//std::string s = "sponza";
	std::string s = scene_names[global_config.scene_id];

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);

	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());

	std::string filename = current_folder_path + "../../../../datasets/" + s + ".obj";
	std::string treelet_filename = current_folder_path + "../../../../datasets/" + s + "_treelets.cache";
	std::string triangle_filename = current_folder_path + "../../../../datasets/" + s + "_triangles.cache";

	std::ifstream inputTreelets(treelet_filename, std::ios::binary);
	std::ifstream inputTriangles(triangle_filename, std::ios::binary);

	rtm::PackedTreeletBVH treelet_bvh;
	std::vector<rtm::Triangle> tris;
	if (inputTreelets.is_open() && inputTriangles.is_open())
	{
		// Do not need to rebuild treelets every time
		printf("Loading packed treelets from %s\n", treelet_filename.c_str());
		rtm::PackedTreelet curr_tree;
		while (inputTreelets.read(reinterpret_cast<char*>(&curr_tree), sizeof(rtm::PackedTreelet)))
			treelet_bvh.treelets.push_back(curr_tree);
		printf("Loaded %zd packed treelets\n", treelet_bvh.treelets.size());

		printf("Loading triangles from %s\n", triangle_filename.c_str());
		rtm::Triangle cur_tri;
		while (inputTriangles.read(reinterpret_cast<char*>(&cur_tri), sizeof(rtm::Triangle)))
			tris.push_back(cur_tri);
		printf("Loaded %zd triangles\n", tris.size());
	}
	else
	{
		rtm::Mesh mesh(filename);
		rtm::BVH bvh;
		std::vector<rtm::BVH::BuildObject> build_objects;
		for (uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));
		bvh.build(build_objects);
		mesh.reorder(build_objects);
		mesh.get_triangles(tris);

		rtm::PackedTreeletBVH treelet_bvh(bvh, mesh);

		std::ofstream outputTreelets(treelet_filename, std::ios::binary);
		std::ofstream outputTriangles(triangle_filename, std::ios::binary);

		printf("Writing %zd packed treelets to %s\n", treelet_bvh.treelets.size(), treelet_filename.c_str());
		for (auto& t : treelet_bvh.treelets)
			outputTreelets.write(reinterpret_cast<const char*>(&t), sizeof(rtm::PackedTreelet));

		printf("Writing %zd triangles to %s\n", tris.size(), triangle_filename.c_str());
		for (auto& tt : tris)
			outputTriangles.write(reinterpret_cast<const char*>(&tt), sizeof(rtm::Triangle));
	}
	DualStreamingKernelArgs args;
	args.framebuffer_width = global_config.framebuffer_width;
	args.framebuffer_height = global_config.framebuffer_height;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;

	args.samples_per_pixel = 1;
	args.max_path_depth = 1;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	args.camera = global_config.scene_config.camera;

	args.use_early = global_config.use_early;
	args.hit_delay = global_config.hit_delay;
	args.use_secondary_rays = global_config.use_secondary_rays;
	args.weight_scheme = global_config.weight_scheme;


	std::cout << args.weight_scheme << '\n';
	heap_address = align_to(ROW_BUFFER_SIZE, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	std::vector<rtm::Hit> hits(args.framebuffer_size);
	for (auto& hit : hits) hit.t = T_MAX;
	args.hit_records = write_vector(main_memory, ROW_BUFFER_SIZE, hits, heap_address);
	treelet_range.first = heap_address;
	args.treelets = write_vector(main_memory, ROW_BUFFER_SIZE, treelet_bvh.treelets, heap_address);
	treelet_range.second = heap_address;
	args.triangles = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);
	args.secondary_rays = write_vector(main_memory, CACHE_BLOCK_SIZE, Arches::secondary_rays, heap_address);

	main_memory->direct_write(&args, sizeof(DualStreamingKernelArgs), KERNEL_ARGS_ADDRESS);

	return args;
}

void print_header(std::string string, uint header_length = 80)
{
	uint spacers = string.length() < header_length ? header_length - string.length() : 0;
	printf("\n");
	for (uint i = 0; i < spacers / 2; ++i)
		printf("-");
	printf("%s", string.c_str());
	for (uint i = 0; i < (spacers + 1) / 2; ++i)
		printf("-");
	printf("\n");
}

static void run_sim_dual_streaming(GlobalConfig global_config)
{
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM3] = "LWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM4] = "SWI";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM5] = "CSHIT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM6] = "LHIT";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::DualStreaming::custom0;

	double clock_rate = 2'000'000'000.0;

	uint num_threads_per_tp = 8;
	uint num_tps_per_tm = 64;
	uint num_tms = 64;

	uint num_l2_banks = 32;
	uint num_l1_banks = 8;
	uint num_icache_per_tm = 8;

	uint num_tps = num_tps_per_tm * num_tms;
	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;
	uint num_tps_per_i_cache = num_tps_per_tm / num_icache_per_tm;

	//hardware spec
	uint64_t mem_size = 4ull * 1024 * 1024 * 1024; //4GB
	uint64_t l2_size = 32ull * 1024 * 1024;
	uint64_t l1_size = 128ull * 1024;
	uint64_t stack_size = 1ull * 1024; //1KB

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::DualStreaming::UnitRayStagingBuffer*> rsbs;
	std::vector<Units::DualStreaming::UnitTreeletRTCore*> rtcs;
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

	Units::UnitDRAM dram(6 * NUM_DRAM_CHANNELS, mem_size, &simulator); dram.clear();
	simulator.register_unit(&dram);

	simulator.new_unit_group();

	ELF elf(current_folder_path + "../../dual-streaming-kernel/riscv/kernel");
	paddr_t heap_address = dram.write_elf(elf);

	DualStreamingKernelArgs kernel_args = DualStreaming::initilize_buffers(&dram, heap_address, global_config);

	Units::DualStreaming::UnitSceneBuffer::Configuration scene_buffer_config;
	scene_buffer_config.size = 4 * 1024 * 1024; // 4MB
	scene_buffer_config.num_ports = num_tms * num_l1_banks;
	scene_buffer_config.main_mem = &dram;
	scene_buffer_config.main_mem_port_offset = 1;
	scene_buffer_config.main_mem_port_stride = 6;
	scene_buffer_config.num_banks = 32;
	scene_buffer_config.bank_select_mask = 0b0000'0111'1100'0000ull;
	scene_buffer_config.segment_start = treelet_range.first;
	scene_buffer_config.segment_size = sizeof(rtm::PackedTreelet);
	scene_buffer_config.allow_wait = global_config.allow_wait;
	Units::DualStreaming::UnitSceneBuffer scene_buffer(scene_buffer_config);
	simulator.register_unit(&scene_buffer);

	Units::DualStreaming::UnitStreamScheduler::Configuration stream_scheduler_config;
	stream_scheduler_config.treelet_addr = *(paddr_t*)&kernel_args.treelets;
	stream_scheduler_config.heap_addr = *(paddr_t*)&heap_address;
	stream_scheduler_config.num_tms = num_tms;
	stream_scheduler_config.num_banks = 16;
	stream_scheduler_config.cheat_treelets = (rtm::PackedTreelet*)&dram._data_u8[(size_t)kernel_args.treelets];
	stream_scheduler_config.main_mem = &dram;
	stream_scheduler_config.main_mem_port_offset = 2;
	stream_scheduler_config.main_mem_port_stride = 6;
	stream_scheduler_config.traversal_scheme = global_config.traversal_scheme;
	stream_scheduler_config.num_root_rays = kernel_args.framebuffer_size;
	stream_scheduler_config.max_active_segments = scene_buffer_config.size / sizeof(rtm::PackedTreelet);
	stream_scheduler_config.scene_buffer = &scene_buffer;
	if (global_config.use_secondary_rays) stream_scheduler_config.num_root_rays = global_config.valid_secondary_rays;
	stream_scheduler_config.weight_scheme = global_config.weight_scheme;
	Units::DualStreaming::UnitStreamScheduler stream_scheduler(stream_scheduler_config);
	simulator.register_unit(&stream_scheduler);

	Units::DualStreaming::UnitHitRecordUpdater::Configuration hit_record_updater_config;
	hit_record_updater_config.num_tms = num_tms;
	hit_record_updater_config.main_mem = &dram;
	hit_record_updater_config.main_mem_port_offset = 5;
	hit_record_updater_config.main_mem_port_stride = 6;
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
	l2_config.size = l2_size;
	l2_config.associativity = 8;
	l2_config.num_ports = num_tms * num_l1_banks;
	l2_config.num_banks = num_l2_banks;
	l2_config.cross_bar_width = 16;
	l2_config.bank_select_mask = 0b0001'1110'0000'0100'0000ull; //The high order bits need to match the channel assignment bits
	l2_config.latency = 10;
	l2_config.cycle_time = 2;
	l2_config.mem_higher = &dram;
	l2_config.mem_higher_port_offset = 0;
	l2_config.mem_higher_port_stride = 3;

	Units::UnitBlockingCache l2(l2_config);
	simulator.register_unit(&l2);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);

	for (uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		simulator.new_unit_group();
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);

		std::vector<Units::UnitMemoryBase*> mem_list;

		Units::UnitNonBlockingCache::Configuration l1_config;
		l1_config.size = l1_size;
		l1_config.associativity = 4;
		l1_config.num_ports = num_tps_per_tm;
#ifdef USE_RT_CORE
		l1_config.num_ports += 1; //add extra port for RT core
#endif
		l1_config.num_banks = num_l1_banks;
		l1_config.cross_bar_width = num_l1_banks;
		l1_config.bank_select_mask = generate_nbit_mask(log2i(num_l1_banks)) << log2i(CACHE_BLOCK_SIZE);
		l1_config.latency = 1;
		l1_config.num_lfb = 16;
		l1_config.check_retired_lfb = false;
		l1_config.mem_higher = &l2;
		l1_config.mem_higher_port_offset = l1_config.num_banks * tm_index;
		l1_config.scene_buffer = &scene_buffer;
		l1_config.treelet_range = treelet_range;

		l1s.push_back(new Units::UnitNonBlockingCache(l1_config));
		simulator.register_unit(l1s.back());
		mem_list.push_back(l1s.back());

		unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1s.back();
		unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1s.back();

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, kernel_args.framebuffer_width, kernel_args.framebuffer_height));
		simulator.register_unit(thread_schedulers.back());
		mem_list.push_back(thread_schedulers.back());

		unit_table[(uint)ISA::RISCV::InstrType::ATOMIC] = thread_schedulers.back();
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

#ifdef USE_RT_CORE
		rsbs.push_back(_new Units::DualStreaming::UnitRayStagingBuffer(1, tm_index, &stream_scheduler, &hit_record_updater));
		simulator.register_unit(rsbs.back());

		Units::DualStreaming::UnitTreeletRTCore::Configuration rtc_config;
		rtc_config.max_rays = 128;
		rtc_config.num_tp = num_tps_per_tm;
		rtc_config.treelet_base_addr = (paddr_t)kernel_args.treelets;
		rtc_config.hit_record_base_addr = (paddr_t)kernel_args.hit_records;
		rtc_config.use_early_termination = global_config.use_early;
		rtc_config.cache = l1s.back();
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

		std::vector<Units::UnitSFU*> sfu_list;

		sfu_list.push_back(_new Units::UnitSFU(16, 2, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(2, 1, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(1, 1, 16, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(2, 3, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(1, 22, 8, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM2] = sfu_list.back();

		for (auto& sfu : sfu_list)
			sfus.push_back(sfu);

		unit_tables.emplace_back(unit_table);
		sfu_lists.emplace_back(sfu_list);
		mem_lists.emplace_back(mem_list);

		for (uint tp_index = 0; tp_index < num_tps_per_tm; ++tp_index)
		{
			Units::UnitTP::Configuration tp_config;
			tp_config.num_threads = 2;
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
			tp_config.num_tps_per_i_cache = 4;

			tps.push_back(new Units::DualStreaming::UnitTP(tp_config));
			simulator.register_unit(tps.back());
			simulator.units_executing++;
		}
	}
	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute();
	auto stop = std::chrono::high_resolution_clock::now();

	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
	double frame_time = simulator.current_cycle / clock_rate;

	Units::UnitNonBlockingCache::Log l1_log;
	for (auto& l1 : l1s)
		l1_log.accumulate(l1->log);

	Units::UnitTP::Log tp_log(0x10000);
	for (auto& tp : tps)
		tp_log.accumulate(tp->log);

	Units::DualStreaming::UnitTreeletRTCore::Log rtc_log;
	for (auto& rtc : rtcs)
		rtc_log.accumulate(rtc->log);

	//tp_log.print_profile(mm._data_u8);

	print_header("DRAM");
	dram.print_usimm_stats(CACHE_BLOCK_SIZE, 4, simulator.current_cycle);

	print_header("Stream Scheduler");
	stream_scheduler.log.print();

	print_header("L2$");
	l2.log.print_log(simulator.current_cycle);

	print_header("L1d$");
	l1_log.print(simulator.current_cycle);

	print_header("TP");
	tp_log.print();

	print_header("RT Core");
	rtc_log.print_log(simulator.current_cycle);

	print_header("Performance Summary");
	//printf("Clock rate: %.0fMHz\n", clock_rate / 1'000'000.0);
	printf("Frame time: %.2fms\n", frame_time * 1000.0);
	printf("Cycles: %lld\n", simulator.current_cycle);
	printf("MRays/s: %.0f\n", (float)kernel_args.framebuffer_size / frame_time / (1 << 20));

	print_header("Simulation Summary");
	printf("Simulation rate: %.1fKHz\n", simulator.current_cycle / simulation_time / 1000.0);
	printf("Simulation time: %.1fs\n", simulation_time);

	paddr_t paddr_frame_buffer = reinterpret_cast<paddr_t>(kernel_args.framebuffer);

	std::string scene_name = scene_names[global_config.scene_id];
	dram.dump_as_png_uint8(paddr_frame_buffer, kernel_args.framebuffer_width, kernel_args.framebuffer_height, scene_name + "_dual_out.png");

	for (auto& tp : tps) delete tp;
	for (auto& rtc : rtcs) delete rtc;
	for (auto& sfu : sfus) delete sfu;
	for (auto& rsb : rsbs) delete rsb;
	for (auto& ts : thread_schedulers) delete ts;
	for (auto& l1 : l1s) delete l1;
}
}
}