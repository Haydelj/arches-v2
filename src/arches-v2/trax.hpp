#pragma once
#include "shared-utils.hpp"
#include "units/trax/unit-tp.hpp"
#include "units/trax/unit-rt-core.hpp"
#include "units/trax/unit-treelet-rt-core.hpp"
namespace Arches {

namespace ISA { namespace RISCV { namespace TRaX {

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
		hit.t     = fr[17].f32;
		hit.bc[0] = fr[18].f32;
		hit.bc[1] = fr[19].f32;
		hit.id    = fr[20].u32;

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
	InstructionInfo(0x2, IMPL_NONE),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, IMPL_NONE),
	InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::I, RegType::FLOAT, MEM_REQ_DECL
	{
		RegAddr reg_addr;
		reg_addr.reg = instr.i.rd;
		reg_addr.reg_type = RegType::FLOAT;
		reg_addr.sign_ext = false;

		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(rtm::Ray);
		mem_req.dst = reg_addr.u8;
		mem_req.flags = (uint16_t)ISA::RISCV::i_imm(instr);
		mem_req.vaddr = 0xdeadbeefull;

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(rtm::Ray) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.i.rs1 + i].f32;

		return mem_req;
	}),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{ return isa_custom0_funct3[instr.i.funct3]; });

}}}

namespace TRaX {
#include "trax-kernel/include.hpp"
static TRaXKernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, GlobalConfig global_config)
{
	std::string s = scene_names[global_config.scene_id];
	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());

	std::string filename = current_folder_path + "../../../../datasets/" + s + ".obj";
	std::string bvh_filename = current_folder_path + "../../../../datasets/" + s + "_bvh.cache";
	std::string triangle_filename = current_folder_path + "../../../../datasets/" + s + "_triangles.cache";
	std::string treelet_filename = current_folder_path + "../../../../datasets/" + s + "_treelets.cache";
	std::ifstream inputBVH(bvh_filename, std::ios::binary);
	std::ifstream inputTriangles(triangle_filename, std::ios::binary);
	std::ifstream inputTreelets(triangle_filename, std::ios::binary);
	rtm::PackedBVH2 packed_bvh;
	std::vector<rtm::Triangle> tris;
	rtm::PackedTreeletBVH treelet_bvh;
	if (inputTreelets.is_open() && inputBVH.is_open() && inputTriangles.is_open())
	{
		printf("Loading packed bvh from %s\n", bvh_filename.c_str());
		rtm::PackedBVH2::Node curr_node;
		while (inputBVH.read(reinterpret_cast<char*>(&curr_node), sizeof(rtm::PackedBVH2::Node)))
			packed_bvh.nodes.push_back(curr_node);
		printf("Loaded %zd packed bvh nodes\n", packed_bvh.nodes.size());

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
	else {
		rtm::Mesh mesh(filename);
		rtm::BVH bvh;
		std::vector<rtm::BVH::BuildObject> build_objects;
		for (uint i = 0; i < mesh.size(); ++i)
			build_objects.push_back(mesh.get_build_object(i));
		bvh.build(build_objects);
		mesh.reorder(build_objects);
		mesh.get_triangles(tris);

		packed_bvh = rtm::PackedBVH2(bvh);
		treelet_bvh = rtm::PackedTreeletBVH(packed_bvh, mesh);
		std::ofstream outputBVH(bvh_filename, std::ios::binary);
		std::ofstream outputTriangles(triangle_filename, std::ios::binary);
		std::ofstream outputTreelets(treelet_filename, std::ios::binary);
		printf("Writing %zd packed bvh nodes to %s, %zd treelets to %s\n", packed_bvh.nodes.size(), bvh_filename.c_str(), treelet_bvh.treelets.size(), treelet_filename.c_str());
		for (auto& t : packed_bvh.nodes)
		{
			outputBVH.write(reinterpret_cast<const char*>(&t), sizeof(t));
		}
		for (auto& tt : tris)
		{
			outputTriangles.write(reinterpret_cast<const char*>(&tt), sizeof(tt));
		}
		for (auto& treelet : treelet_bvh.treelets)
		{
			outputTreelets.write(reinterpret_cast<const char*>(&treelet), sizeof(treelet));
		}
	}

	TRaXKernelArgs args;
	args.framebuffer_width = global_config.framebuffer_width;
	args.framebuffer_height = global_config.framebuffer_height;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;

	// secondary rays only
	args.use_secondary_rays = global_config.use_secondary_rays;

	heap_address = align_to(ROW_BUFFER_SIZE, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address); heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.samples_per_pixel = 1;
	args.max_depth = 1;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	args.camera = global_config.scene_config.camera;
	//global_data.camera = Camera(global_data.framebuffer_width, global_data.framebuffer_height, 24.0f, rtm::vec3(0.0f, 0.0f, 5.0f));

	args.nodes = write_vector(main_memory, CACHE_BLOCK_SIZE, packed_bvh.nodes, heap_address);
	args.tris = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);
	args.treelets = write_vector(main_memory, ROW_BUFFER_SIZE, treelet_bvh.treelets, heap_address);

	main_memory->direct_write(&args, sizeof(TRaXKernelArgs), KERNEL_ARGS_ADDRESS);

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

static void run_sim_trax(GlobalConfig global_config)
{
	double clock_rate = 2'000'000'000.0;

	uint num_threads_per_tp = 8;
	uint num_tps_per_tm = 64;
	uint num_tms_per_l2 = 64;
	uint num_l2 = 1;

	uint num_l2_banks = 32;
	uint num_l1_banks = 8;
	uint num_icache_per_tm = 8;

	uint num_tps = num_l2 * num_tms_per_l2 * num_tps_per_tm;
	uint num_tms = num_tms_per_l2 * num_l2;
	uint num_tps_per_i_cache = num_tps_per_tm / num_icache_per_tm;
	uint num_l2_ports_per_tm = 2 * num_l1_banks;
	uint sfu_table_size = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES);

	//hardware spec
	uint64_t mem_size = 4ull * 1024ull * 1024ull * 1024ull; //4GB

	//cached global data
	uint64_t stack_size = 1024; //1KB

	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::TRaX::custom0;
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM7] = "TRACERAY";

	Simulator simulator;

	std::vector<Units::TRaX::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitNonBlockingCache*> l1ds;
	std::vector<Units::UnitBlockingCache*> l1is;
	std::vector<Units::UnitBlockingCache*> l2s;
	std::vector<Units::TRaX::UnitRTCore*> rtcs;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);


	Units::UnitDRAM mm(num_l2 * num_l2_banks, 1024ull * 1024ull * 1024ull, &simulator); mm.clear();
	simulator.register_unit(&mm);

	TCHAR exePath[MAX_PATH];
	GetModuleFileName(NULL, exePath, MAX_PATH);
	std::wstring fullPath(exePath);
	std::wstring exeFolder = fullPath.substr(0, fullPath.find_last_of(L"\\") + 1);
	std::string current_folder_path(exeFolder.begin(), exeFolder.end());
	ELF elf(current_folder_path + "../../trax-kernel/riscv/kernel");
	//ELF elf("../trax-kernel/riscv/kernel");
	vaddr_t global_pointer;
	paddr_t heap_address = mm.write_elf(elf);

	TRaXKernelArgs kernel_args = initilize_buffers(&mm, heap_address, global_config);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);

	for (uint l2_index = 0; l2_index < num_l2; ++l2_index)
	{
		Units::UnitBlockingCache::Configuration l2_config;
		l2_config.size = 36 * 1024 * 1024;
		l2_config.associativity = 8;
		l2_config.latency = 10;
		l2_config.cycle_time = 2;
		l2_config.num_ports = num_l2_ports_per_tm * num_tms_per_l2;
		l2_config.num_banks = num_l2_banks;
		l2_config.cross_bar_width = 16;
		l2_config.bank_select_mask = 0b0001'1110'0000'0100'0000ull;
		l2_config.mem_higher = &mm;
		l2_config.mem_higher_port_offset = l2_index;
		l2_config.mem_higher_port_stride = num_l2;

		l2s.push_back(new Units::UnitBlockingCache(l2_config));

		for (uint tm_i = 0; tm_i < num_tms_per_l2; ++tm_i)
		{
			simulator.new_unit_group();
			uint tm_index = l2_index * num_tms_per_l2 + tm_i;
			if(tm_i == 0) simulator.register_unit(l2s.back());

			std::vector<Units::UnitSFU*> sfu_list;
			std::vector<Units::UnitMemoryBase*> mem_list;
			std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);

			Units::UnitNonBlockingCache::Configuration l1_config;
			l1_config.size = 128 * 1024;
			l1_config.associativity = 4;
			l1_config.latency = 1;
			l1_config.num_ports = num_tps_per_tm;
		#ifdef USE_RT_CORE
			l1_config.num_ports += 1; //add extra port for RT core
		#endif
			l1_config.num_banks = num_l1_banks;
			l1_config.cross_bar_width = num_l1_banks;
			l1_config.bank_select_mask = 0b0101'0100'0000ull;
			l1_config.num_lfb = 128;
			l1_config.check_retired_lfb = false;
			l1_config.mem_higher = l2s.back();
			l1_config.mem_higher_port_offset = num_l2_ports_per_tm * tm_i;
			l1_config.mem_higher_port_stride = 2;

			l1ds.push_back(new Units::UnitNonBlockingCache(l1_config));
			simulator.register_unit(l1ds.back());
			mem_list.push_back(l1ds.back());
			unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1ds.back();
			unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1ds.back();

			// L1 instruction cache
			for(uint i_cache_index = 0; i_cache_index < num_icache_per_tm; ++i_cache_index)
			{
				Units::UnitBlockingCache::Configuration i_l1_config;
				i_l1_config.size = 8 * 1024;
				i_l1_config.associativity = 4;
				i_l1_config.latency = 1;
				i_l1_config.cycle_time = 1;
				i_l1_config.num_ports = num_tps_per_i_cache;
				i_l1_config.num_banks = 1;
				i_l1_config.cross_bar_width = 1;
				i_l1_config.bank_select_mask = 0;
				i_l1_config.mem_higher = l2s.back();
				i_l1_config.mem_higher_port_offset = num_l2_ports_per_tm * tm_i + i_cache_index * 2 + 1;
				Units::UnitBlockingCache* i_l1 = new Units::UnitBlockingCache(i_l1_config);
				l1is.push_back(i_l1);
				simulator.register_unit(l1is.back());
			}

		#ifdef USE_RT_CORE
			Units::TRaX::UnitRTCore::Configuration rtc_config;
			rtc_config.max_rays = 128;
			rtc_config.num_tp = num_tps_per_tm;
			rtc_config.node_base_addr = (paddr_t)kernel_args.nodes;
			rtc_config.tri_base_addr = (paddr_t)kernel_args.tris;
			rtc_config.cache = l1ds.back();

			rtcs.push_back(_new  Units::TRaX::UnitRTCore(rtc_config));
			simulator.register_unit(rtcs.back());
			mem_list.push_back(rtcs.back());
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM7] = rtcs.back();
		#endif

			thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, kernel_args.framebuffer_width, kernel_args.framebuffer_height, 8, 4));
			simulator.register_unit(thread_schedulers.back());
			mem_list.push_back(thread_schedulers.back());
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

			sfu_list.push_back(_new Units::UnitSFU(64, 2, 1, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

			sfu_list.push_back(_new Units::UnitSFU(8, 1, 1, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

			sfu_list.push_back(_new Units::UnitSFU(4, 6, 1, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

			sfu_list.push_back(_new Units::UnitSFU(2, 3, 1, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

			sfu_list.push_back(_new Units::UnitSFU(1, 22, 8, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM2] = sfu_list.back();

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
				tp_config.tp_index = tp_index;
				tp_config.tm_index = tm_index;
				tp_config.pc = elf.elf_header->e_entry.u64;
				tp_config.sp = 0x0;
				tp_config.stack_size = stack_size;
				tp_config.cheat_memory = mm._data_u8;
				tp_config.inst_cache = l1is[uint(tm_index * num_icache_per_tm + tp_index / num_tps_per_i_cache)];
				tp_config.num_tps_per_i_cache = num_tps_per_i_cache;
				tp_config.unit_table = &unit_tables.back();
				tp_config.unique_mems = &mem_lists.back();
				tp_config.unique_sfus = &sfu_lists.back();
				tp_config.num_threads = num_threads_per_tp;
				//tp_config.num_threads = 1;
				tps.push_back(new Units::TRaX::UnitTP(tp_config));
				simulator.register_unit(tps.back());
				simulator.units_executing++;
			}
		}
	}

	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute();
	auto stop = std::chrono::high_resolution_clock::now();

	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
	double frame_time = simulator.current_cycle / clock_rate;

	Units::TRaX::UnitRTCore::Log rt_core_log;
	for(auto& rt_core : rtcs)
		rt_core_log.accumulate(rt_core->log);

	Units::UnitTP::Log tp_log(elf.segments[0]->vaddr);
	for (auto& tp : tps)
		tp_log.accumulate(tp->log);

	Units::UnitBlockingCache::Log i_l1_log;
	for (auto& i_l1 : l1is)
		i_l1_log.accumulate(i_l1->log);

	Units::UnitNonBlockingCache::Log l1_log;
	for (auto& l1 : l1ds)
		l1_log.accumulate(l1->log);

	Units::UnitBlockingCache::Log l2_log;
	for (auto& l2 : l2s)
		l2_log.accumulate(l2->log);

	//tp_log.print_profile(mm._data_u8);

	mm.print_usimm_stats(CACHE_BLOCK_SIZE, 4, simulator.current_cycle);

	print_header("L2$");
	l2_log.print_log(simulator.current_cycle);

	print_header("L1d$");
	l1_log.print(simulator.current_cycle);

	print_header("L1i$");
	i_l1_log.print_log(simulator.current_cycle);

	print_header("TP");
	tp_log.print();

	print_header("RT Core");
	rt_core_log.print_log(simulator.current_cycle, rtcs.size());

	print_header("Performance Summary");
	//printf("Clock rate: %.0fMHz\n", clock_rate / 1'000'000.0);
	printf("Frame time: %.2fms\n", frame_time * 1000.0);
	printf("Cycles: %lld\n", simulator.current_cycle);
	printf("MRays/s: %.2f\n", tp_log.instruction_counters[(size_t)ISA::RISCV::InstrType::CUSTOM7] / frame_time / (1 << 20));
	printf("MPixels/s: %.2f\n", kernel_args.framebuffer_size / frame_time / (1 << 20));

	print_header("Simulation Summary");
	printf("Simulation rate: %.1fKHz\n", simulator.current_cycle / simulation_time / 1000.0);
	printf("Simulation time: %.1fs\n", simulation_time);



	for(auto& tp : tps) delete tp;
	for(auto& sfu : sfus) delete sfu;
	for(auto& l1 : l1ds) delete l1;
	for(auto& i_l1 : l1is) delete i_l1;
	for(auto& l2 : l2s) delete l2;
	for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
	for(auto& rt_core : rtcs) delete rt_core;

	paddr_t paddr_frame_buffer = reinterpret_cast<paddr_t>(kernel_args.framebuffer);
	stbi_flip_vertically_on_write(true);
	std::string scene_name = scene_names[global_config.scene_id];
	mm.dump_as_png_uint8(paddr_frame_buffer, kernel_args.framebuffer_width, kernel_args.framebuffer_height, scene_name + "_trax_out.png");
}
}
}