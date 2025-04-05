#pragma once
#include "shared-utils.hpp"
#include "units/trax/unit-tp.hpp"
#include "units/trax/unit-rt-core.hpp"
#include "units/trax/unit-prt-core.hpp"
#include "units/trax/unit-treelet-rt-core.hpp"

namespace Arches {

namespace ISA { namespace RISCV { namespace TRaX {

//see the opcode map for details
const static InstructionInfo isa_custom0_000_imm[8] =
{
	InstructionInfo(0x0, "fchthrd", InstrType::CUSTOM0, Encoding::U, RegFile::INT, MEM_REQ_DECL
	{
		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = sizeof(uint32_t);
		req.dst.push(DstReg(instr.rd, RegType::UINT32).u9, 9);
		req.vaddr = 0x0ull;
		return req;
	}),
	InstructionInfo(0x1, "boxisect", InstrType::CUSTOM1, Encoding::U, RegFile::FLOAT, EXEC_DECL
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
	InstructionInfo(0x2, "triisect", InstrType::CUSTOM2, Encoding::U, RegFile::FLOAT, EXEC_DECL
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
	InstructionInfo(0x2, IMPL_NONE),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, IMPL_NONE),
	InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::I, RegFile::FLOAT, MEM_REQ_DECL
	{
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(rtm::Ray);
		mem_req.dst.push(DstReg(instr.rd, RegType::FLOAT32).u9, 9);
		mem_req.vaddr = 0xdeadbeefull;

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(rtm::Ray) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.i.rs1 + i].f32;

		return mem_req;
	}),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3];});

}}}

namespace TRaX {

#include "trax-kernel/include.hpp"
#include "trax-kernel/intersect.hpp"

typedef Units::UnitDRAMRamulator UnitDRAM;
typedef Units::UnitCache UnitL2Cache;
typedef Units::UnitCache UnitL1Cache;
//typedef Units::TRaX::UnitTreeletRTCore UnitRTCore;
//typedef Units::TRaX::UnitRTCore<rtm::NVCWBVH::Node, rtm::Triangle> UnitRTCore;
typedef Units::TRaX::UnitRTCore<rtm::NVCWBVH::Node, rtm::TriangleStrip> UnitRTCore;
//typedef Units::TRaX::UnitRTCore<rtm::HECWBVH::Node, rtm::HECWBVH::Strip> UnitRTCore;

static TRaXKernelArgs initilize_buffers(uint8_t* main_memory, paddr_t& heap_address, const SimulationConfig& sim_config, uint page_size)
{
	std::string scene_name = sim_config.get_string("scene_name");
	std::string project_folder = get_project_folder_path();
	std::string scene_file = project_folder + "datasets\\" + scene_name + ".obj";
	std::string bvh_cache_filename = project_folder + "datasets\\cache\\" + scene_name + ".bvh";

	TRaXKernelArgs args;
	args.framebuffer_width = sim_config.get_int("framebuffer_width");
	args.framebuffer_height = sim_config.get_int("framebuffer_height");
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	heap_address = align_to(page_size, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.pregen_rays = sim_config.get_int("pregen_rays");

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));

	args.camera = sim_config.camera;

	rtm::Mesh mesh(scene_file);
	//float scale = mesh.normalize_verts();
	//printf("Scale: %f\n", scale);
	// mesh.quantize_verts();
	//args.camera._position *= scale;

	std::vector<rtm::BVH2::BuildObject> build_objects;
	mesh.get_build_objects(build_objects);

	rtm::BVH2 bvh2(bvh_cache_filename, build_objects);
	mesh.reorder(build_objects);

	rtm::WBVH wbvh(bvh2, mesh, build_objects);
	//args.nodes = write_vector(main_memory, 256, wbvh.nodes, heap_address);
	mesh.reorder(build_objects);

	args.strips = write_vector(main_memory, 256, wbvh.triangle_strips, heap_address);;

	rtm::NVCWBVH cwbvh(wbvh);
	args.nodes = write_vector(main_memory, 256, cwbvh.nodes, heap_address);

	//rtm::HECWBVH hecwbvh(wbvh, wbvh.triangle_strips);
	//args.nodes = write_vector(main_memory, 256, hecwbvh.nodes, heap_address);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);
	args.tris = write_vector(main_memory, 256, tris, heap_address);

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
		pregen_rays(args.framebuffer_width, args.framebuffer_height, args.camera, cwbvh.nodes.data(), wbvh.triangle_strips.data(), tris.data(), sim_config.get_int("pregen_bounce"), rays);
	args.rays = write_vector(main_memory, 256, rays, heap_address);

	std::memcpy(main_memory + TRAX_KERNEL_ARGS_ADDRESS, &args, sizeof(TRaXKernelArgs));
	return args;
}

static void run_sim_trax(SimulationConfig& sim_config)
{
	std::string project_folder_path = get_project_folder_path();

#if 0 //RTX 4090 ish
	//Compute
	double core_clock = 2235.0e6;
	double dram_clock = 5250.0e6;
	uint64_t stack_size = 512;
	uint num_tms = 128;
	uint num_tps = 128;
	uint num_threads = 8;

	//Memory
	uint64_t block_size = CACHE_BLOCK_SIZE;
	uint num_partitions = 12;
	uint partition_stride = 1 << 12;

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = project_folder_path + "build\\src\\arches-v2\\config-files\\gddr6x_21000_config.yaml";
	dram_config.size = 1ull << 30; //1GB per partition
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency - 92;

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.level = 2;
	l2_config.miss_alloc = true;
	l2_config.size = 6 << 20;
	l2_config.associativity = 16;
	l2_config.num_slices = 6;
	l2_config.crossbar_width = l2_config.num_slices;
	l2_config.num_mshr = 256;
	l2_config.num_subentries = 4;
	l2_config.latency = 176;

	UnitL2Cache::PowerConfig l2_power_config;

	Units::UnitCrossbar::Configuration xbar_config;
	xbar_config.num_slices = l2_config.num_slices;
	xbar_config.slice_stride = l2_config.block_size;
	xbar_config.num_partitions = num_partitions;
	xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = true;
	l1d_config.size = 128 << 10;
	l1d_config.associativity = 32;
	l1d_config.policy = Units::UnitCacheBase::Policy::RANDOM;
	l1d_config.num_banks = 4;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 512;
	l1d_config.num_subentries = 16;
	l1d_config.latency = 30;

	UnitL1Cache::PowerConfig l1d_power_config;

	UnitRTCore::Configuration rtc_config;
	rtc_config.max_rays = 64;
	rtc_config.num_cache_ports = 4;

#elif 1 //Turing spec
#if 1 //RTX 2080
	double core_clock = 1515.0e6;
	uint num_threads = 16;
	uint num_tps = 64;
	uint num_tms = 46;
	uint64_t stack_size = 512;

	double dram_clock = 3500.0e6;
	uint num_partitions = 8;
	uint partition_stride = 1 << 12;
#else //RTX 2060
	double core_clock = 1365.0e6;
	uint num_threads = 1;
	uint num_tps = 64;
	uint num_tms = 30;
	uint64_t stack_size = 512;

	double dram_clock = 3500.0e6;
	uint num_partitions = 6;
	uint partition_stride = 1 << 12;
#endif

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = project_folder_path + "build\\src\\arches-v2\\config-files\\gddr6_14000_config.yaml";
	dram_config.size = 1ull << 30; //1GB
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency = 92;

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.level = 2;
	l2_config.miss_alloc = true;
	l2_config.size = 512 << 10;
	l2_config.associativity = 16;
	l2_config.num_slices = 4;
	l2_config.crossbar_width = l2_config.num_slices;
	l2_config.num_mshr = 192;
	l2_config.num_subentries = 4;
	l2_config.latency = 160;

	UnitL2Cache::PowerConfig l2_power_config;

	Units::UnitCrossbar::Configuration xbar_config;
	xbar_config.num_slices = l2_config.num_slices;
	xbar_config.slice_stride = l2_config.block_size;
	xbar_config.num_partitions = num_partitions;
	xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = true;
	l1d_config.size = 64 << 10;
	l1d_config.associativity = 32;
	l1d_config.policy = Units::UnitCacheBase::Policy::LRU_RANDOM;
	l1d_config.num_banks = 4;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 256;
	l1d_config.num_subentries = 16;
	l1d_config.latency = 20;

	UnitL1Cache::PowerConfig l1d_power_config;

	UnitRTCore::Configuration rtc_config;
	rtc_config.max_rays = 64;
	rtc_config.num_cache_ports = 2;
#else //TRaX 1.0
	double core_clock = 1000.0e6;
	uint num_threads = 1;
	uint num_tps = 32;
	uint num_tms = 32;
	uint num_rays = 32;
	uint64_t stack_size = 4096;

	double dram_clock = 2000.0e6;
	uint num_partitions = 4;
	uint partition_stride = 1 << 12;

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = project_folder_path + "build\\src\\arches-v2\\config-files\\gddr6_pch_config.yaml";
	dram_config.size = 1ull << 30; //1GB
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency = 1;
	//dram_config.latency = 56;

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.level = 2;
	l2_config.miss_alloc = false;
	l2_config.size = 256 << 10;
	l2_config.associativity = 16;
	l2_config.num_slices = 1;
	l2_config.crossbar_width = l2_config.num_slices;
	l2_config.num_mshr = 1;
	l2_config.num_subentries = 1;
	l2_config.latency = 10;

	UnitL2Cache::PowerConfig l2_power_config;

	Units::UnitCrossbar::Configuration xbar_config;
	xbar_config.num_slices = l2_config.num_slices;
	xbar_config.slice_stride = l2_config.block_size;
	xbar_config.num_partitions = num_partitions;
	xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = false;
	l1d_config.size = 32 << 10;
	l1d_config.associativity = 4;
	l1d_config.num_banks = 8;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 256;
	l1d_config.num_subentries = 16;
	l1d_config.latency = 1;

	UnitL1Cache::PowerConfig l1d_power_config;
#endif

	ELF elf(project_folder_path + "src\\trax-kernel\\riscv\\kernel");

	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM7] = "TRACERAY";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::TRaX::custom0;

	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<UnitRTCore*> rtcs;
	std::vector<UnitL1Cache*> l1ds;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

	//construct memory partitions
	std::vector<UnitDRAM*> drams;
	std::vector<UnitL2Cache*> l2s;
	dram_config.num_ports = l2_config.num_slices;
	l2_config.num_ports = l2_config.num_slices;
	for(uint i = 0; i < num_partitions; ++i)
	{
		drams.push_back(_new UnitDRAM(dram_config));
		simulator.register_unit(drams.back());

		l2_config.mem_higher_port = 0;
		l2_config.mem_highers = {drams.back()};
		l2s.push_back(_new UnitL2Cache(l2_config));
		simulator.register_unit(l2s.back());
		simulator.new_unit_group();

		xbar_config.mem_highers.push_back(l2s.back());
	}

	xbar_config.num_clients = num_tms;
	Units::UnitCrossbar xbar(xbar_config);
	simulator.register_unit(&xbar);
	simulator.new_unit_group();

	uint8_t* device_mem = (uint8_t*)malloc(3 << 29);
	paddr_t heap_address = elf.load(device_mem);
	TRaXKernelArgs kernel_args = initilize_buffers(device_mem, heap_address, sim_config, partition_stride);
	heap_address = align_to(partition_stride, heap_address);

	for(uint addr = 0; addr < heap_address; addr += partition_stride)
		drams[xbar.get_partition(addr)]->direct_write(device_mem + addr, partition_stride, xbar.strip_partition_bits(addr));

	bool warm_l2 = false;
	if(warm_l2)
	{
		paddr_t start = (paddr_t)kernel_args.nodes & ~(1 - partition_stride);
		paddr_t end = start + l2_config.size * num_partitions;
		for(paddr_t block_addr = end - l2_config.block_size; block_addr >= start; block_addr -= l2_config.block_size)
			l2s[xbar.get_partition(block_addr)]->direct_write(xbar.strip_partition_bits(block_addr), device_mem + block_addr);
	}

	bool deserialize_l2 = false, serialize_l2 = !deserialize_l2;
	if(deserialize_l2)
		for(uint i = 0; i < num_partitions; ++i)
			serialize_l2 = !l2s[i]->deserialize("l2-p" + std::to_string(i) + ".bin", *drams[i]);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);
	simulator.new_unit_group();

	l1d_config.num_ports = num_tps;
	l1d_config.mem_highers = {&xbar};
#if TRAX_USE_RT_CORE
	l1d_config.num_ports += num_tps;
	l1d_config.crossbar_width *= 2;
#endif

	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);
		std::vector<Units::UnitMemoryBase*> mem_list;
		std::vector<Units::UnitSFU*> sfu_list;

		l1d_config.mem_higher_port = tm_index;
		l1ds.push_back(new UnitL1Cache(l1d_config));
		simulator.register_unit(l1ds.back());
		mem_list.push_back(l1ds.back());
		unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1ds.back();
		unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1ds.back();

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps, tm_index, &atomic_regs, 64));
		simulator.register_unit(thread_schedulers.back());
		mem_list.push_back(thread_schedulers.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

		//sfu_list.push_back(_new Units::UnitSFU(num_tps_per_tm, 2, 1, num_tps_per_tm));
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

	#if TRAX_USE_HARDWARE_INTERSECTORS
		sfu_list.push_back(_new Units::UnitSFU(2, 3, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(1, 22, 8, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM2] = sfu_list.back();
	#endif

		for(auto& sfu : sfu_list)
			sfus.push_back(sfu);

		//l1s_config.mem_higher_port = tm_index * 2 + 1;
		//l1ss.push_back(new Units::UnitStreamCache(l1s_config));
		//simulator.register_unit(l1ss.back());

	#if TRAX_USE_RT_CORE
		rtc_config.num_clients = num_tps;
		rtc_config.node_base_addr = (paddr_t)kernel_args.nodes;
		//rtc_config.tri_base_addr = (paddr_t)kernel_args.tris;
		rtc_config.tri_base_addr = (paddr_t)kernel_args.strips;
		//rtc_config.tri_base_addr = (paddr_t)kernel_args.nodes;
		//rtc_config.treelet_base_addr = (paddr_t)kernel_args.treelets;
		rtc_config.cache = l1ds.back();
		rtc_config.cache_port = num_tps;
		rtc_config.num_cache_ports = 2;
		rtc_config.cache_port_stride = num_tps / l1d_config.num_banks;

		rtcs.push_back(_new  UnitRTCore(rtc_config));
		simulator.register_unit(rtcs.back());
		mem_list.push_back(rtcs.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM7] = rtcs.back();
	#endif

		unit_tables.emplace_back(unit_table);
		sfu_lists.emplace_back(sfu_list);
		mem_lists.emplace_back(mem_list);

		Units::UnitTP::Configuration tp_config;
		tp_config.tm_index = tm_index;
		tp_config.stack_size = stack_size;
		tp_config.cheat_memory = device_mem;
		tp_config.unique_mems = &mem_lists.back();
		tp_config.unique_sfus = &sfu_lists.back();
		tp_config.num_threads = num_threads;
		for(uint tp_index = 0; tp_index < num_tps; ++tp_index)
		{
			tp_config.tp_index = tp_index;
			tp_config.unit_table = &unit_tables.back();
			tps.push_back(new Units::TRaX::UnitTP(tp_config));
			simulator.register_unit(tps.back());
		}

		simulator.new_unit_group();
	}

	printf("Starting TRaX\n");
	for(auto& tp : tps)
		tp->set_entry_point(elf.elf_header->e_entry.u64);

	//master logs
	UnitDRAM::Log dram_log;
	UnitL2Cache::Log l2_log;
	UnitL1Cache::Log l1d_log;
	Units::UnitTP::Log tp_log;

	UnitRTCore::Log rtc_log;

	uint delta = sim_config.get_int("logging_interval");
	float delta_s = delta / core_clock;
	float delta_ns = delta_s * 1e9;
	float delta_dram_cycles = delta_s * dram_clock;
	float peak_dram_bandwidth = dram_clock / core_clock * num_partitions * 4 * 32 / 8;
	float peak_l2_bandwidth = num_partitions * l2_config.num_slices * MemoryRequest::MAX_SIZE;
	float peak_l1d_bandwidth = num_tms * l1d_config.num_banks * MemoryRequest::MAX_SIZE;

	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute(delta, [&]() -> void
	{
		UnitDRAM::Log dram_delta_log = delta_log(dram_log, drams);
		UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2s);
		UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);
		UnitRTCore::Log rtc_delta_log = delta_log(rtc_log, rtcs);

		double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0;

		printf("                            \n");
		printf("Cycle: %lld                 \n", simulator.current_cycle);
		printf("Threads Launched: %d        \n", atomic_regs.iregs[0]);
		printf("Simulation rate: %.2f KHz\n", simulator.current_cycle / simulation_time / 1000.0);
		printf("                            \n");
		printf("DRAM Read: %8.1f GB/s  (%.2f%%)\n", (float)dram_delta_log.bytes_read / delta_ns, 100.0f * dram_delta_log.bytes_read / delta / peak_dram_bandwidth);
		printf(" L2$ Read: %8.1f B/clk (%.2f%%)\n", (float)l2_delta_log.bytes_read / delta, 100.0f * l2_delta_log.bytes_read / delta / peak_l2_bandwidth);
		printf("L1d$ Read: %8.1f B/clk (%.2f%%)\n", (float)l1d_delta_log.bytes_read / delta, 100.0 * l1d_delta_log.bytes_read / delta / peak_l1d_bandwidth);
		printf("                            \n");
		printf(" L2$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l2_delta_log.hits / l2_delta_log.get_total(), 100.0 * l2_delta_log.half_misses / l2_delta_log.get_total(), 100.0 * l2_delta_log.misses / l2_delta_log.get_total());
		printf("L1d$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l1d_delta_log.hits / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.half_misses / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.misses / l1d_delta_log.get_total());
		printf("                            \n");
		printf(" L2$ Stalls: %0.2f%%\n", 100.0 * l2_delta_log.mshr_stalls / num_partitions / l2_config.num_slices / l2_config.num_banks / delta);
		printf("L1d$ Stalls: %0.2f%%\n", 100.0 * l1d_delta_log.mshr_stalls / num_tms  / l1d_config.num_banks / delta);
		printf("                            \n");
		printf("L2$  Occ: %0.2f%%\n", 100.0 * l2_delta_log.get_total() / num_partitions / l2_config.num_slices / l2_config.num_banks / delta);
		printf("L1d$ Occ: %0.2f%%\n", 100.0 * l1d_delta_log.get_total() / num_tms  / l1d_config.num_banks / delta);
		printf("                            \n");
		if(!rtcs.empty())
		{
			printf("MRays/s: %.0f\n\n", rtc_delta_log.rays / delta_ns * 1000.0);
			rtc_delta_log.print(rtcs.size());
		}
	});

	auto stop = std::chrono::high_resolution_clock::now();

	for(uint addr = 0; addr < heap_address; addr += partition_stride)
		drams[xbar.get_partition(addr)]->direct_read(device_mem + addr, partition_stride, xbar.strip_partition_bits(addr));

	if(serialize_l2)
		for(uint i = 0; i < num_partitions; ++i)
			l2s[i]->serialize("l2-p" + std::to_string(i) + ".bin");

	cycles_t frame_cycles = simulator.current_cycle;
	double frame_time = frame_cycles / core_clock;
	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;

	tp_log.print_profile(device_mem);

	float total_power = 0.0f;
	for(auto& dram : drams)
	{
		dram->print_stats(4, frame_cycles);
		total_power += dram->total_power();
	}
	print_header("DRAM");
	delta_log(dram_log, drams);
	printf("DRAM Read: %.1f GB/s (%.2f%%)\n", (float)dram_log.bytes_read / frame_cycles, 100.0f * dram_log.bytes_read / frame_cycles / peak_dram_bandwidth);
	dram_log.print(frame_cycles);

	print_header("L2$");
	delta_log(l2_log, l2s);
	printf(" L2$ Read: %.1f B/clk (%.2f%%)\n", (float)l2_log.bytes_read / frame_cycles, 100.0f * l2_log.bytes_read / frame_cycles / peak_l2_bandwidth);
	l2_log.print(frame_cycles);
	total_power += l2_log.print_power(l2_power_config, frame_time);

	print_header("L1d$");
	delta_log(l1d_log, l1ds);
	printf("L1d$ Read: %.1f B/clk (%.2f%%)\n", (float)l1d_log.bytes_read / frame_cycles, 100.0 * l1d_log.bytes_read / frame_cycles / peak_l1d_bandwidth);
	l1d_log.print(frame_cycles);
	total_power += l1d_log.print_power(l1d_power_config, frame_time);

	print_header("TP");
	delta_log(tp_log, tps);
	tp_log.print(tps.size());

	if(!rtcs.empty())
	{
		print_header("RT Core");
		delta_log(rtc_log, rtcs);
		rtc_log.print(rtcs.size());
	}

	float total_energy = total_power * frame_time;

	print_header("Performance Summary");
	printf("Cycles: %lld\n", simulator.current_cycle);
	printf("Clock rate: %.0f MHz\n", core_clock / 1'000'000.0);
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
	stbi_write_png("out.png", (int)kernel_args.framebuffer_width,  (int)kernel_args.framebuffer_height, 4, device_mem + (size_t)kernel_args.framebuffer, 0);
	free(device_mem);

	for(auto& tp : tps) delete tp;
	for(auto& sfu : sfus) delete sfu;
	for(auto& l1d : l1ds) delete l1d;
	for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
	for(auto& rtc : rtcs) delete rtc;
	for(auto& l2 : l2s) delete l2;
	for(auto& dram : drams) delete dram;
}
}
}