#pragma once
#include "shared-utils.hpp"
#include "units/trax/unit-sgt-core.hpp"
#include "sg-kernel/include.hpp"

namespace Arches {

namespace ISA {
namespace RISCV {
namespace TRaXSG {

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
	InstructionInfo(0x1, IMPL_NONE),
	InstructionInfo(0x2, IMPL_NONE),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, IMPL_NONE),
	InstructionInfo(0x5, IMPL_NONE),
	InstructionInfo(0x6, IMPL_NONE),
	InstructionInfo(0x7, IMPL_NONE),
};

const static InstructionInfo isa_custom0_funct3[8] =
{
	InstructionInfo(0x0, META_DECL{return isa_custom0_000_imm[instr.u.imm_31_12 >> 3]; }),
	InstructionInfo(0x1, IMPL_NONE),
	InstructionInfo(0x2, IMPL_NONE),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, IMPL_NONE),
	InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::ICR, RegFile::INT, sizeof(SGKernel::HitPacket) / sizeof(uint32_t), RegFile::FLOAT, 9, MEM_REQ_DECL
	{
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::TRACERAY;
		mem_req.size = 9 * sizeof(float);
		mem_req.dst.push(DstReg(instr.rd, RegType::UINT32).u9, 9);
		mem_req.vaddr = 0xdeadbeefull;

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < 9; ++i)
			((float*)mem_req.data)[i] = fr[instr.i.rs1 + i].f32;

		return mem_req;
	}),
	InstructionInfo(0x6, IMPL_NONE),
	InstructionInfo(0x7, IMPL_NONE),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3]; });

}
}
}

namespace TRaXSG {

#include "trax-kernel/include.hpp"
#include "trax-kernel/intersect.hpp"

typedef Units::UnitDRAMRamulator UnitDRAM;
typedef Units::UnitCache UnitL2Cache;
typedef Units::UnitCache UnitL1Cache;
typedef Units::TRaXSG::UnitRTCore<rtm::CompressedWideBVH> UnitRTCore;

static SGKernel::Args initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address, const SimulationConfig& sim_config, uint page_size)
{
	std::string scene_name = "point-cloud";
	std::string project_folder = get_project_folder_path();
	std::string scene_file = project_folder + "datasets\\" + scene_name + ".ply";
	std::string bvh_cache_filename = project_folder + "datasets\\cache\\" + scene_name + ".bvh";

	SGKernel::Args args;
	args.framebuffer_width = sim_config.get_int("framebuffer_width");
	args.framebuffer_height = sim_config.get_int("framebuffer_height");
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	heap_address = align_to(page_size, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.pregen_rays = false;

	//args.camera = sim_config.camera;
	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 40.0f, rtm::vec3(-3, 0.0, -3), rtm::vec3(0, 1.0, 0));

	std::vector<rtm::SphericalGaussian> sgs;
	std::vector<rtm::SphericalHarmonic> shs;
	rtm::PLY ply(scene_file);
	ply.read(sgs, shs);

	std::vector<rtm::BVH2::BuildObject> build_objects;
	for(uint i = 0; i < sgs.size(); ++i)
	{
		rtm::BVH2::BuildObject obj = sgs[i].build_object();
		if(obj.aabb.surface_area() > 0.0f)
		{
			build_objects.push_back(obj);
			build_objects.back().index = i;
		}
	}

	rtm::BVH2 bvh2("../../../datasets/cache/point-cloud.bvh", build_objects, 2);
	rtm::WideBVH wbvh(bvh2, build_objects);
	rtm::CompressedWideBVH cwbvh(wbvh);

	{
		std::vector<rtm::SphericalGaussian> temp_sgs(sgs);
		std::vector<rtm::SphericalHarmonic> temp_shs(shs);
		for(uint i = 0; i < build_objects.size(); ++i)
		{
			sgs[i] = temp_sgs[build_objects[i].index];
			shs[i] = temp_shs[build_objects[i].index];
			build_objects[i].index = i;
		}
	}

	args.nodes = write_vector(main_memory, CACHE_BLOCK_SIZE, cwbvh.nodes, heap_address);
	args.sgs = write_vector(main_memory, CACHE_BLOCK_SIZE, sgs, heap_address);
	args.shs = write_vector(main_memory, CACHE_BLOCK_SIZE, shs, heap_address);

	main_memory->direct_write(&args, sizeof(SGKernel::Args), TRAX_KERNEL_ARGS_ADDRESS);
	return args;
}

static void run_sim(SimulationConfig& sim_config)
{
	std::string project_folder_path = get_project_folder_path();

#if 1 //Modern config
	//Compute
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
	dram_config.config_path = project_folder_path + "build\\src\\arches-v2\\config-files\\gddr6_pch_config.yaml";
	dram_config.size = 4ull << 30; //4GB
	dram_config.num_controllers = num_partitions;
	dram_config.partition_mask = partition_mask;

	//L2$
	UnitL2Cache::Configuration l2_config;
	l2_config.in_order = sim_config.get_int("l2_in_order");
	l2_config.level = 2;
	l2_config.block_size = block_size;
	l2_config.size = sim_config.get_int("l2_size");
	l2_config.associativity = sim_config.get_int("l2_associativity");
	l2_config.num_partitions = num_partitions;
	l2_config.partition_select_mask = partition_mask;
	l2_config.num_banks = 2;
	l2_config.bank_select_mask = generate_nbit_mask(log2i(l2_config.num_banks)) << log2i(block_size);
	l2_config.crossbar_width = 64;
	l2_config.num_mshr = 192;
	l2_config.rob_size = 4 * l2_config.num_mshr / l2_config.num_banks;
	l2_config.input_latency = 85;
	l2_config.output_latency = 85;

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
	l1d_config.input_latency = 20;
	l1d_config.output_latency = 10;

	UnitL1Cache::PowerConfig l1d_power_config;
	l1d_power_config.leakage_power = 7.19746e-3f * l1d_config.num_banks * num_tms;
	l1d_power_config.tag_energy = 0.000663943e-9f;
	l1d_power_config.read_energy = 0.0310981e-9f - l1d_power_config.tag_energy;
	l1d_power_config.write_energy = 0.031744e-9f - l1d_power_config.tag_energy;
#else //Legacy config

#endif

	_assert(block_size <= MemoryRequest::MAX_SIZE);
	_assert(block_size == CACHE_BLOCK_SIZE);

	ELF elf(project_folder_path + "src\\sg-kernel\\riscv\\kernel");

	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM7] = "TRACERAY";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::TRaXSG::custom0;

	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<UnitRTCore*> rtcs;
	std::vector<UnitL1Cache*> l1ds;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms * num_rtc);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

	dram_config.num_ports = dram_config.num_controllers;
	UnitDRAM dram(dram_config);
	simulator.register_unit(&dram);
	simulator.new_unit_group();

	dram.clear();
	paddr_t heap_address = dram.write_elf(elf);
	SGKernel::Args kernel_args = initilize_buffers(&dram, heap_address, sim_config, partition_stride);

	l2_config.num_ports = num_tms;
	l2_config.mem_highers = {&dram};
	l2_config.mem_higher_port = 0;
	UnitL2Cache l2(l2_config);
	simulator.register_unit(&l2);
	simulator.new_unit_group();

	std::string l2_cache_path = project_folder_path + "\\datasets\\cache\\" + sim_config.get_string("scene_name") + "-" + std::to_string(sim_config.get_int("pregen_bounce")) + "-l2.cache";
	bool deserialized_cache = false;
	if(sim_config.get_int("warm_l2"))
	{
		deserialized_cache = l2.deserialize(l2_cache_path, dram);
		//l2.copy(TRAX_KERNEL_ARGS_ADDRESS, dram._data_u8 + TRAX_KERNEL_ARGS_ADDRESS, sizeof(kernel_args));
	}

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);
	simulator.new_unit_group();

	l1d_config.num_ports = num_tps;
#ifdef TRAX_USE_RT_CORE
	l1d_config.num_ports += num_rtc * l1d_config.num_ports / l1d_config.crossbar_width; //add extra port for RT core
	l1d_config.crossbar_width += num_rtc;
#endif
	l1d_config.mem_highers = {&l2};
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

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps, tm_index, &atomic_regs, 32));
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

	#ifdef TRAX_USE_RT_CORE
		UnitRTCore::Configuration rtc_config;
		rtc_config.num_clients = num_tps;
		rtc_config.max_rays = 256 / num_rtc;
		rtc_config.node_base_addr = (paddr_t)kernel_args.nodes;
		rtc_config.sg_base_addr = (paddr_t)kernel_args.sgs;
		rtc_config.cache = l1ds.back();
		for(uint i = 0; i < num_rtc; ++i)
		{
			rtc_config.cache_port = num_tps + i * 32;
			rtcs.push_back(_new  UnitRTCore(rtc_config));
			simulator.register_unit(rtcs.back());
			mem_list.push_back(rtcs.back());
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM7] = rtcs.back();

			unit_tables.emplace_back(unit_table);
		}
	#endif

		sfu_lists.emplace_back(sfu_list);
		mem_lists.emplace_back(mem_list);

		Units::UnitTP::Configuration tp_config;
		tp_config.tm_index = tm_index;
		tp_config.stack_size = stack_size;
		tp_config.cheat_memory = dram._data_u8;
		tp_config.unique_mems = &mem_lists.back();
		tp_config.unique_sfus = &sfu_lists.back();
		tp_config.num_threads = num_threads;
		for(uint tp_index = 0; tp_index < num_tps; ++tp_index)
		{
			tp_config.tp_index = tp_index;
			tp_config.unit_table = &unit_tables[num_rtc * tm_index + tp_index * num_rtc / num_tps];
			tps.push_back(new Units::UnitTP(tp_config));
			simulator.register_unit(tps.back());
		}

		simulator.new_unit_group();
	}

	printf("Starting TRaX-SG\n");
	for(auto& tp : tps)
		tp->set_entry_point(elf.elf_header->e_entry.u64);

	//master logs
	UnitDRAM::Log dram_log;
	UnitL2Cache::Log l2_log;
	UnitL1Cache::Log l1d_log;
	Units::UnitBlockingCache::Log l1i_log;
	Units::UnitTP::Log tp_log;

	UnitRTCore::Log rtc_log;

	uint delta = sim_config.get_int("logging_interval");
	auto start = std::chrono::high_resolution_clock::now();
	simulator.execute(delta, [&]() -> void
	{
		float epsilon_ns = delta / (clock_rate / 1'000'000'000);

		UnitDRAM::Log dram_delta_log = delta_log(dram_log, dram);
		UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2);
		UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);
		UnitRTCore::Log rtc_delta_log = delta_log(rtc_log, rtcs);

		printf("                            \n");
		printf("Cycle: %lld                 \n", simulator.current_cycle);
		printf("Threads Launched: %d        \n", atomic_regs.iregs[0]);
		printf("                            \n");
		printf("DRAM Read: %8.1f bytes/cycle\n", (float)dram_delta_log.bytes_read / delta);
		printf(" L2$ Read: %8.1f bytes/cycle\n", (float)l2_delta_log.bytes_read / delta);
		printf("L1d$ Read: %8.1f bytes/cycle\n", (float)l1d_delta_log.bytes_read / delta);
		printf("                            \n");
		printf(" L2$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l2_delta_log.hits / l2_delta_log.get_total(), 100.0 * l2_delta_log.half_misses / l2_delta_log.get_total(), 100.0 * l2_delta_log.misses / l2_delta_log.get_total());
		printf("L1d$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l1d_delta_log.hits / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.half_misses / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.misses / l1d_delta_log.get_total());
		printf("                            \n");
		if(!rtcs.empty())
		{
			printf("MRays/s: %.0f\n\n", rtc_delta_log.rays / epsilon_ns * 1000.0);
			rtc_delta_log.print(rtcs.size());
		}
	});

	auto stop = std::chrono::high_resolution_clock::now();

	if(!deserialized_cache)
		l2.serialize(l2_cache_path);

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

	print_header("TP");
	delta_log(tp_log, tps);
	tp_log.print(tps.size());

	if(!rtcs.empty())
	{
		print_header("RT Core");
		delta_log(rtc_log, rtcs);
		rtc_log.rays = kernel_args.framebuffer_size;
		rtc_log.print(rtcs.size());
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
	for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
	for(auto& rtc : rtcs) delete rtc;
}
}
}