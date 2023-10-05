#pragma once

#include "simulator/simulator.hpp"

#include "units/unit-dram.hpp"
#include "units/unit-non-blocking-cache.hpp"
#include "units/unit-blocking-cache.hpp"
#include "units/unit-atomic-reg-file.hpp"
#include "units/unit-tile-scheduler.hpp"
#include "units/unit-sfu.hpp"
#include "units/unit-tp.hpp"

#include "util/elf.hpp"

#include "../../trax-benchmark/src/include.hpp"

namespace Arches {

namespace ISA { namespace RISCV { namespace TRaX {

//TRAXAMOIN
const static InstructionInfo traxamoin(0b00010, "fchthrd", InstrType::CUSTOM0, Encoding::U, RegType::INT, MEM_REQ_DECL
{
	RegAddr reg_addr;
	reg_addr.reg = instr.i.rd;
	reg_addr.reg_type = RegType::INT;
	reg_addr.sign_ext = false;

	MemoryRequest req;
	req.dst = reg_addr.u8;
	req.size = sizeof(uint32_t);
	req.vaddr = 0x0ull;
	req.type = MemoryRequest::Type::LOAD;

	return req;
});

}}}

static paddr_t align_to(size_t alignment, paddr_t paddr)
{
	return (paddr + alignment - 1) & ~(alignment - 1);
}

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

static KernelArgs initilize_buffers(Units::UnitMainMemoryBase* main_memory, paddr_t& heap_address)
{
	rtm::Mesh mesh("../datasets/sponza.obj");
	rtm::BVH blas;
	std::vector<rtm::Triangle> tris;
	std::vector<rtm::BVH::BuildObject> build_objects;
	for(uint i = 0; i < mesh.size(); ++i)
		build_objects.push_back(mesh.get_build_object(i));
	blas.build(build_objects);
	mesh.reorder(build_objects);
	mesh.get_triangles(tris);

	KernelArgs args;
	args.framebuffer_width = 64;
	args.framebuffer_height = 64;
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;

	heap_address = align_to(ROW_BUFFER_SIZE, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address); heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.samples_per_pixel = 1;
	args.max_depth = 1;

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	args.camera = rtm::Camera(args.framebuffer_width, args.framebuffer_height, 12.0f, rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f));
	//global_data.camera = Camera(global_data.framebuffer_width, global_data.framebuffer_height, 24.0f, rtm::vec3(0.0f, 0.0f, 5.0f));

	heap_address = align_to(CACHE_BLOCK_SIZE, heap_address) + 32;
	args.mesh.blas = write_vector(main_memory, 32, blas.nodes, heap_address);
	args.mesh.tris = write_vector(main_memory, CACHE_BLOCK_SIZE, tris, heap_address);

	main_memory->direct_write(&args, sizeof(KernelArgs), KERNEL_ARGS_ADDRESS);

	return args;
}

static void run_sim_trax(int argc, char* argv[])
{
	uint num_tps_per_tm = 32;
	uint num_tms_per_l2 = 8;
	uint num_l2 = 4;

	uint num_tps = num_l2 * num_tms_per_l2 * num_tps_per_tm;
	uint num_tms = num_tms_per_l2 * num_l2;
	uint sfu_table_size = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES);

	//hardware spec
	uint64_t mem_size = 4ull * 1024ull * 1024ull * 1024ull; //4GB

	//cached global data
	uint64_t stack_size = 2048; //1KB

	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::TRaX::traxamoin;
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";

	Simulator simulator;

	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitNonBlockingCache*> l1s;
	std::vector<Units::UnitBlockingCache*> l2s;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);
	
	Units::UnitDRAM mm(num_l2 * 16, 1024ull * 1024ull * 1024ull, &simulator); mm.clear();
	simulator.register_unit(&mm);
	
	ELF elf("../trax-benchmark/riscv/path-tracer");
	vaddr_t global_pointer;
	paddr_t heap_address = mm.write_elf(elf);
	
	KernelArgs kernel_args = initilize_buffers(&mm, heap_address);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);

	for(uint l2_index = 0; l2_index < num_l2; ++l2_index)
	{
		Units::UnitBlockingCache::Configuration l2_config;
		l2_config.size = 512 * 1024;
		l2_config.associativity = 1;
		l2_config.data_array_latency = 3;
		l2_config.num_ports = num_tms_per_l2 * 8;
		l2_config.num_banks = 16;
		l2_config.bank_select_mask = 0b1110'0000'0100'0000;
		l2_config.mem_higher = &mm;
		l2_config.mem_higher_port_offset = 16 * l2_index;

		l2s.push_back(new Units::UnitBlockingCache(l2_config));

		for(uint tm_i = 0; tm_i < num_tms_per_l2; ++tm_i)
		{
			simulator.new_unit_group();
			if(tm_i == 0) simulator.register_unit(l2s.back());

			uint tm_index = l2_index * num_tms_per_l2 + tm_i;

			Units::UnitNonBlockingCache::Configuration l1_config;
			l1_config.size = 32 * 1024;
			l1_config.associativity = 1;
			l1_config.data_array_latency = 0;
			l1_config.num_ports = num_tps_per_tm;
			l1_config.num_banks = 8;
			l1_config.bank_select_mask = 0b0101'0100'0000;
			l1_config.num_lfb = 8;
			l1_config.mem_higher = l2s.back();
			l1_config.mem_higher_port_offset = 8 * tm_i;

			l1s.push_back(new Units::UnitNonBlockingCache(l1_config));
			simulator.register_unit(l1s.back());

			thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps_per_tm, tm_index, &atomic_regs, kernel_args.framebuffer_width, kernel_args.framebuffer_height));
			simulator.register_unit(thread_schedulers.back());

			std::vector<Units::UnitSFU*> sfu_list;
			std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);
			std::vector<Units::UnitMemoryBase*> mem_list = {l1s.back(), thread_schedulers.back()};

			unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1s.back();
			unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1s.back();
			unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

			sfu_list.push_back(_new Units::UnitSFU(16, 1, 1, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

			sfu_list.push_back(_new Units::UnitSFU(2, 1, 1, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

			sfu_list.push_back(_new Units::UnitSFU(1, 1, 20, num_tps_per_tm));
			simulator.register_unit(sfu_list.back());
			unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
			unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

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
				tp_config.pc = elf.elf_header->e_entry.u64;
				tp_config.sp = 0x0;
				tp_config.gp = 0x0000000000012c34;
				tp_config.stack_size = stack_size;
				tp_config.cheat_memory = mm._data_u8;
				tp_config.unit_table = &unit_tables.back();
				tp_config.unique_mems = &mem_lists.back();
				tp_config.unique_sfus = &sfu_lists.back();

				tps.push_back(new Units::UnitTP(tp_config));
				simulator.register_unit(tps.back());
				simulator.units_executing++;
			}
		}
	}

	{
		auto start = std::chrono::high_resolution_clock::now();
		simulator.execute();
		auto stop = std::chrono::high_resolution_clock::now();

		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout << "Runtime: " << duration.count() << " ms\n";
		std::cout << "Cycles: " << simulator.current_cycle << "\n";
	}

	printf("\nTP\n");
	Units::UnitTP::Log tp_log(elf.segments[0]->vaddr);
	for(auto& tp : tps)
		tp_log.accumulate(tp->log);
	tp_log.print_log();

	printf("\nL1\n");
	Units::UnitNonBlockingCache::Log l1_log;
	for(auto& l1 : l1s)
		l1_log.accumulate(l1->log);
	l1_log.print_log();

	printf("\nL2\n");
	Units::UnitBlockingCache::Log l2_log;
	for(auto& l2 : l2s)
		l2_log.accumulate(l2->log);
	l2_log.print_log();

	mm.print_usimm_stats(CACHE_BLOCK_SIZE, 4, simulator.current_cycle);
	tp_log.print_profile(mm._data_u8);

	for(auto& tp : tps) delete tp;
	for(auto& sfu : sfus) delete sfu;
	for(auto& l1 : l1s) delete l1;
	for(auto& l2 : l2s) delete l2;
	for(auto& ts : thread_schedulers) delete ts;

	paddr_t paddr_frame_buffer = reinterpret_cast<paddr_t>(kernel_args.framebuffer);
	stbi_flip_vertically_on_write(true);
	mm.dump_as_png_uint8(paddr_frame_buffer, kernel_args.framebuffer_width, kernel_args.framebuffer_height, "out.png");
}

}