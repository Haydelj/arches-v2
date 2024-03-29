#pragma once

#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-dram.hpp"

#include "dual-streaming-kernel/include.hpp"
#include "simulator/interconnects.hpp"
#include "simulator/transactions.hpp"

namespace Arches {
namespace Units {
namespace DualStreaming {

class UnitSceneBuffer : public UnitBase
{
public:
	struct Configuration
	{
		uint size = 1024 * 1024 * 4; // 4MB
		uint num_ports = 64 * 8;
		uint num_banks = 32; // should be SIZE_OF_BUFFER / SIZE_OF_TREELET
		uint treelet_size = TREELET_SIZE;
		paddr_t bank_select_mask = 0b0;
		
		uint row_size = CACHE_BLOCK_SIZE; // 64B per row in the bank

		paddr_t treelet_start = 0;
		paddr_t treelet_end = 0;
		UnitMainMemoryBase* main_mem;
		uint                main_mem_port_offset{ 0 };
		uint                main_mem_port_stride{ 1 };

		bool allow_wait = true;
	};

	class SceneBufferLoadRequestCrossbar : public CasscadedCrossBar<SceneBufferLoadRequest>
	{
	public:
		SceneBufferLoadRequestCrossbar(uint ports, uint banks) : CasscadedCrossBar<SceneBufferLoadRequest>(ports, banks, banks) {}
		uint get_sink(const SceneBufferLoadRequest& request) override
		{
			return request.sink;
		}
	};
	class InternalCrossbar: public CasscadedCrossBar<MemoryReturn>
	{
	public:
		InternalCrossbar(uint ports, uint banks, uint cross_bar_width) : CasscadedCrossBar<MemoryReturn>(banks, ports, cross_bar_width) {}

		uint get_sink(const MemoryReturn& ret) override
		{
			return ret.port;
		}
	};

	// When load data, we need to get physical addresses
	// We also need to use the physical address to get the segment number and fill the data to the buffer
	// Returning data is similar, we just need to find the segment, and return the data
	// calculate segment index based on a given address
	uint calculate_segment(paddr_t address)
	{
		assert(address >= treelet_start && address <= treelet_end);
		return (address - treelet_start) / treelet_size;
	}
	uint get_bank(paddr_t address)
	{
		uint segment = calculate_segment(address);
		if (finished.count(segment) || !segment_offset.count(segment))
		{
			//std::cout << segment << '\n';
			//std::cout << address << ' ' << offset_in_treelet(address) << '\n';
			//exit(0);
			// something wrong here
			// shouldn't be here but bug sometimes happens
			return ~0u;
		}
		assert(!finished.count(segment));
		assert(segment_offset.count(segment));
		paddr_t offset_in_buffer = 1ull * segment_offset[segment] * treelet_size + (address - treelet_start) % treelet_size;

		uint bank = (offset_in_buffer / row_size) % num_bank;
		//uint bank = pext(offset_in_buffer, bank_select_mask);
		return bank;

		//return (offset_in_buffer / row_size) % num_bank;
	}
	uint offset_in_treelet(paddr_t address)
	{
		return (address - treelet_start) % treelet_size;
	}
	paddr_t calculate_treelet_address(uint segment_index)
	{
		return treelet_start + 1ull * segment_index * treelet_size;
	}
	
	// The buffer is divided into banks
	// Each bank is 128 KB, which may contain multiple treelets
	// When we receive prefetch request, with one request may be across multiple banks, we push prefetch requests to those banks
	// When prefetching data from DRAM, we need to record the total bytes requested to DRAM and loaded from DRAM
	// When a full treelet has been loaded, we send a signal to Stream Scheduler (which simply modify the stream scheduler's segment state)
	// When L1 requests data, we calculate its bank (done in L1), and find its segment and get data
	
	// requests: L1 -> channels
	// returns: channels -> banks -> L1
	struct Bank
	{
		//std::queue<std::pair<uint, uint>> prefetch_queue; // segments that need to prefetch for this bank
		//Pipline<MemoryRequest> data_pipline;
		//uint byte_requested = 0;
		bool has_return;
		MemoryReturn ret;
		uint segment_prefetched = ~0u;
	};

	struct Channel
	{
		std::queue<paddr_t> prefetch_queue; // Segment size is a multiple of DRAM row size, so we only need a start address in DRAM
		uint byte_requested = 0;
	};
	

	// A segment may be across 2 banks
	struct SegmentState
	{
		enum class State
		{
			EMPTY = 0,
			PREFETCHING,
			LOADED
		};
		State state{ State::EMPTY };
		uint byte_returned = 0;
		uint byte_requested = 0;
		uint8_t* data_u8 = nullptr;
		uint segment_index = ~0u;
		std::set<paddr_t> returned_blocks;
	};
	uint treelet_size;
	paddr_t treelet_start;
	paddr_t treelet_end;
	paddr_t bank_select_mask;
	uint num_segments;
	uint row_size;
	uint num_bank;
	uint dram_row_size = 8 * 1024; // 8KB in DRAM
	bool allow_wait = false;
	std::vector<Bank> banks;
	std::vector<Channel> channels;

	std::map<uint, uint> segment_offset;
	std::vector<SegmentState> segments;

	std::queue<uint> prefetch_request;
	std::queue<uint> segment_prefetched;
	std::queue<uint> segment_finished;

	std::map<uint, bool> segment_signal_returned;
	std::map<uint, bool> finished;

	// Request comes from TM, this request_network connects banks and TM request
	SceneBufferLoadRequestCrossbar request_network;
	// When data returns from channels, we use a crossbar to send the data to its corresponding bank
	InternalCrossbar internal_return_crossbar;
	
	// Return sent to Stream Scheduler and TM
	FIFOArray<MemoryReturn> return_network;

	UnitMemoryBase* main_memory;
	uint                main_mem_port_offset{ 0 };
	uint                main_mem_port_stride{ 1 };

	UnitSceneBuffer(Configuration config) : treelet_start(config.treelet_start), bank_select_mask(config.bank_select_mask), treelet_size(config.treelet_size), request_network(config.num_ports, config.num_banks), return_network(config.num_ports), main_memory(config.main_mem), main_mem_port_stride(config.main_mem_port_stride), main_mem_port_offset(config.main_mem_port_offset), row_size(config.row_size), num_bank(config.num_banks), treelet_end(config.treelet_end), internal_return_crossbar(config.num_banks, NUM_DRAM_CHANNELS, NUM_DRAM_CHANNELS), allow_wait(config.allow_wait)
	{
		channels.resize(NUM_DRAM_CHANNELS);
		banks.resize(config.num_banks);
		num_segments = config.size / treelet_size;
		segments.resize(num_segments);
		for (auto& segment : segments)
		{
			segment.data_u8 = (uint8_t*)malloc(treelet_size);
			segment.segment_index = ~0u;
		}
	}
	~UnitSceneBuffer()
	{
		for (auto& segment : segments)
		{
			free(segment.data_u8);
		}
	}

	uint find_segment_slot();

	void process_finish();

	void process_prefetch();

	void process_requests(uint bank_index);

	void process_internal_return(uint bank_index);

	void process_returns(uint channel_index);

	void issue_requests(uint channel_index);

	void issue_returns(uint bank_index);

	void clock_rise() override;

	void clock_fall() override;

	bool request_port_write_valid(uint port_index)
	{
		return request_network.is_write_valid(port_index);
	}

	void write_request(const SceneBufferLoadRequest& request, uint port_index)
	{
		request_network.write(request, port_index);
	}

	bool return_port_read_valid(uint port_index)
	{
		return return_network.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index)
	{
		return return_network.peek(port_index);
	}

	const MemoryReturn& read_return(uint port_index)
	{
		return return_network.read(port_index);
	}
};

}

}
}