#pragma once
#include "stdafx.hpp"

#include "units/unit-memory-base.hpp"
#include "units/unit-dram.hpp"

#include "dual-streaming-kernel/include.hpp"
#include "simulator/interconnects.hpp"
#include "simulator/transactions.hpp"

namespace Arches { namespace Units { namespace DualStreaming {

class UnitSceneBuffer : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint size = {1024};
		uint latency{1};
		uint num_ports{1};
		uint num_banks{1};
		paddr_t bank_select_mask{0};

		size_t  segment_size{0};
		paddr_t segment_start{0};

		bool dynamic_prefetch{ 0 };
		uint prefetch_block{ 2 };

		UnitMainMemoryBase* main_mem;
		uint                main_mem_port_offset{ 0 };
		uint                main_mem_port_stride{ 1 };
	};

	struct PowerConfig
	{
		//Energy is joules, power in watts, and time in seconds
		float read_energy{0.0f};
		float write_energy{0.0f};
		float leakage_power{0.0f};
	};

	FIFO<std::pair<uint, uint>> prefetch_sideband; //clock rise
	FIFO<uint> retire_sideband; //clock rise
	FIFO<uint> prefetch_complete_sideband; 	//clock fall

private:
	class AddressTranslator
	{
	public:
		size_t segment_size;

	private:
		paddr_t start;
		std::set<uint> free_slots;
		std::unordered_map<uint, uint> segment_map;

	public:
		AddressTranslator(paddr_t start, uint segment_size, uint segments) : start(start), segment_size(segment_size)
		{
			for(uint i = 0; i < segments; ++i)
				free_slots.emplace(i);
		}

		uint map(uint segment_id)
		{
			segment_map[segment_id] = *free_slots.begin();
			free_slots.erase(*free_slots.begin());
			return segment_map[segment_id];
		}

		bool is_mapped(uint segment_id)
		{
			return segment_map.find(segment_id) != segment_map.end();
		}

		uint get_slot(uint segment_id)
		{
			_assert(is_mapped(segment_id));
			return segment_map[segment_id];
		}

		void unmap(uint segment_id)
		{
			free_slots.insert(get_slot(segment_id));
			segment_map.erase(segment_id);
		}

		uint num_free_slots()
		{
			return free_slots.size();
		}

		paddr_t translate(paddr_t addr)
		{
			uint segment_id = get_segment_id(addr);
			uint segment_offset = addr - get_segment_address(segment_id);

			if(!is_mapped(segment_id))
				printf("Segmnet %d not in buffer! %lld:%d", segment_id, addr, segment_offset);

			uint slot_id = get_slot(segment_id);
			return slot_id * segment_size + segment_offset;
		}

		paddr_t get_segment_id(paddr_t addr)
		{
			return (addr - start) / segment_size;
		}

		paddr_t get_segment_address(uint segment_id)
		{
			return start + segment_id * segment_size;
		}
	}
	_address_translator;

	class SceneBufferRequestCrossBar : public CasscadedCrossBar<MemoryRequest>
	{
	public:
		uint64_t           bank_select_mask;
		AddressTranslator& address_translator;

	public:
		SceneBufferRequestCrossBar(uint ports, uint banks, uint64_t bank_select_mask, AddressTranslator& address_translator) : 
			CasscadedCrossBar<MemoryRequest>(ports, banks, banks), address_translator(address_translator), bank_select_mask(bank_select_mask)
		{
		}

		uint get_sink(const MemoryRequest& request) override
		{
			paddr_t buffer_addr = address_translator.translate(request.paddr);
			uint bank = pext(buffer_addr, bank_select_mask);
			_assert(bank < num_sinks());
			return bank;
		}
	};

	struct Bank
	{
		Pipline<MemoryReturn> data_array_pipline;
		Bank(uint latency = 1) : data_array_pipline(latency) {}
	};

	struct Channel
	{
		std::queue<std::pair<paddr_t, uint>> dynamic_prefetch_queue;
		std::queue<paddr_t> prefetch_queue;
		uint byte_requested = 0;
	};

	struct SegmentState
	{
		uint bytes_returned = 0;
	};

	std::vector<int> _block_status; // 0: not in the buffer, 1: fetching, 2: in the buffer, it's pretty cheap to maintain this, only need SCENE_BUFFER_SIZE / 64 / 4 bytes
	bool _dynamic_prefetch{ false };
	uint _prefetch_block = 2; // blocks

	std::vector<uint8_t> _data_u8;
	std::vector<Bank> _banks;
	std::vector<Channel> _channels;
	std::map<uint, SegmentState> _segment_states;
	std::queue<uint> _prefetch_complete_queue;
	
	SceneBufferRequestCrossBar _request_network;
	ReturnCrossBar _return_network;

	UnitMemoryBase* _main_memory;
	uint            _main_mem_port_offset{0};
	uint            _main_mem_port_stride{1};

public:
	UnitSceneBuffer(Configuration config) :
		_address_translator(config.segment_start, config.segment_size, config.size / config.segment_size),
		_request_network(config.num_ports, config.num_banks, config.bank_select_mask, _address_translator),
		_return_network(config.num_ports, config.num_banks, config.num_banks), 
		prefetch_sideband(16), retire_sideband(16), prefetch_complete_sideband(16),
		_main_memory(config.main_mem), _main_mem_port_stride(config.main_mem_port_stride), _main_mem_port_offset(config.main_mem_port_offset), _dynamic_prefetch(config.dynamic_prefetch), _prefetch_block(config.prefetch_block)
	{
		_banks.resize(config.num_banks, config.latency);
		_channels.resize(NUM_DRAM_CHANNELS);
		_data_u8.resize(config.size);
		_block_status.resize(config.size / CACHE_BLOCK_SIZE, 0);
	}

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override
	{
		return _request_network.is_write_valid(port_index);
	}

	void write_request(const MemoryRequest& request) override
	{
		_request_network.write(request, request.port);
	}

	bool return_port_read_valid(uint port_index) override
	{
		return _return_network.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index) override
	{
		return _return_network.peek(port_index);
	}

	const MemoryReturn read_return(uint port_index) override
	{
		return _return_network.read(port_index);
	}

private:
	void process_finish();
	void process_prefetch();
	void process_requests(uint bank_index);
	void process_returns(uint channel_index);
	void issue_requests(uint channel_index);
	void issue_returns(uint bank_index);

public:
	class alignas(64) Log
	{
	public:
		const static uint NUM_COUNTERS = 5;
		union
		{
			struct
			{
				uint64_t loads;
				uint64_t stores;
				uint64_t bytes_read;
				uint64_t bytes_written;
				uint64_t read_misses;
			};
			uint64_t counters[NUM_COUNTERS];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];
		}

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = loads + stores;

			printf("Read Bandwidth: %.1f bytes/cycle\n", (double)bytes_read / units / cycles);
			printf("Write Bandwidth: %.1f bytes/cycle\n", (double)bytes_written / units / cycles);

			printf("\n");
			printf("Total: %lld\n", total / units);
			printf("Loads: %lld\n", loads / units);
			printf("Stores: %lld\n", stores / units);
			printf("Hit Rate: %.2f%%\n", 100.0 * (loads - read_misses) / loads);
		}

		bool print_power(PowerConfig power_config, float time_delta, uint units = 1)
		{
			float read_energy = loads * power_config.read_energy / units;
			float write_energy = stores * power_config.write_energy / units;
			float leakage_energy = time_delta * power_config.leakage_power / units;

			float total_energy = read_energy + write_energy + leakage_energy;
			float total_power = total_energy / time_delta;

			printf("\n");
			printf("Total Energy: %.2f mJ\n", total_energy * 1000.0f);
			printf("Total Power: %.2f W\n", total_power);

			return total_power;
		}
	}
	log;
};

}}}