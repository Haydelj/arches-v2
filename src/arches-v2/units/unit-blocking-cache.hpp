#pragma once 
#include "stdafx.hpp"

#include "unit-cache-base.hpp"

namespace Arches {
namespace Units {

class UnitBlockingCache : public UnitCacheBase
{
public:
	struct Configuration
	{
		uint size{1024};
		uint block_size{CACHE_BLOCK_SIZE};
		uint associativity{1};

		uint latency{1};
		uint cycle_time{1};

		uint num_ports{1};
		uint num_banks{1};
		uint64_t bank_select_mask{0};

		UnitMemoryBase* mem_higher{nullptr};
		uint            mem_higher_port_offset{0};
		uint            mem_higher_port_stride{1};

		std::shared_ptr<std::vector<MemoryRange>> memory_ranges;
		const char* unit_name;
	};

	struct PowerConfig
	{
		//Energy is joules, power in watts, and time in seconds
		float tag_energy{0.0f};
		float read_energy{0.0f};
		float write_energy{0.0f};
		float leakage_power{0.0f};
	};

	UnitBlockingCache(Configuration config);
	virtual ~UnitBlockingCache();

	void clock_rise() override;
	void clock_fall() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

private:
	struct Bank
	{
		enum class State : uint8_t
		{
			IDLE,
			MISSED,
			ISSUED,
			FILLED,
		}
		state{State::IDLE};
		MemoryRequest current_request{};
		Pipline<MemoryRequest> tag_array_pipline;
		Pipline<MemoryReturn> data_array_pipline;
		Bank(uint latency, uint cycle_time) : tag_array_pipline(cycle_time, cycle_time), data_array_pipline(latency, cycle_time) {}
	};

	std::vector<Bank> _banks;
	RequestCrossBar _request_cross_bar;
	ReturnCrossBar _return_cross_bar;

	UnitMemoryBase* _mem_higher;
	uint _mem_higher_port_offset;
	uint _mem_higher_port_stride;

	void _clock_rise(uint bank_index);
	void _clock_fall(uint bank_index);

public:
	class Log
	{
	public:


		const static uint NUM_COUNTERS = 8;
		union
		{
			struct
			{
				uint64_t requests;
				uint64_t hits;
				uint64_t misses;
				uint64_t uncached_writes;
				uint64_t bytes_read;
				uint64_t tag_array_access;
				uint64_t data_array_reads;
				uint64_t data_array_writes;
			};
			uint64_t counters[NUM_COUNTERS];
		};

		std::map<std::tuple<const char*, const char*, MemoryRequest::Type>, uint64_t> request_logs;

		std::shared_ptr<std::vector<MemoryRange>> memory_ranges;

	public:
		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] = 0;
			
			request_logs.clear();
		}

		const char* GetDataType(paddr_t paddr)
		{
			assert(!memory_ranges->empty());
			int idx = std::upper_bound(memory_ranges->begin(), memory_ranges->end(), MemoryRange({ paddr, nullptr })) - memory_ranges->begin();
			if (idx == memory_ranges->size())
				return "OutOfBound / Unlabeled Data";
			return memory_ranges->at(idx).data_type;
		}

		void log_request(const MemoryRequest& request)
		{
			const char* data_type = GetDataType(request.paddr);
			auto key = std::make_tuple(request.unit_name, data_type, request.type);
			request_logs[key] += request.size;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < NUM_COUNTERS; ++i)
				counters[i] += other.counters[i];

			for (auto& a : other.request_logs)
			{
				request_logs[a.first] += a.second;
			}
		}

		uint64_t get_total() { return hits + misses; }
		uint64_t get_total_data_array_accesses() { return data_array_reads + data_array_writes; }

		void print_request_logs(cycles_t cycles, uint units = 1)
		{
			printf("\nBandwidth Utilization Starts:\n");
			for (const auto& [key, value] : request_logs)
			{
				auto unit_name = std::get<0>(key);
				auto data_type = std::get<1>(key);
				auto request_type = std::get<2>(key);
				uint64_t bytes = value;
				std::string request_label = std::string(data_type);
				if (request_type == MemoryRequest::Type::LOAD)
				{
					request_label = "Load " + request_label;
				}
				else if (request_type == MemoryRequest::Type::PREFETCH)
				{
					request_label = "Prefetch " + request_label;
				}
				else if (request_type == MemoryRequest::Type::STORE)
				{
					request_label = "Store " + request_label;
				}
				printf("Unit name: %s, Request label: %s, Bandwidth Utilization: %.1f bytes/cycle\n", unit_name, request_label.c_str(), (double)bytes / units / cycles);
			}
			printf("Bandwidth Utilization Ends.\n\n");
		}

		void print(cycles_t cycles, uint units = 1)
		{
			uint64_t total = get_total();

			print_request_logs(cycles);

			printf("Read Bandwidth: %.1f bytes/cycle\n", (double)bytes_read / units / cycles);

			printf("\n");
			printf("Total: %lld\n", total / units);
			printf("Hits: %lld(%.2f%%)\n", hits / units, 100.0f * hits / total);
			printf("Misses: %lld(%.2f%%)\n", misses / units, 100.0f * misses / total);

			printf("\n");
			printf("Tag Array Access: %lld\n", tag_array_access);
			printf("Data Array Reads: %lld\n", data_array_reads);
			printf("Data Array Writes: %lld\n", data_array_writes);
		}

		float print_power(PowerConfig power_config, float time_delta, uint units = 1)
		{
			float read_energy = data_array_reads * power_config.read_energy / units;
			float write_energy = data_array_writes * power_config.write_energy / units;
			float tag_energy = tag_array_access * power_config.tag_energy / units;
			float leakage_energy = time_delta * power_config.leakage_power / units;

			float total_energy = read_energy + write_energy + tag_energy + leakage_energy;
			float total_power = total_energy / time_delta;

			printf("\n");
			printf("Total Energy: %.2f mJ\n", total_energy * 1000.0f);
			printf("Total Power: %.2f W\n", total_power);

			return total_power;
		}
	}
	log;
};

}
}