#pragma once 
#include "stdafx.hpp"

#include "units/unit-base.hpp"
#include "units/unit-main-memory-base.hpp"

#include "strata-rt-kernel/ray-data.hpp"

#define ENABLE_RSB_DEBUG_PRINTS (true)

namespace Arches { namespace Units { namespace STRaTART {

class HitComparitor
{
public:
	bool operator() (MemoryRequest a, MemoryRequest b)
	{
		return a.paddr > b.paddr;
	}
};

class UnitRayStreamBuffer : public UnitMemoryBase
{
public:
	struct Configuration
	{
		uint64_t size{1024};
		uint num_tm{1};
		uint num_banks{1};
		uint latency{1};
		uint rtc_max_rays{256};
		rtm::WideTreeletBVH::Treelet::Header* cheat_treelets;
	};

	UnitRayStreamBuffer(const Configuration& config);

	struct TMState
	{
		std::queue<MemoryRequest> ray_load_queue{};
		uint current_treelet{~0u};
	};

	struct TreeletState
	{
		std::queue<STRaTARTKernel::RayData> rays{};
		uint num_tms{0};
		uint age{0};
		bool first_ray{true};
	};

	struct Bank
	{
		std::priority_queue<MemoryRequest, std::vector<MemoryRequest>, HitComparitor> hit_load_queue;
		Pipline<MemoryRequest> request_pipline;
		Bank(uint latency) : request_pipline(latency) {}
	};

	std::map<uint, TreeletState> _treelet_states;
private:
	rtm::WideTreeletBVH::Treelet::Header* _cheat_treelets;
	RequestCascade _request_network;
	ReturnCascade _return_network;
	std::vector<TMState> _tm_states;
	std::vector<Bank> _banks;

	std::queue<STRaTARTKernel::RayData> _completed_rays;
	uint64_t _max_rays{0};

	uint64_t _buffer_address_mask;
	uint32_t _rtc_max_rays;
	RoundRobinArbiter<uint64_t> arb;

	uint _treelet_arb = 0;

public:
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

public:
	class Log
	{
	public:
		union
		{
			struct
			{
				uint64_t hits;
				uint64_t bytes_read;
				uint64_t bytes_written;
			};
			uint64_t counters[8];
		};

		Log() { reset(); }

		void reset()
		{
			for(uint i = 0; i < 8; ++i)
				counters[i] = 0;
		}

		void accumulate(const Log& other)
		{
			for(uint i = 0; i < 8; ++i)
				counters[i] += other.counters[i];
		}

		void print(cycles_t cycles, uint units = 1)
		{
			printf("Read Bandwidth: %.2f bytes/cycle\n", (double)bytes_read / units / cycles);
			printf("Write Bandwidth: %.2f bytes/cycle\n", (double)bytes_written / units / cycles);
			printf("\n");
		}
	}
	log;
};

}}}