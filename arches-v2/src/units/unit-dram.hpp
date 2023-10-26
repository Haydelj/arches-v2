#pragma once 
#include "../stdafx.hpp"

#include "USIMM/memory_controller.h"

#include "unit-base.hpp"
#include "unit-main-memory-base.hpp"
#include "../util/arbitration.hpp"

namespace Arches { namespace Units {

#define NUM_DRAM_CHANNELS (16)

class UnitDRAM : public UnitMainMemoryBase, public UsimmListener
{
private:
	struct USIMMReturn
	{
		cycles_t return_cycle;
		arches_request_t req;

		friend bool operator<(const USIMMReturn& l, const USIMMReturn& r)
		{
			return l.return_cycle > r.return_cycle;
		}
	};

	struct Channel
	{
		std::priority_queue<USIMMReturn> return_queue;
	};

	bool _busy{false};

	std::vector<Channel> _channels;
	Casscade<MemoryRequest> _request_network;
	FIFOArray<MemoryReturn> _return_network;
	cycles_t _current_cycle{ 0 };

	std::vector<MemoryReturn> returns;
	std::stack<uint> free_return_ids;

public:
	UnitDRAM(uint num_clients, uint64_t size, Simulator* simulator);
	virtual ~UnitDRAM() override;

	bool request_port_write_valid(uint port_index) override;
	void write_request(const MemoryRequest& request, uint port_index) override;

	bool return_port_read_valid(uint port_index) override;
	const MemoryReturn& peek_return(uint port_index) override;
	const MemoryReturn read_return(uint port_index) override;

	void clock_rise() override;
	void clock_fall() override;

	bool usimm_busy();
	void print_usimm_stats(uint32_t const L2_line_size, uint32_t const word_size, cycles_t cycle_count);
	float total_power_in_watts();

	virtual void UsimmNotifyEvent(cycles_t write_cycle, const arches_request_t& req);

private:
	bool _load(const MemoryRequest& request_item, uint channel_index);
	bool _store(const MemoryRequest& request_item, uint channel_index);
};

}}