#include "unit-dram.hpp"

#include "USIMM/usimm.h"

namespace Arches { namespace Units {

#define ENABLE_DRAM_DEBUG_PRINTS 0

UnitDRAM::UnitDRAM(uint num_ports, uint64_t size, Simulator* simulator) : UnitMainMemoryBase(size),
	_request_network(num_ports, NUM_DRAM_CHANNELS), _return_network(num_ports)
{
	char* usimm_config_file = (char*)REL_PATH_BIN_TO_SAMPLES"gddr5_16ch.cfg";
	char* usimm_vi_file = (char*)REL_PATH_BIN_TO_SAMPLES"1Gb_x16_amd2GHz.vi";
	if (usimm_setup(usimm_config_file, usimm_vi_file) < 0) assert(false); //usimm faild to initilize

	assert(numDramChannels() == NUM_DRAM_CHANNELS);

	_channels.resize(numDramChannels());

	registerUsimmListener(this);
}

UnitDRAM::~UnitDRAM() /*override*/
{
	usimmDestroy();
}

bool UnitDRAM::request_port_write_valid(uint port_index)
{
	return _request_network.is_write_valid(port_index);
}

void UnitDRAM::write_request(const MemoryRequest& request, uint port_index)
{
	_request_network.write(request, port_index);
}

bool UnitDRAM::return_port_read_valid(uint port_index)
{
	return _return_network.is_read_valid(port_index);
}

const MemoryReturn& UnitDRAM::peek_return(uint port_index)
{
	return _return_network.peek(port_index);
}

const MemoryReturn UnitDRAM::read_return(uint port_index)
{
	return _return_network.read(port_index);
}

bool UnitDRAM::usimm_busy() {
	return usimmIsBusy();
}

void UnitDRAM::print_usimm_stats(uint32_t const L2_line_size,
	uint32_t const word_size,
	cycles_t cycle_count)
{
	printUsimmStats(L2_line_size, word_size, cycle_count);
}

float UnitDRAM::total_power_in_watts()
{
	return getUsimmPower() / 1000.0f;
}

void UnitDRAM::UsimmNotifyEvent(cycles_t write_cycle, const arches_request_t& req)
{
	_channels[req.channel].return_queue.push({write_cycle, req});
}


bool UnitDRAM::_load(const MemoryRequest& request, uint channel_index)
{
	//iterface with usimm
	dram_address_t const dram_addr = calcDramAddr(request.paddr);
	assert(dram_addr.channel == channel_index);


#if ENABLE_DRAM_DEBUG_PRINTS
	printf("Load(%d): 0x%llx(%d, %d, %d, %lld, %d)\n", request.port, request.paddr, dram_addr.channel, dram_addr.rank, dram_addr.bank, dram_addr.row, dram_addr.column);
#endif

	arches_request_t arches_request;
	arches_request.channel = dram_addr.channel;
	if(free_return_ids.empty())
	{
		arches_request.return_id = returns.size();
		returns.emplace_back();
	}
	else
	{
		arches_request.return_id = free_return_ids.top();
		free_return_ids.pop();
	}

	reqInsertRet_t reqRet = insert_read(dram_addr, arches_request, _current_cycle * DRAM_CLOCK_MULTIPLIER);
	if(reqRet.retType == reqInsertRet_tt::RRT_READ_QUEUE_FULL)
	{
		return false;
	}

	MemoryReturn& ret = returns[arches_request.return_id];
	ret = MemoryReturn(request, _data_u8 + request.paddr);

	assert(reqRet.retType == reqInsertRet_tt::RRT_WRITE_QUEUE || reqRet.retType == reqInsertRet_tt::RRT_READ_QUEUE);

	if (reqRet.retLatencyKnown)
	{
		_channels[dram_addr.channel].return_queue.push({(Arches::cycles_t)reqRet.completionTime / DRAM_CLOCK_MULTIPLIER, arches_request});
	}

	return true;
}

bool UnitDRAM::_store(const MemoryRequest& request, uint channel_index)
{
	//interface with usimm
	dram_address_t const dram_addr = calcDramAddr(request.paddr);
	assert(dram_addr.channel == channel_index);

#if ENABLE_DRAM_DEBUG_PRINTS
	printf("Store(%d): 0x%llx(%d, %d, %d, %lld, %d)\n", request.port, request.paddr, dram_addr.channel, dram_addr.rank, dram_addr.bank, dram_addr.row, dram_addr.column);
#endif

	arches_request_t arches_request;
	arches_request.channel = dram_addr.channel;
	arches_request.return_id = ~0;

	reqInsertRet_t reqRet = insert_write(dram_addr, arches_request, _current_cycle * DRAM_CLOCK_MULTIPLIER);
	if(reqRet.retType == reqInsertRet_tt::RRT_WRITE_QUEUE_FULL)
	{
		return false;
	}

	//Masked write
	for(uint i = 0; i < request.size; ++i)
		if((request.write_mask >> i) & 0x1)
			_data_u8[request.paddr + i] = request.data[i];

	assert(!reqRet.retLatencyKnown);
	assert(reqRet.retType == reqInsertRet_tt::RRT_WRITE_QUEUE);

	return true;
}

void UnitDRAM::clock_rise()
{
	_request_network.clock();

	for(uint channel_index = 0; channel_index < _channels.size(); ++channel_index)
	{
		if(!_request_network.is_read_valid(channel_index)) continue;

		const MemoryRequest& request = _request_network.peek(channel_index);

		if(request.type == MemoryRequest::Type::STORE)
		{
			if(_store(request, channel_index))
				_request_network.read(channel_index);
		}
		else if(request.type == MemoryRequest::Type::LOAD)
		{
			if(_load(request, channel_index))
				_request_network.read(channel_index);
		}

		if(!_busy)
		{
			_busy = true;
			simulator->units_executing++;
		}
	}
}

void UnitDRAM::clock_fall()
{
	for(uint i = 0; i < DRAM_CLOCK_MULTIPLIER; ++i)
		usimmClock();

	if(_busy && !usimmIsBusy())
	{
		_busy = false;
		simulator->units_executing--;
	}

	++_current_cycle;
	for(uint channel_index = 0; channel_index < _channels.size(); ++channel_index)
	{
		Channel& channel = _channels[channel_index];
		if(!channel.return_queue.empty())
		{
			const USIMMReturn& usimm_return = channel.return_queue.top();
			const MemoryReturn& ret = returns[usimm_return.req.return_id];
			if(_current_cycle >= usimm_return.return_cycle && _return_network.is_write_valid(ret.port))
			{
#if ENABLE_DRAM_DEBUG_PRINTS
				printf("Load Return(%d): 0x%llx\n", ret.port, ret.paddr);
#endif
				_return_network.write(ret, ret.port);
				free_return_ids.push(usimm_return.req.return_id);
				channel.return_queue.pop();
			}
		}
	}

	_return_network.clock();
}



}}