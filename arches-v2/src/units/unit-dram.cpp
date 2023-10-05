#include "unit-dram.hpp"

#include "USIMM/usimm.h"

namespace Arches { namespace Units {

#define DRAM_CHANNELS 8
#define ENABLE_DRAM_DEBUG_PRINTS 0

UnitDRAM::UnitDRAM(uint num_ports, uint64_t size, Simulator* simulator) : UnitMainMemoryBase(size),
	_request_network(num_ports, DRAM_CHANNELS), _return_network(num_ports, DRAM_CHANNELS)
{
	char* usimm_config_file = (char*)REL_PATH_BIN_TO_SAMPLES"gddr5.cfg";
	char* usimm_vi_file = (char*)REL_PATH_BIN_TO_SAMPLES"1Gb_x16_amd2GHz.vi";
	if (usimm_setup(usimm_config_file, usimm_vi_file) < 0) assert(false); //usimm faild to initilize

	assert(numDramChannels() == DRAM_CHANNELS);

	_channels.resize(numDramChannels());
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

void UnitDRAM::UsimmNotifyEvent(paddr_t const address, cycles_t write_cycle, uint32_t request_id)
{
	dram_address_t const dram_addr = calcDramAddr(address);
	_channels[dram_addr.channel].return_queue.emplace(address, request_id, write_cycle);
}

bool UnitDRAM::_load(const MemoryRequest& request)
{
	//iterface with usimm
	dram_address_t const dram_addr = calcDramAddr(request.paddr);

	arches_request_t req;
	req.arches_addr = request.paddr;
	req.listener = this;
	req.id = request.port;

	reqInsertRet_t reqRet = insert_read(dram_addr, req, _current_cycle * DRAM_CLOCK_MULTIPLIER);

	if(reqRet.retType == reqInsertRet_tt::RRT_READ_QUEUE_FULL) return false;

	assert(reqRet.retType == reqInsertRet_tt::RRT_WRITE_QUEUE || reqRet.retType == reqInsertRet_tt::RRT_READ_QUEUE);

	if (reqRet.retLatencyKnown)
	{
		_channels[dram_addr.channel].return_queue.emplace(request.paddr, request.port, reqRet.completionTime / DRAM_CLOCK_MULTIPLIER);
	}

#if ENABLE_DRAM_DEBUG_PRINTS
	printf("Load(%d): 0x%llx(%d, %d, %d, %lld, %d)\n", req.id, request.paddr, dram_addr.channel, dram_addr.rank, dram_addr.bank, dram_addr.row, dram_addr.column);
#endif


	return true;
}

bool UnitDRAM::_store(const MemoryRequest& request)
{
	//interface with usimm
	dram_address_t const dram_addr = calcDramAddr(request.paddr);

	arches_request_t req;
	req.arches_addr = request.paddr;
	req.listener = this;
	req.id = request.port;

	reqInsertRet_t reqRet = insert_write(dram_addr, req, _current_cycle * DRAM_CLOCK_MULTIPLIER);
	if (reqRet.retType == reqInsertRet_tt::RRT_WRITE_QUEUE_FULL) return false;

	assert(!reqRet.retLatencyKnown);
	assert(reqRet.retType == reqInsertRet_tt::RRT_WRITE_QUEUE);

#if ENABLE_DRAM_DEBUG_PRINTS
	printf("Store(%d): 0x%llx(%d, %d, %d, %lld, %d)\n", req.id, request.paddr, dram_addr.channel, dram_addr.rank, dram_addr.bank, dram_addr.row, dram_addr.column);
#endif

	//Masked write
	for(uint i = 0; i < request.size; ++i)
		if((request.write_mask >> i) & 0x1)
			_data_u8[request.paddr + i] = request.data[i];

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
			if(_store(request))
				_request_network.read(channel_index);
		}
		else if(request.type == MemoryRequest::Type::LOAD)
		{
			if(_load(request))
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
		if(_return_network.is_write_valid(channel_index) && !channel.return_queue.empty())
		{
			const ReturnItem& return_item = channel.return_queue.top();
			if(_current_cycle >= return_item.return_cycle)
			{
#if ENABLE_DRAM_DEBUG_PRINTS
				printf("Load Return(%d): 0x%llx\n", return_item.port, return_item.paddr);
#endif

				MemoryReturn ret;
				ret.type = MemoryReturn::Type::LOAD_RETURN;
				ret.size = CACHE_BLOCK_SIZE;
				ret.paddr = return_item.paddr;
				ret.port = return_item.port;
				std::memcpy(ret.data, &_data_u8[return_item.paddr], CACHE_BLOCK_SIZE);
				_return_network.write(ret, channel_index);
				channel.return_queue.pop();
			}
		}
	}

	_return_network.clock();
}



}}