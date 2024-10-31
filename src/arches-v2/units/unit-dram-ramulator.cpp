#include "unit-dram-ramulator.hpp"


namespace Arches { namespace Units {

#define ENABLE_DRAM_DEBUG_PRINTS 0

UnitDRAMRamulator::UnitDRAMRamulator(uint num_ports, uint num_channels, uint64_t size) : UnitMainMemoryBase(size),
	_request_network(num_ports, num_channels), _return_network(num_channels, num_ports)
{
	config_path = "./config-files/gddr6_16ch_config.yaml";
	YAML::Node config = Ramulator::Config::parse_config_file(config_path, {});
	ramulator2_frontend = Ramulator::Factory::create_frontend(config);
	ramulator2_memorysystem = Ramulator::Factory::create_memory_system(config);
	clock_ratio = ramulator2_memorysystem->get_clock_ratio();

	if ((ramulator2_frontend == nullptr) || (ramulator2_memorysystem == nullptr))
		assert(false);

	ramulator2_frontend->connect_memory_system(ramulator2_memorysystem);
	ramulator2_memorysystem->connect_frontend(ramulator2_frontend);

	_channels.resize(num_channels);
}

UnitDRAMRamulator::~UnitDRAMRamulator() /*override*/
{
	
}

bool UnitDRAMRamulator::request_port_write_valid(uint port_index)
{
	return _request_network.is_write_valid(port_index);
}

void UnitDRAMRamulator::write_request(const MemoryRequest& request)
{
	_request_network.write(request, request.port);
}

bool UnitDRAMRamulator::return_port_read_valid(uint port_index)
{
	return _return_network.is_read_valid(port_index);
}

const MemoryReturn& UnitDRAMRamulator::peek_return(uint port_index)
{
	return _return_network.peek(port_index);
}

const MemoryReturn UnitDRAMRamulator::read_return(uint port_index)
{
	return _return_network.read(port_index);
}


void UnitDRAMRamulator::print_stats(
	uint32_t const word_size,
	cycles_t cycle_count)
{
	ramulator2_frontend->finalize();
	ramulator2_memorysystem->finalize();
}

float UnitDRAMRamulator::total_power()
{
	return 0.0f;
}


bool UnitDRAMRamulator::_load(const MemoryRequest& request, uint channel_index)
{
	//const int byteOffsetWidth = log_base2(CACHE_LINE_SIZE);
	//paddr_t req_addr = request.paddr >> byteOffsetWidth;
	uint return_id = ~0;
	if (free_return_ids.empty())
	{
		return_id = returns.size();
		returns.emplace_back();
	}
	else
	{
		return_id = free_return_ids.top();
		free_return_ids.pop();
	}

	bool enqueue_success = ramulator2_frontend->receive_external_requests(0, request.paddr, return_id, [this, channel_index](Ramulator::Request& req)
	{
		_assert(req.addr_vec[0] == channel_index);
		
		// your read request callback 
#if ENABLE_DRAM_DEBUG_PRINTS
		printf("Load: 0x%llx(%d, %d, %d, %d, %d): %d cycles\n", req.addr, req.addr_vec[0], req.addr_vec[1], req.addr_vec[2], req.addr_vec[3], req.addr_vec[4], (req.depart - req.arrive) / clock_ratio);
#endif
		_channels[channel_index].return_queue.push({ req.depart, (uint)req.source_id });
	});

	if (enqueue_success)
	{
		//std::cout << "Load channel_index: " << channel_index << std::endl;
		MemoryReturn& ret = returns[return_id];
		ret = MemoryReturn(request, _data_u8 + request.paddr);
		log.loads++;
	}

	return enqueue_success;
}

bool UnitDRAMRamulator::_store(const MemoryRequest& request, uint channel_index)
{
	//interface with ramulator
	bool enqueue_success = ramulator2_frontend->receive_external_requests(1,  request.paddr & ~0x3full, -1, [this](Ramulator::Request& req)
	{	// your read request callback 
#if ENABLE_DRAM_DEBUG_PRINTS
		printf("Load(%d): 0x%llx(%d, %d, %d, %lld, %d)\n", request.port, request.paddr, req.addr_vec[0], req.addr_vec[1], req.addr_vec[2], req.addr_vec[3], req.addr_vec[4]);
#endif
	});

	//Masked write
	if (enqueue_success)
	{
		std::memcpy(&_data_u8[request.paddr], request.data, request.size);
		log.stores++;
		log.bytes_written += request.size;
	}

	return enqueue_success;
}

void UnitDRAMRamulator::clock_rise()
{
	_request_network.clock();

	for(uint channel_index = 0; channel_index < _channels.size(); ++channel_index)
	{
		if(!_request_network.is_read_valid(channel_index)) continue;

		const MemoryRequest& request = _request_network.peek(channel_index);

		if (request.type == MemoryRequest::Type::STORE)
		{
			if (_store(request, channel_index))
				_request_network.read(channel_index);
		}
		else if (request.type == MemoryRequest::Type::LOAD)
		{
			if (_load(request, channel_index))
				_request_network.read(channel_index);
		}

		if(!_busy)
		{
			_busy = true;
			simulator->units_executing++;
		}
	}
}

void UnitDRAMRamulator::clock_fall()
{
	for (uint i = 0; i < clock_ratio; ++i)
		ramulator2_memorysystem->tick();

	if(_busy && !ramulator2_memorysystem->ramu_is_busy())
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
			const RamulatorReturn& ramulator_return = channel.return_queue.top();
			const MemoryReturn& ret = returns[ramulator_return.return_id];
			if(_return_network.is_write_valid(channel_index))
			{
#if ENABLE_DRAM_DEBUG_PRINTS
				printf("Load Return(%d): 0x%llx\n", ret.port, ret.paddr);
#endif
				assert(_current_cycle >= (ramulator_return.return_cycle / clock_ratio));
				log.bytes_read += ret.size;
				_return_network.write(ret, channel_index);
				free_return_ids.push(ramulator_return.return_id);
				channel.return_queue.pop();
			}
		}
	}

	_return_network.clock();
}



}}