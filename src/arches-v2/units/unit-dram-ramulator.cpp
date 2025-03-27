	#include "unit-dram-ramulator.hpp"

namespace Arches { namespace Units {

#define ENABLE_DRAM_DEBUG_PRINTS 0

UnitDRAMRamulator::UnitDRAMRamulator(Configuration config) : UnitMainMemoryBase(config.size),
	_request_network(config.num_ports, config.num_controllers), _return_network(config.num_controllers, config.num_ports), _partition_mask(config.partition_stride)
{
	YAML::Node yaml = Ramulator::Config::parse_config_file(config.config_path, {});

	_controllers.resize(config.num_controllers, 100);
	for(uint i = 0; i < config.num_controllers; ++i)
	{
		_controllers[i].ramulator2_frontend = Ramulator::Factory::create_frontend(yaml);
		_controllers[i].ramulator2_memorysystem = Ramulator::Factory::create_memory_system(yaml);

		_assert(_controllers[i].ramulator2_frontend);
		_assert(_controllers[i].ramulator2_memorysystem);

		_controllers[i].ramulator2_frontend->connect_memory_system(_controllers[i].ramulator2_memorysystem);
		_controllers[i].ramulator2_memorysystem->connect_frontend(_controllers[i].ramulator2_frontend);
	}

	_clock_ratio = config.clock_ratio;
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
	for(auto& controller : _controllers)
	{
		controller.ramulator2_frontend->finalize();
		controller.ramulator2_memorysystem->finalize();
	}
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
	if (_free_return_ids.empty())
	{
		return_id = _returns.size();
		_returns.emplace_back();
	}
	else
	{
		return_id = _free_return_ids.top();
		_free_return_ids.pop();
	}

	bool enqueue_success = _controllers[channel_index].ramulator2_frontend->receive_external_requests(0, _convert_address(request.paddr), return_id, [this, channel_index](Ramulator::Request& req)
	{
		// your read request callback 
#if ENABLE_DRAM_DEBUG_PRINTS
		printf("Load: 0x%llx(%d, %d, %d, %d, %d): %d cycles\n", req.addr, req.addr_vec[0], req.addr_vec[1], req.addr_vec[2], req.addr_vec[3], req.addr_vec[4], (req.depart - req.arrive) / _clock_ratio);
#endif
		_controllers[channel_index].return_queue.push({ req.depart, (uint)req.source_id });
	});

	if (enqueue_success)
	{
		//std::cout << "Load channel_index: " << channel_index << std::endl;
		MemoryReturn& ret = _returns[return_id];
		ret = MemoryReturn(request, _data_u8 + request.paddr);
		log.loads++;
		if(_load_map[request.paddr]++ == 0) log.unique_loads++;
		if(_row_map[request.paddr & ~0x1fff]++ == 0) log.unique_rows++;
	}

	return enqueue_success;
}

bool UnitDRAMRamulator::_store(const MemoryRequest& request, uint channel_index)
{
	//interface with ramulator
	bool enqueue_success = _controllers[channel_index].ramulator2_frontend->receive_external_requests(1, _convert_address(request.paddr), -1, [this](Ramulator::Request& req)
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
	
	bool busy = _pending_requests > 0;
	for(uint controller_index = 0; controller_index < _controllers.size(); ++controller_index)
	{
		_controllers[controller_index].req_pipline.clock();
		if(_request_network.is_read_valid(controller_index) && _controllers[controller_index].req_pipline.is_write_valid())
			_controllers[controller_index].req_pipline.write(_request_network.read(controller_index));

		if(!_controllers[controller_index].req_pipline.empty()) busy = true;
		if(!_controllers[controller_index].req_pipline.is_read_valid()) continue;
		const MemoryRequest& request = _controllers[controller_index].req_pipline.peek();

		if (request.type == MemoryRequest::Type::STORE)
		{
			if (_store(request, controller_index))
			{
				_controllers[controller_index].req_pipline.read();
			}
		}
		else if (request.type == MemoryRequest::Type::LOAD)
		{
			if (_load(request, controller_index))
			{
				_controllers[controller_index].req_pipline.read();
				_pending_requests++;
			}
		}

	}

	if(busy)
	{
		if(!_busy)
		{
			_busy = true;
			simulator->units_executing++;
		}
	}
	else
	{
		if(_busy)
		{
			_busy = false;
			simulator->units_executing--;
		}
	}
}

void UnitDRAMRamulator::clock_fall()
{
	double delta = _fractional_cycle + _clock_ratio;
	uint clock_cycles = (uint)delta;
	_fractional_cycle = std::fmod(delta, 1.0);
	for(uint i = 0; i < clock_cycles; ++i)
	{
		++_current_cycle;
		for(uint j = 0; j < _controllers.size(); ++j)
			_controllers[j].ramulator2_memorysystem->tick();
	}

	for(uint controller_index = 0; controller_index < _controllers.size(); ++controller_index)
	{
		MemoryController& controller = _controllers[controller_index];
		if(!controller.return_queue.empty())
		{
			const RamulatorReturn& ramulator_return = controller.return_queue.top();
			const MemoryReturn& ret = _returns[ramulator_return.return_id];
			if(_return_network.is_write_valid(controller_index))
			{
			#if ENABLE_DRAM_DEBUG_PRINTS
				printf("Load Return(%d): 0x%llx\n", ret.port, ret.paddr);
			#endif
				_assert(_current_cycle >= ramulator_return.return_cycle);
				log.bytes_read += ret.size;
				_return_network.write(ret, controller_index);
				_free_return_ids.push(ramulator_return.return_id);
				controller.return_queue.pop();
				_pending_requests--;
			}
		}
	}

	_return_network.clock();
}



}}