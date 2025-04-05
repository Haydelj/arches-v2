#include "unit-stream-cache.hpp"

namespace Arches {namespace Units {

UnitStreamCache::UnitStreamCache(Configuration config) :
	UnitCacheBase(config.size, config.block_size, config.associativity, config.sector_size),
	_request_network(config.num_ports, config.num_banks, config.block_size, config.crossbar_width),
	_return_network(config.num_banks, config.num_ports, config.crossbar_width),
	_mem_highers(config.mem_highers),
	_level(config.level)
{
	mem_higher_port = config.mem_higher_port;
	banks.reserve(config.num_banks);
	for(uint i = 0; i < config.num_banks; ++i)
		banks.push_back(config);
}

UnitStreamCache::Bank::Bank(Configuration config) :
	request_pipline(config.latency), return_pipline(1) {}

UnitStreamCache::~UnitStreamCache()
{

}

void UnitStreamCache::_recive_return()
{
	for(UnitMemoryBase* mem_higher : _mem_highers)
	{
		if(mem_higher->return_port_read_valid(mem_higher_port))
		{
			MemoryReturn ret = mem_higher->peek_return(mem_higher_port);
			bool cached = !(ret.flags.omit_cache & (0x1 << _level));
			if(cached)
			{
				//fill all subentries and queue for return
				paddr_t sector_addr = _get_sector_addr(ret.paddr);
				paddr_t block_addr = _get_block_addr(ret.paddr);
				uint sector_index = _get_sector_index(ret.paddr);

				//Mark the associated lse as filled and put it in the return queue
				if(_write_sector(sector_addr, ret.data, false)) //Update block. Might have been evicted in which case this does nothing and returns null
					log.data_array_writes++;

				uint b = _get_bank(ret.paddr);
				Bank& bank = banks[b];
				if(bank.return_pipline.is_write_valid())
				{
					uint offset = ret.dst.pop(5);
					ret.size = ret.dst.pop(6);
					ret.port = ret.dst.pop(8);
					std::memcpy(ret.data, ret.data + offset, ret.size);
					ret.paddr += offset;

					bank.return_pipline.write(ret);
					log.bytes_read += ret.size;
					mem_higher->read_return(mem_higher_port);
				}
			}
			else
			{
				uint b = _get_bank(ret.paddr);
				Bank& bank = banks[b];
				if(bank.return_pipline.is_write_valid())
				{
					ret.port = ret.dst.pop(8);
					bank.return_pipline.write(ret);
					log.bytes_read += ret.size;
					mem_higher->read_return(mem_higher_port);
				}
			}
		}
	}
}

void UnitStreamCache::_recive_request()
{
	for(uint b = 0; b < banks.size(); ++b)
	{
		Bank& bank = banks[b];
		if(!bank.request_pipline.is_read_valid() || !bank.return_pipline.is_write_valid()) continue;

		MemoryRequest request = bank.request_pipline.peek();
		paddr_t sector_addr = _get_sector_addr(request.paddr);
		paddr_t sector_offset = _get_sector_offset(request.paddr);
		uint8_t sector_index = _get_sector_index(request.paddr);

		bool cached = !(request.flags.omit_cache & (0x1 << _level));
		if(!cached)
		{
			//Forward request
			request.dst.push(request.port, 8);
			request.port = mem_higher_port;
			mem_higher_request_queue.push(request);
			log.uncached_requests++;
		}
		else if(request.type == MemoryRequest::Type::LOAD)
		{
			//check data array
			uint8_t* sector_data = _read_sector(sector_addr);
			log.tag_array_access++;

			if(sector_data)
			{
				//Hit: fill request and insert into return queue
				bank.return_pipline.write(MemoryReturn(request, sector_data + sector_offset));
				log.bytes_read += request.size;
				log.data_array_reads++;
				log.hits++;
			}
			else
			{
				_allocate_block(sector_addr);

				//Forward request
				request.dst.push(request.port, 8);
				request.dst.push(request.size, 6);
				request.dst.push(sector_offset, 5);
				request.paddr = sector_addr;
				request.size = _sector_size;
				request.port = mem_higher_port;
				mem_higher_request_queue.push(request);
				log.misses++;
			}
		}
		else _assert(false);

		//pop the request
		bank.request_pipline.read();
	}
}

void UnitStreamCache::_send_request()
{
	if(!mem_higher_request_queue.empty())
	{
		const MemoryRequest& request = mem_higher_request_queue.front();
		UnitMemoryBase* mem_higher = _get_mem_higher(request.paddr);

		_assert(request.port == mem_higher_port);
		if(mem_higher->request_port_write_valid(request.port))
		{
			mem_higher->write_request(request);
			mem_higher_request_queue.pop();
		}
	}
}

void UnitStreamCache::clock_rise()
{
	for(uint i = 0; i < 2; ++i) _request_network.clock();

	for(uint b = 0; b < banks.size(); ++b)
	{
		Bank& bank = banks[b];
		uint port = b;

		if(_request_network.is_read_valid(port) && bank.request_pipline.is_write_valid())
			bank.request_pipline.write(_request_network.read(port));
		bank.request_pipline.clock();
	}

	_recive_return();
	_recive_request();
}

void UnitStreamCache::clock_fall()
{
	_send_request();

	for(uint b = 0; b < banks.size(); ++b)
	{
		Bank& bank = banks[b];
		uint port = b;

		bank.return_pipline.clock();
		if(bank.return_pipline.is_read_valid() && _return_network.is_write_valid(port))
			_return_network.write(bank.return_pipline.read(), port);
	}

	for(uint i = 0; i < 2; ++i) _return_network.clock();
}

bool UnitStreamCache::request_port_write_valid(uint port_index)
{
	return _request_network.is_write_valid(port_index);
}

void UnitStreamCache::write_request(const MemoryRequest& request)
{
	_request_network.write(request, request.port);
}

bool UnitStreamCache::return_port_read_valid(uint port_index)
{
	return _return_network.is_read_valid(port_index);
}

const MemoryReturn& UnitStreamCache::peek_return(uint port_index)
{
	return _return_network.peek(port_index);
}

const MemoryReturn UnitStreamCache::read_return(uint port_index)
{
	return _return_network.read(port_index);
}

}}