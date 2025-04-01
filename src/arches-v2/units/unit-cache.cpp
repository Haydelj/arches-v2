#include "unit-cache.hpp"

namespace Arches {namespace Units {

UnitCache::UnitCache(Configuration config) :
	UnitCacheBase(config.size, config.block_size, config.associativity, config.sector_size, config.policy),
	_request_network(config.num_ports, config.num_slices * config.num_banks, config.block_size, config.crossbar_width),
	_return_network(config.num_slices * config.num_banks, config.num_ports, config.crossbar_width),
	_mem_highers(config.mem_highers),
	_level(config.level), _block_prefetch(config.block_prefetch), _num_mshr(config.num_mshr), _num_subentries(config.num_subentries), _miss_alloc(config.miss_alloc)
{
	_slices.reserve(config.num_slices);
	for(uint i = 0; i < config.num_slices; ++i)
	{
		_slices.push_back(config);
		config.mem_higher_port += config.mem_higher_port_stride;
	}
}

UnitCache::Slice::Slice(Configuration config) :
	miss_network(config.num_banks, 1, 4, 128)
{
	mem_higher_port = config.mem_higher_port;

	banks.reserve(config.num_banks);
	for(uint i = 0; i < config.num_banks; ++i)
		banks.push_back(config);
}

UnitCache::Bank::Bank(Configuration config) :
	request_pipline(config.latency), return_pipline(1), return_queue(32) {}

UnitCache::~UnitCache()
{

}

void UnitCache::_recive_return()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{ 
		Slice& slice = _slices[s];
		for(UnitMemoryBase* mem_higher : _mem_highers)
		{
			if(mem_higher->return_port_read_valid(slice.mem_higher_port))
			{
				MemoryReturn ret = mem_higher->peek_return(slice.mem_higher_port);
				bool cached = !(ret.flags.omit_cache & (0x1 << _level));
				if(cached)
				{
					paddr_t sector_addr = _get_sector_addr(ret.paddr);
					MSHR& mshr = slice.mshrs[sector_addr];
					uint b = _get_bank(ret.paddr);
					Bank& bank = slice.banks[b];

					if(!_miss_alloc) _allocate_block(sector_addr);
					_write_sector(sector_addr, ret.data, false);

					//fill a subentry and queue for return
					if(bank.return_pipline.is_write_valid() && !mshr.subentries.empty())
					{
						MemoryRequest& sube_req = mshr.subentries.front();
						uint sector_offset = _get_sector_offset(sube_req.paddr);
						bank.return_pipline.write(MemoryReturn(sube_req, ret.data + sector_offset));
						mshr.subentries.pop();
					}

					if(mshr.subentries.empty())
					{
						mem_higher->read_return(slice.mem_higher_port);
						slice.mshrs.erase(sector_addr); //free mshr
					}
				}
				else
				{
					uint b = _get_bank(ret.paddr);
					Bank& bank = slice.banks[b];
					if(bank.return_pipline.is_write_valid())
					{
						uint i = s * slice.banks.size() + b;
						ret.port = ret.dst.pop(8);
						bank.return_pipline.write(ret);
						mem_higher->read_return(slice.mem_higher_port);
					}
				}
			}
		}
	}
}

void UnitCache::_recive_request()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			if(!bank.request_pipline.is_read_valid()) continue;
			if(!bank.return_queue.is_write_valid() || !slice.miss_network.is_write_valid(b))
			{
				log.mshr_stalls++;
				continue;
			}

			MemoryRequest request = bank.request_pipline.peek();
			paddr_t sector_addr = _get_sector_addr(request.paddr);
			paddr_t sector_offset = _get_sector_offset(request.paddr);
			uint8_t sector_index = _get_sector_index(request.paddr);

			bool cached = !(request.flags.omit_cache & (0x1 << _level));
			if(!cached)
			{
				//Forward request
				request.dst.push(request.port, 8);
				request.port = slice.mem_higher_port;
				slice.mem_higher_request_queue.push(request);
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
					bank.return_queue.write(MemoryReturn(request, sector_data + sector_offset));
					log.data_array_reads++;
					log.hits++;
				}
				else
				{
					//Miss: allocate a block and insert into miss queue
					if(_miss_alloc) _allocate_block(sector_addr);
					slice.miss_network.write(request, b);
				}
			}
			else _assert(false);

			//pop the request
			bank.request_pipline.read();
		}

		//Proccess misses
		slice.miss_network.clock();
		if(!slice.miss_network.is_read_valid(0)) continue;
		const MemoryRequest& miss = slice.miss_network.peek(0);

		//Try to fetch an mshr for the line or allocate a new mshr for the line
		bool request_sector = false;
		paddr_t sector_addr = _get_sector_addr(miss.paddr);
		if(slice.mshrs.find(sector_addr) == slice.mshrs.end())
		{
			//Didn't find mshr. Try to allocate one
			if(slice.mshrs.size() < _num_mshr) 
				MSHR& mshr = slice.mshrs[sector_addr]; //Allocated a new MSHR
			else continue; //Out of MSHRs
			request_sector = true;
		}

		MSHR& mshr = slice.mshrs[sector_addr];
		if(mshr.subentries.size() < _num_subentries)
		{
			mshr.subentries.push(miss);
			slice.miss_network.read(0);
			if(request_sector)
			{
				MemoryRequest mshr_fill_req;
				mshr_fill_req.type = MemoryRequest::Type::LOAD;
				mshr_fill_req.paddr = sector_addr;
				mshr_fill_req.size = _sector_size;
				mshr_fill_req.port = slice.mem_higher_port;
				slice.mem_higher_request_queue.push(mshr_fill_req);
				log.misses++;
			}
			else log.half_misses++;
		}

		if(_block_prefetch)
		{
			paddr_t block_address = _get_block_addr(sector_addr);
			for(uint i = 0; i < _block_size; i += _sector_size)
			{
				sector_addr = block_address + i;
				if(slice.mshrs.find(sector_addr) == slice.mshrs.end())
				{
					slice.mshrs.insert({sector_addr, MSHR()});
					MemoryRequest mshr_fill_req;
					mshr_fill_req.type = MemoryRequest::Type::LOAD;
					mshr_fill_req.paddr = sector_addr;
					mshr_fill_req.size = _sector_size;
					mshr_fill_req.port = slice.mem_higher_port;
					slice.mem_higher_request_queue.push(mshr_fill_req);
				}
			}
		}
	}
}

void UnitCache::_send_request()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		if(slice.mem_higher_request_queue.empty()) continue;

		const MemoryRequest& request = slice.mem_higher_request_queue.front();
		UnitMemoryBase* mem_higher = _get_mem_higher(request.paddr);

		_assert(request.port == slice.mem_higher_port);
		if(!mem_higher->request_port_write_valid(request.port)) continue;

		mem_higher->write_request(request);
		slice.mem_higher_request_queue.pop();
	}
}

void UnitCache::clock_rise()
{
	for(uint i = 0; i < 2; ++i) _request_network.clock();

	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			uint port = s * slice.banks.size() + b;

			if(_request_network.is_read_valid(port) && bank.request_pipline.is_write_valid())
				bank.request_pipline.write(_request_network.read(port));
			bank.request_pipline.clock();
		}
	}

	_recive_return();
	_recive_request();
}

void UnitCache::clock_fall()
{
	_send_request();

	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			uint port = s * slice.banks.size() + b;

			if(bank.return_pipline.is_write_valid() && bank.return_queue.is_read_valid())
				bank.return_pipline.write(bank.return_queue.read());

			bank.return_pipline.clock();
			if(bank.return_pipline.is_read_valid() && _return_network.is_write_valid(port))
			{
				log.bytes_read += bank.return_pipline.peek().size;
				_return_network.write(bank.return_pipline.read(), port);
			}
		}
	}

	for(uint i = 0; i < 2; ++i) _return_network.clock();
}

bool UnitCache::request_port_write_valid(uint port_index)
{
	return _request_network.is_write_valid(port_index);
}

void UnitCache::write_request(const MemoryRequest& request)
{
	_request_network.write(request, request.port);
}

bool UnitCache::return_port_read_valid(uint port_index)
{
	return _return_network.is_read_valid(port_index);
}

const MemoryReturn& UnitCache::peek_return(uint port_index)
{
	return _return_network.peek(port_index);
}

const MemoryReturn UnitCache::read_return(uint port_index)
{
	return _return_network.read(port_index);
}

}}