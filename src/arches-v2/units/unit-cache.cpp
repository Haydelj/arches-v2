#include "unit-cache.hpp"

namespace Arches {namespace Units {

UnitCache::UnitCache(Configuration config) :
	UnitCacheBase(config.size, config.block_size, config.associativity, config.sector_size),
	_request_network(config.num_ports, config.num_slices * config.num_banks, config.slice_select_mask | config.bank_select_mask, config.crossbar_width),
	_return_network(config.num_slices * config.num_banks, config.num_ports, config.crossbar_width),
	_bank_select_mask(config.bank_select_mask), 
	_prefetch_block(config.prefetch_block),
	_mem_highers(config.mem_highers),
	_in_order(config.in_order),
	_level(config.level)
{
	_assert(popcnt(config.slice_select_mask) == log2i(config.num_slices));
	_assert(popcnt(config.bank_select_mask) == log2i(config.num_banks));
	_assert((config.slice_select_mask & config.bank_select_mask) == 0x0ull);

	_slices.reserve(config.num_slices);
	for(uint i = 0; i < config.num_slices; ++i)
	{
		_slices.push_back(config);
		config.mem_higher_port += config.mem_higher_port_stride;
	}
}

UnitCache::Slice::Slice(Configuration config) :
	miss_queue(config.num_banks, 1, 4, 4),
	num_mshr(config.num_mshr),
	recived_return(false)
{
	mem_higher_port = config.mem_higher_port;

	banks.reserve(config.num_banks);
	for(uint i = 0; i < config.num_banks; ++i)
		banks.push_back(config);
}

UnitCache::Bank::Bank(Configuration config) :
	request_pipline(config.latency),
	return_pipline(1),
	rob_filled(config.rob_size, false),
	rob_head(0),
	rob(config.rob_size),
	rob_size(0)
{
	for(uint i = 0; i < rob.size(); ++i)
		rob_free_set.insert(i);
}

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
					//fill all subentries and queue for return
					bool set_dirty = false;
					paddr_t block_addr = _get_block_addr(ret.paddr);
					paddr_t block_offset = _get_block_offset(ret.paddr);
					MSHR& mshr = slice.mshrs[block_addr];
					Bank& bank = slice.banks[_get_bank(block_addr)];

					std::memcpy(mshr.block_data + block_offset, ret.data, ret.size);
					mshr.sectors_filled |= 1 << (block_offset / _sector_size);

					if(mshr.sectors_filled == mshr.sectors_requested)
					{
						while(!mshr.sub_entries.empty())
						{
							uint16_t rob_index = mshr.sub_entries.front();
							mshr.sub_entries.pop();

							MemoryReturn& buffer = bank.rob[rob_index];
							uint offset = buffer.paddr - block_addr;
							if(buffer.type == MemoryRequest::Type::LOAD)
							{
								std::memcpy(buffer.data, mshr.block_data + offset, buffer.size);
							}
							else if(buffer.type == MemoryRequest::Type::CSHIT)
							{
								//play the CSHIT against the line
								rtm::Hit cur_hit, new_hit;
								std::memcpy(&new_hit, buffer.data, sizeof(rtm::Hit));
								std::memcpy(&cur_hit, mshr.block_data + offset, sizeof(rtm::Hit));
								if(new_hit.t < cur_hit.t)
								{
									set_dirty = true;
									std::memcpy(mshr.block_data + offset, buffer.data, sizeof(rtm::Hit));
								}
							}
							else _assert(false);
							bank.return_queue.push(rob_index);
						}

						//Mark the associated lse as filled and put it in the return queue
						for(uint i = 0; i < _block_size / _sector_size; ++i)
							if((mshr.sectors_filled >> i) & 0x1)
								if(_write_sector(block_addr + _sector_size * i, mshr.block_data + _sector_size * i, set_dirty)) //Update block. Might have been evicted in which case this does nothing and returns null
									log.data_array_writes++;

						//free mshr
						slice.mshrs.erase(block_addr);
					}

					mem_higher->read_return(slice.mem_higher_port);
					slice.recived_return = true;
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
						log.bytes_read += ret.size;
						mem_higher->read_return(slice.mem_higher_port);
					}
				}
			}
			else slice.recived_return = false;
		}
	}
}

uint8_t UnitCache::_sector_mask(const MemoryRequest& req)
{
	uint s0 = _get_sector_index(req.paddr);
	uint sc = (req.size + _sector_size - 1) / _sector_size;
	return generate_nbit_mask(sc) << s0;;
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
			if(!slice.miss_queue.is_write_valid(b))
			{
				log.mshr_stalls++;
				continue;
			}

			MemoryRequest request = bank.request_pipline.peek();
			paddr_t block_addr = _get_block_addr(request.paddr);
			paddr_t block_offset = _get_block_offset(request.paddr);
			uint8_t sector_mask = _sector_mask(request);
			if(_prefetch_block) sector_mask = generate_nbit_mask(_block_size / _sector_size);

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
				//check if we have room in the return buffer. If we don't stall
				if(bank.rob_size >= bank.rob.size())
				{
					log.rob_stalls++;
					continue;
				}

				uint rob_index;
				if(_in_order) rob_index = (bank.rob_head + bank.rob_size) % bank.rob.size();
				else          rob_index = *bank.rob_free_set.begin();

				//check data array
				uint8_t valid_bits = 0;
				uint8_t* block_data = _read_block(block_addr, valid_bits);
				log.tag_array_access++;

				uint8_t hit_sectors = (sector_mask & valid_bits);
				uint8_t miss_sectors = (sector_mask & ~valid_bits);
				log.hits += popcnt(hit_sectors);
				if(!miss_sectors)
				{
					//Hit: fill request and insert into return queue
					bank.rob[rob_index] = MemoryReturn(request, block_data + block_offset);
					bank.return_queue.push(rob_index);
					log.data_array_reads++;
				}
				else
				{
					bank.rob[rob_index] = MemoryReturn(request);
					slice.miss_queue.write({block_addr, rob_index, sector_mask}, b);
				}

				//Update buffer and reserve it
				bank.rob_size++;
				if(!_in_order) bank.rob_free_set.erase(rob_index);
			}
			else if(request.type == MemoryRequest::Type::STORE)
			{
				//Check data array
				uint8_t* block_data = _read_block(block_addr);
				log.tag_array_access++;

				if(block_data)
				{
					//Update block
					std::memcpy(block_data + block_offset, request.data, request.size);
					log.data_array_writes++;
					log.hits++;
				}
				else
				{
					//Do nothing
					log.misses++;
				}

				//Forward store
				MemoryRequest forward_request(request);
				forward_request.port = slice.mem_higher_port;
				slice.mem_higher_request_queue.push(forward_request);
			}
			else if(request.type == MemoryRequest::Type::PREFECTH)
			{
				//check data array
				uint8_t* block_data = _read_block(block_addr);
				log.tag_array_access++;

				if(!block_data)
					slice.miss_queue.write({block_addr, ~0u}, b);
			}
			else if(request.type == MemoryRequest::Type::CSHIT)
			{
				//check if we have room in the return buffer. If we don't stall
				if(bank.rob_size >= bank.rob.size())
				{
					log.rob_stalls++;
					continue;
				}

				uint rob_index;
				if(_in_order) rob_index = (bank.rob_head + bank.rob_size) % bank.rob.size();
				else          rob_index = *bank.rob_free_set.begin();

				//check data array
				uint8_t* block_data = _read_block(block_addr);
				log.tag_array_access++;

				if(block_data)
				{
					//Hit: play the CSHIT against the line
					rtm::Hit cur_hit, new_hit;
					std::memcpy(&new_hit, request.data, sizeof(rtm::Hit));
					std::memcpy(&cur_hit, block_data + block_offset, sizeof(rtm::Hit));
					if(new_hit.t < cur_hit.t)
					{
						std::memcpy(block_data + block_offset, request.data, sizeof(rtm::Hit));
						_write_sector(block_addr, block_data, true);
					}
					log.data_array_reads++;
					log.hits++;
				}
				else
				{
					bank.rob[rob_index] = MemoryReturn(request, request.data);
					slice.miss_queue.write({block_addr, rob_index}, b);
					if(!_in_order) bank.rob_free_set.erase(rob_index);
					bank.rob_size++;
				}
			}
			else _assert(false);

			//pop the request
			bank.request_pipline.read();
		}



		//Proccess misses
		slice.miss_queue.clock();
		//if(partition.recived_return) continue;
		if(!slice.miss_queue.is_read_valid(0)) continue;
		const Miss& miss = slice.miss_queue.peek(0);

		//Try to fetch an mshr for the line or allocate a new mshr for the line
		if(slice.mshrs.find(miss.block_addr) == slice.mshrs.end())
		{
			//Didn't find mshr. Try to allocate one
			if(slice.mshrs.size() < slice.num_mshr)
			{
				//Allocate a new block for the miss to return to
				Victim victim = _insert_block(miss.block_addr);

				for(uint i = 0; i < _block_size / _sector_size; ++i)
				{
					if(((victim.valid & victim.dirty) >> i) & 0x1)
					{
						MemoryRequest vic_req;
						vic_req.type = MemoryRequest::Type::STORE;
						vic_req.paddr = victim.addr + i * _sector_size;
						vic_req.size = _sector_size;
						vic_req.port = slice.mem_higher_port;
						std::memcpy(vic_req.data, victim.data + i * _sector_size, _sector_size);
						slice.mem_higher_request_queue.push(vic_req);
					}
				}

				//Allocated a new MSHR and queue up the fill request
				MSHR& mshr = slice.mshrs[miss.block_addr];
				mshr.sectors_filled = 0;
				mshr.sectors_requested = 0;
			}
			else
			{
				//Out of MSHRs
				continue;
			}
		}
		//else Found a mshr

		if(miss.request_buffer_index != ~0u)
			slice.mshrs[miss.block_addr].sub_entries.push(miss.request_buffer_index);

		MSHR& mshr = slice.mshrs[miss.block_addr];
		for(uint i = 0; i < _block_size / _sector_size; ++i)
		{
			if((miss.sectors >> i) & 0x1)
			{
				if((~mshr.sectors_requested >> i) & 0x1)
				{
					log.misses++;
					MemoryRequest miss_req;
					miss_req.type = MemoryRequest::Type::LOAD;
					miss_req.paddr = miss.block_addr + i * _sector_size;
					miss_req.size = _sector_size;
					miss_req.port = slice.mem_higher_port;
					slice.mem_higher_request_queue.push(miss_req);
				}
				else
				{
					log.half_misses++;
				}
			}
		}
		mshr.sectors_requested |= miss.sectors;

		slice.miss_queue.read(0);
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

void UnitCache::_send_return()
{
	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			if(_in_order)
			{
				//In order return logic
				if(!bank.return_queue.empty())
				{
					bank.rob_filled[bank.return_queue.front()] = true;
					bank.return_queue.pop();
				}

				if(bank.rob_size > 0 && bank.rob_filled[bank.rob_head] && bank.return_pipline.is_write_valid())
				{
					uint rob_index = bank.rob_head;
					bank.rob_filled[rob_index] = false;
					bank.rob_head = (bank.rob_head + 1) % bank.rob.size();
					bank.rob_size--;

					const MemoryReturn& ret = bank.rob[rob_index];
					if(ret.type != MemoryRequest::Type::CSHIT)
						bank.return_pipline.write(ret);
					log.bytes_read += ret.size;
				}
			}
			else
			{
				//Out of order return logic
				if(!bank.return_queue.empty() && bank.return_pipline.is_write_valid())
				{
					uint rob_index = bank.return_queue.front();
					bank.rob_filled[rob_index] = true;
					bank.return_queue.pop();

					const MemoryReturn& ret = bank.rob[rob_index];
					if(ret.type != MemoryRequest::Type::CSHIT)
						bank.return_pipline.write(ret);
					log.bytes_read += ret.size;

					bank.rob_free_set.insert(rob_index);
					bank.rob_size--;
				}
			}
		}
	}
}

void UnitCache::clock_rise()
{
	_request_network.clock();

	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			uint i = s * slice.banks.size() + b;

			bank.request_pipline.clock();
			if(_request_network.is_read_valid(i) && bank.request_pipline.is_write_valid())
				bank.request_pipline.write(_request_network.read(i));
		}
	}

	_recive_return();
	_recive_request();
}

void UnitCache::clock_fall()
{
	_send_request();
	_send_return();

	for(uint s = 0; s < _slices.size(); ++s)
	{
		Slice& slice = _slices[s];
		for(uint b = 0; b < slice.banks.size(); ++b)
		{
			Bank& bank = slice.banks[b];
			uint i = s * slice.banks.size() + b;

			if(bank.return_pipline.is_read_valid() && _return_network.is_write_valid(i))
				_return_network.write(bank.return_pipline.read(), i);

			bank.return_pipline.clock();
		}
	}

	_return_network.clock();
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