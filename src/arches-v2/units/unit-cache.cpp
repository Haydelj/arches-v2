#include "unit-cache.hpp"

namespace Arches {namespace Units {

UnitCache::UnitCache(Configuration config) :
	UnitCacheBase(config.size, config.block_size, config.associativity),
	_request_network(config.num_ports, config.num_partitions * config.num_banks, config.partition_select_mask | config.bank_select_mask, config.crossbar_width),
	_return_network(config.num_partitions * config.num_banks, config.num_ports, config.crossbar_width),
	_bank_select_mask(config.bank_select_mask),
	_mem_highers(config.mem_highers),
	_in_order(config.in_order)
{
	_assert(popcnt(config.partition_select_mask) == log2i(config.num_partitions));
	_assert(popcnt(config.bank_select_mask) == log2i(config.num_banks));
	_assert((config.partition_select_mask & config.bank_select_mask) == 0x0ull);

	_partitions.reserve(config.num_partitions);
	for(uint i = 0; i < config.num_partitions; ++i)
	{
		_partitions.push_back(config);
		config.mem_higher_port += config.mem_higher_port_stride;
	}
}

UnitCache::Partition::Partition(Configuration config) :
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
	request_pipline(config.input_latency),
	return_pipline(config.output_latency),
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
	for(uint p = 0; p < _partitions.size(); ++p)
	{ 
		Partition& partition = _partitions[p];
		for(UnitMemoryBase* mem_higher : _mem_highers)
		{
			if(mem_higher->return_port_read_valid(partition.mem_higher_port))
			{
				MemoryReturn ret = mem_higher->read_return(partition.mem_higher_port);
				_assert(ret.paddr == _get_block_addr(ret.paddr));

				//Mark the associated lse as filled and put it in the return queue
				paddr_t block_addr = ret.paddr;
				if(_update_block(block_addr, ret.data)) //Update block. Might have been evicted in which case this does nothing and returns null
					log.data_array_writes++;

				//fill all subentries and queue for return
				MSHR& mshr = partition.mshrs[block_addr];
				Bank& bank = partition.banks[_get_bank(block_addr)];
				while(!mshr.sub_entries.empty())
				{
					uint16_t rob_index = mshr.sub_entries.front();
					mshr.sub_entries.pop();

					MemoryReturn& buffer = bank.rob[rob_index];
					uint offset = buffer.paddr - block_addr;
					std::memcpy(buffer.data, ret.data + offset, ret.size);

					bank.return_queue.push(rob_index);
				}

				//free mshr
				partition.mshrs.erase(block_addr);
				partition.recived_return = true;
			}
			else partition.recived_return = false;
		}
	}
}

void UnitCache::_recive_request()
{
	for(uint p = 0; p < _partitions.size(); ++p)
	{
		Partition& partition = _partitions[p];
		for(uint b = 0; b < partition.banks.size(); ++b)
		{
			Bank& bank = partition.banks[b];
			if(!bank.request_pipline.is_read_valid()) continue;
			if(!partition.miss_queue.is_write_valid(b))
			{
				log.mshr_stalls++;
				continue;
			}

			MemoryRequest request = bank.request_pipline.peek();
			paddr_t block_addr = _get_block_addr(request.paddr);
			paddr_t block_offset = _get_block_offset(request.paddr);

			if(request.type == MemoryRequest::Type::LOAD)
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
				uint8_t* block_data = _get_block(block_addr);
				log.tag_array_access++;

				if(block_data)
				{
					//Hit: fill request and insert into return queue
					bank.rob[rob_index] = MemoryReturn(request, block_data + block_offset);
					bank.return_queue.push(rob_index);
					log.data_array_reads++;
					log.hits++;
				}
				else
				{
					bank.rob[rob_index] = MemoryReturn(request);
					partition.miss_queue.write({block_addr, rob_index}, b);
				}

				//Update buffer and reserve it
				bank.rob_size++;
				if(!_in_order) bank.rob_free_set.erase(rob_index);
			}
			else if(request.type == MemoryRequest::Type::PREFECTH)
			{
				//check data array
				uint8_t* block_data = _get_block(block_addr);
				log.tag_array_access++;

				if(!block_data)
					partition.miss_queue.write({block_addr, ~0u}, b);
			}
			else if(request.type == MemoryRequest::Type::STORE)
			{
				bool cached = false; //TODO actually derive from request
				if(cached)
				{
					//Check data array
					uint8_t* block_data = _get_block(block_addr);
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
				}
				else
				{
					log.uncached_writes++;
				}

				//Forward store
				MemoryRequest forward_request(request);
				forward_request.port = partition.mem_higher_port;
				partition.mem_higher_request_queue.push(forward_request);
			}
			else _assert(false);

			//pop the request
			bank.request_pipline.read();
		}



		//Proccess misses
		partition.miss_queue.clock();
		//if(partition.recived_return) continue;
		if(!partition.miss_queue.is_read_valid(0)) continue;
		paddr_t block_addr = partition.miss_queue.peek(0).block_addr;

		//Try to fetch an mshr for the line or allocate a new mshr for the line
		if(partition.mshrs.find(block_addr) == partition.mshrs.end())
		{
			//Didn't find mshr. Try to allocate one
			if(partition.mshrs.size() < partition.num_mshr)
			{
				//Allocate a new block for the miss to return to
				_insert_block(block_addr);

				//Allocated a new MSHR and queue up the fill request
				MemoryRequest miss_req;
				miss_req.type = MemoryRequest::Type::LOAD;
				miss_req.paddr = block_addr;
				miss_req.size = _block_size;
				miss_req.port = partition.mem_higher_port;
				partition.mem_higher_request_queue.push(miss_req);

				log.misses++;
			}
			else
			{
				//Out of MSHRs
				continue;
			}
		}
		else
		{
			//Found a mshr log a half miss
			log.half_misses++;
		}

		uint request_buffer_index = partition.miss_queue.peek(0).request_buffer_index;
		if(request_buffer_index != ~0u)
			partition.mshrs[block_addr].sub_entries.push(request_buffer_index);

		partition.miss_queue.read(0);
	}
}

void UnitCache::_send_request()
{
	for(uint p = 0; p < _partitions.size(); ++p)
	{
		Partition& partition = _partitions[p];
		if(partition.mem_higher_request_queue.empty()) continue;

		const MemoryRequest& request = partition.mem_higher_request_queue.front();
		UnitMemoryBase* mem_higher = _get_mem_higher(request.paddr);

		_assert(request.port == partition.mem_higher_port);
		if(!mem_higher->request_port_write_valid(request.port)) continue;

		mem_higher->write_request(request);
		partition.mem_higher_request_queue.pop();
	}
}

void UnitCache::_send_return()
{
	for(uint p = 0; p < _partitions.size(); ++p)
	{
		Partition& partition = _partitions[p];
		for(uint b = 0; b < partition.banks.size(); ++b)
		{
			Bank& bank = partition.banks[b];
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
	for(uint p = 0; p < _partitions.size(); ++p)
	{
		Partition& partition = _partitions[p];
		for(uint b = 0; b < partition.banks.size(); ++b)
		{
			Bank& bank = partition.banks[b];
			uint i = p * partition.banks.size() + b;

			if(_request_network.is_read_valid(i) && bank.request_pipline.is_write_valid())
				bank.request_pipline.write(_request_network.read(i));
			bank.request_pipline.clock();
		}
	}

	_recive_return();
	_recive_request();
}

void UnitCache::clock_fall()
{
	_send_request();
	_send_return();

	for(uint p = 0; p < _partitions.size(); ++p)
	{
		Partition& partition = _partitions[p];
		for(uint b = 0; b < partition.banks.size(); ++b)
		{
			Bank& bank = partition.banks[b];
			uint i = p * partition.banks.size() + b;

			bank.return_pipline.clock();
			if(bank.return_pipline.is_read_valid() && _return_network.is_write_valid(i))
				_return_network.write(bank.return_pipline.read(), i);
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