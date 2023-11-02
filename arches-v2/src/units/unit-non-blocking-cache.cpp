#include "unit-non-blocking-cache.hpp"

namespace Arches {namespace Units {

UnitNonBlockingCache::UnitNonBlockingCache(Configuration config) : 
	UnitCacheBase(config.size, config.associativity),
	_request_cross_bar(config.num_ports, config.num_banks, config.bank_select_mask),
	_return_cross_bar(config.num_ports, config.num_banks)
{
	_check_retired_lfb = config.check_retired_lfb;

	_mem_higher = config.mem_higher;
	_mem_higher_port_offset = config.mem_higher_port_offset;
	_mem_higher_port_stride = config.mem_higher_port_stride;

	_banks.resize(config.num_banks, {config.num_lfb, config.data_array_latency});
}

UnitNonBlockingCache::~UnitNonBlockingCache()
{

}

uint UnitNonBlockingCache::_fetch_lfb(uint bank_index, LFB& lfb)
{
	Bank& bank = _banks[bank_index];
	std::vector<LFB>& lfbs = bank.lfbs;

	for(uint32_t i = 0; i < lfbs.size(); ++i)
		if(lfbs[i].state != LFB::State::INVALID && lfbs[i] == lfb) return i;

	return ~0u;
}

uint UnitNonBlockingCache::_allocate_lfb(uint bank_index, LFB& lfb)
{
	std::vector<LFB>& lfbs = _banks[bank_index].lfbs;

	uint replacement_index = ~0u;
	uint replacement_lru = 0u;
	for(uint i = 0; i < lfbs.size(); ++i)
	{
		if(lfbs[i].state == LFB::State::INVALID)
		{
			replacement_index = i;
			break;
		}

		if(lfbs[i].state == LFB::State::RETIRED && lfbs[i].lru >= replacement_lru)
		{
			replacement_lru = lfbs[i].lru;
			replacement_index = i;
		}
	}

	//can't allocate
	if(replacement_index == ~0) return ~0;

	for(uint i = 0; i < lfbs.size(); ++i) lfbs[i].lru++;
	lfbs[replacement_index].lru = 0;
	lfbs[replacement_index] = lfb;
	return replacement_index;
}

uint UnitNonBlockingCache::_fetch_or_allocate_lfb(uint bank_index, uint64_t block_addr, LFB::Type type)
{
	std::vector<LFB>& lfbs = _banks[bank_index].lfbs;

	LFB lfb;
	lfb.block_addr = block_addr;
	lfb.type = type;
	lfb.state = LFB::State::EMPTY;
	
	uint lfb_index = _fetch_lfb(bank_index, lfb);
	if(lfb_index != ~0) return lfb_index;
	return _allocate_lfb(bank_index, lfb);
}

void UnitNonBlockingCache::_push_request(LFB& lfb, const MemoryRequest& request)
{
	LFB::SubEntry sub_entry;
	sub_entry.offset = _get_block_offset(request.paddr);
	sub_entry.size = request.size;
	sub_entry.port = request.port;
	sub_entry.dst = request.dst;
	lfb.sub_entries.push(sub_entry);
}

MemoryRequest UnitNonBlockingCache::_pop_request(LFB& lfb)
{
	LFB::SubEntry sub_entry = lfb.sub_entries.front();
	lfb.sub_entries.pop();

	MemoryRequest req;
	req.type = MemoryRequest::Type::LOAD;
	req.size = sub_entry.size;
	req.port = sub_entry.port;
	req.dst = sub_entry.dst;
	req.paddr = lfb.block_addr + sub_entry.offset;
	return req;
}

void UnitNonBlockingCache::_clock_data_array(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	bank.data_array_pipline.clock();
	if(!bank.data_array_pipline.is_read_valid()) return;

	uint lfb_index = bank.data_array_pipline.read();
	if(lfb_index == ~0u) return; 

	LFB& lfb = bank.lfbs[lfb_index];
	assert(lfb.state == LFB::State::DATA_ARRAY);
	lfb.state = LFB::State::FILLED;
	bank.lfb_return_queue.push(lfb_index);
}

bool UnitNonBlockingCache::_proccess_return(uint bank_index)
{
	uint mem_higher_port_index = bank_index * _mem_higher_port_stride + _mem_higher_port_offset;
	if(!_mem_higher->return_port_read_valid(mem_higher_port_index)) return false;

	const MemoryReturn ret = _mem_higher->read_return(mem_higher_port_index);
	assert(ret.paddr == _get_block_addr(ret.paddr));

	//Mark the associated lse as filled and put it in the return queue
	Bank& bank = _banks[bank_index];
	for(uint i = 0; i < bank.lfbs.size(); ++i)
	{
		LFB& lfb = bank.lfbs[i];
		if(lfb.block_addr == ret.paddr)
		{
			assert(lfb.state == LFB::State::MISSED);
			std::memcpy(lfb.block_data.bytes, ret.data, CACHE_BLOCK_SIZE);
			lfb.state = LFB::State::FILLED;
			bank.lfb_return_queue.push(i);
			break;
		}
	}

	//Insert block
	log.log_tag_array_access();
	log.log_data_array_write();
	_insert_block(ret.paddr, ret.data);

	if(bank.data_array_pipline.lantecy() != 0)
		bank.data_array_pipline.write(~0u);
	
	return true;
}

bool UnitNonBlockingCache::_proccess_request(uint bank_index)
{
	if(!_request_cross_bar.is_read_valid(bank_index)) return false;

	Bank& bank = _banks[bank_index];
	const MemoryRequest& request = _request_cross_bar.peek(bank_index);
	paddr_t block_addr = _get_block_addr(request.paddr);
	uint block_offset = _get_block_offset(request.paddr);
	log.log_requests();

	if(request.type == MemoryRequest::Type::LOAD)
	{
		//Try to fetch an lfb for the line or allocate a new lfb for the line
		uint lfb_index = _fetch_or_allocate_lfb(bank_index, block_addr, LFB::Type::READ);

		//In parallel access the tag array to check for the line
		BlockData* block_data = _get_block(block_addr);
		log.log_tag_array_access();

		//If the data array access is zero cycle then that means we did it in parallel with th tag lookup
		if(bank.data_array_pipline.lantecy() == 0)
		{
			log.log_data_array_read();
		}

		if(lfb_index != ~0)
		{
			LFB& lfb = bank.lfbs[lfb_index];
			_push_request(lfb, request);

			if(lfb.state == LFB::State::EMPTY)
			{
				if(block_data)
				{
					std::memcpy(lfb.block_data.bytes, block_data, CACHE_BLOCK_SIZE);

					//Copy line from data array to LFB
					if(bank.data_array_pipline.lantecy() == 0)
					{
						lfb.state = LFB::State::FILLED;
						bank.lfb_return_queue.push(lfb_index);
					}
					else
					{
						lfb.state = LFB::State::DATA_ARRAY;
						bank.data_array_pipline.write(lfb_index);
						log.log_data_array_read();
					}

					log.log_hit();
				}
				else
				{
					//Missed the cache queue up a request to mem higher
					lfb.state = LFB::State::MISSED;
					bank.lfb_request_queue.push(lfb_index);
					log.log_miss();
				}
			}
			else if(lfb.state == LFB::State::MISSED)
			{
				log.log_miss();
				log.log_half_miss();
			}
			else if(lfb.state == LFB::State::FILLED)
			{
				log.log_hit();
				log.log_lfb_hit();
			}
			else if(lfb.state == LFB::State::RETIRED)
			{
				//Wake up retired LFB and add it to the return queue
				lfb.state = LFB::State::FILLED;
				bank.lfb_return_queue.push(lfb_index);
				log.log_hit();
				log.log_lfb_hit();
			}

			_request_cross_bar.read(bank_index);
		}
		else log.log_lfb_stall();
	}
	else if(request.type == MemoryRequest::Type::STORE)
	{
		//try to allocate an lfb
		uint lfb_index = _fetch_or_allocate_lfb(bank_index, block_addr, LFB::Type::WRITE_COMBINING);
		if(lfb_index != ~0)
		{
			LFB& lfb = bank.lfbs[lfb_index];
			lfb.write_mask |= request.write_mask << block_offset;
			for(uint i = 0; i < request.size; ++i)
				if((request.write_mask >> i) & 0x1)
					lfb.block_data.bytes[block_offset + i] = request.data[i];
			
			if(lfb.state == LFB::State::EMPTY)
			{
				lfb.state = LFB::State::FILLED; //since we just filled it
				bank.lfb_request_queue.push(lfb_index);
			}

			_request_cross_bar.read(bank_index);
		}
	}

	return true;
}

void UnitNonBlockingCache::_try_request_lfb(uint bank_index)
{
	Bank& bank = _banks[bank_index];
	uint mem_higher_port_index = bank_index * _mem_higher_port_stride + _mem_higher_port_offset;

	if(bank.lfb_request_queue.empty() || !_mem_higher->request_port_write_valid(mem_higher_port_index)) return;
	
	LFB& lfb = bank.lfbs[bank.lfb_request_queue.front()];
	if(lfb.type == LFB::Type::READ)
	{
		assert(lfb.state == LFB::State::MISSED);

		MemoryRequest outgoing_request;
		outgoing_request.type = MemoryRequest::Type::LOAD;
		outgoing_request.size = CACHE_BLOCK_SIZE;
		outgoing_request.port = mem_higher_port_index;
		outgoing_request.paddr = lfb.block_addr;
		_mem_higher->write_request(outgoing_request, mem_higher_port_index);

		bank.lfb_request_queue.pop();
	}
	else if(lfb.type == LFB::Type::WRITE_COMBINING)
	{
		assert(lfb.state == LFB::State::FILLED);

		MemoryRequest outgoing_request;
		outgoing_request.type = MemoryRequest::Type::STORE;
		outgoing_request.size = CACHE_BLOCK_SIZE;
		outgoing_request.port = mem_higher_port_index;
		outgoing_request.write_mask = lfb.write_mask;
		outgoing_request.paddr = lfb.block_addr;
		std::memcpy(outgoing_request.data, lfb.block_data.bytes, CACHE_BLOCK_SIZE);
		_mem_higher->write_request(outgoing_request, mem_higher_port_index);

		lfb.state = LFB::State::INVALID;
		bank.lfb_request_queue.pop();
	}
}

void UnitNonBlockingCache::_try_return_lfb(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	//last return isn't complete do nothing
	if(bank.lfb_return_queue.empty() || !_return_cross_bar.is_write_valid(bank_index)) return;

	LFB& lfb = bank.lfbs[bank.lfb_return_queue.front()];

	//select the next subentry and copy return to interconnect

	MemoryRequest req = _pop_request(lfb);
	MemoryReturn ret(req, lfb.block_data.bytes + _get_block_offset(req.paddr));
	_return_cross_bar.write(ret, bank_index);

	if(lfb.sub_entries.empty())
	{
		if(_check_retired_lfb) lfb.state = LFB::State::RETIRED;
		else                  lfb.state = LFB::State::INVALID;
		bank.lfb_return_queue.pop();
	}
}

void UnitNonBlockingCache::clock_rise()
{
	_request_cross_bar.clock();

	for(uint i = 0; i < _banks.size(); ++i)
	{
		_clock_data_array(i);

		//if we select a return it will access the data array and lfb so we can't accept a request on this cycle
		if(!_proccess_return(i)) 
		{
			_proccess_request(i);
		}
	}
}

void UnitNonBlockingCache::clock_fall()
{
	for(uint i = 0; i < _banks.size(); ++i)
	{
		_try_return_lfb(i);
		_try_request_lfb(i);
	}

	_return_cross_bar.clock();
}

bool UnitNonBlockingCache::request_port_write_valid(uint port_index)
{
	return _request_cross_bar.is_write_valid(port_index);
}

void UnitNonBlockingCache::write_request(const MemoryRequest& request, uint port_index)
{
	_request_cross_bar.write(request, port_index);
}

bool UnitNonBlockingCache::return_port_read_valid(uint port_index)
{
	return _return_cross_bar.is_read_valid(port_index);
}

const MemoryReturn& UnitNonBlockingCache::peek_return(uint port_index)
{
	return _return_cross_bar.peek(port_index);
}

const MemoryReturn UnitNonBlockingCache::read_return(uint port_index)
{
	return _return_cross_bar.read(port_index);
}

}}