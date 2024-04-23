#include "unit-non-blocking-cache.hpp"

namespace Arches {namespace Units {

UnitNonBlockingCache::UnitNonBlockingCache(Configuration config) : 
	UnitCacheBase(config.size, config.block_size, config.associativity),
	_request_cross_bar(config.num_ports, config.num_banks, config.bank_select_mask),
	_return_cross_bar(config.num_ports, config.num_banks)
{
	_use_lfb = config.use_lfb;

	_mem_highers = config.mem_highers;
	_mem_higher_port_offset = config.mem_higher_port_offset;
	_mem_higher_port_stride = config.mem_higher_port_stride;

	_banks.resize(config.num_banks, {config.num_mshr, config.latency});
}

UnitNonBlockingCache::~UnitNonBlockingCache()
{

}

uint UnitNonBlockingCache::_fetch_lfb(uint bank_index, MSHR& mshr)
{
	Bank& bank = _banks[bank_index];
	std::vector<MSHR>& mshrs = bank.mshrs;

	for(uint32_t i = 0; i < mshrs.size(); ++i)
		if(mshrs[i].state != MSHR::State::INVALID && mshrs[i] == mshr) return i;

	return ~0u;
}

uint UnitNonBlockingCache::_allocate_lfb(uint bank_index, MSHR& mshr)
{
	std::vector<MSHR>& mshrs = _banks[bank_index].mshrs;

	uint replacement_index = ~0u;
	uint replacement_lru = 0u;
	for(uint i = 0; i < mshrs.size(); ++i)
	{
		if(mshrs[i].state == MSHR::State::INVALID)
		{
			replacement_index = i;
			break;
		}

		if(mshrs[i].state == MSHR::State::RETIRED && mshrs[i].lru >= replacement_lru)
		{
			replacement_lru = mshrs[i].lru;
			replacement_index = i;
		}
	}

	//can't allocate
	if(replacement_index == ~0) return ~0;

	for(uint i = 0; i < mshrs.size(); ++i) mshrs[i].lru++;
	mshrs[replacement_index].lru = 0;
	mshrs[replacement_index] = mshr;
	return replacement_index;
}

uint UnitNonBlockingCache::_fetch_or_allocate_mshr(uint bank_index, uint64_t block_addr, MSHR::Type type)
{
	std::vector<MSHR>& mshrs = _banks[bank_index].mshrs;

	MSHR mshr;
	mshr.block_addr = block_addr;
	mshr.type = type;
	mshr.state = MSHR::State::EMPTY;
	
	uint mshr_index = _fetch_lfb(bank_index, mshr);
	if(mshr_index != ~0u) return mshr_index;
	return _allocate_lfb(bank_index, mshr);
}

void UnitNonBlockingCache::_push_request(MSHR& mshr, const MemoryRequest& request)
{
	//prefetch requests don't produce a sub entry
	if(request.type != MemoryRequest::Type::PREFECTH)
	{
		MSHR::SubEntry sub_entry;
		sub_entry.offset = _get_block_offset(request.paddr);
		sub_entry.size = request.size;
		sub_entry.port = request.port;
		sub_entry.dst = request.dst;
		mshr.sub_entries.push(sub_entry);
	}
}

MemoryRequest UnitNonBlockingCache::_pop_request(MSHR& mshr)
{
	MSHR::SubEntry sub_entry = mshr.sub_entries.front();
	mshr.sub_entries.pop();

	MemoryRequest req;
	req.type = MemoryRequest::Type::LOAD;
	req.size = sub_entry.size;
	req.port = sub_entry.port;
	req.dst = sub_entry.dst;
	req.paddr = mshr.block_addr + sub_entry.offset;
	return req;
}

void UnitNonBlockingCache::_clock_data_array(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	bank.data_array_pipline.clock();
	if(!bank.data_array_pipline.is_read_valid()) return;

	uint mshr_index = bank.data_array_pipline.read();
	if(mshr_index == ~0u) return; 

	MSHR& mshr = bank.mshrs[mshr_index];
	_assert(mshr.state == MSHR::State::DATA_ARRAY);
	mshr.state = MSHR::State::FILLED;
	bank.mshr_return_queue.push(mshr_index);
}

bool UnitNonBlockingCache::_proccess_return(uint bank_index)
{
	uint mem_higher_port_index = bank_index * _mem_higher_port_stride + _mem_higher_port_offset;
	for(UnitMemoryBase* mem_higher : _mem_highers)
	{
		if(mem_higher->return_port_read_valid(mem_higher_port_index))
		{
			MemoryReturn ret = mem_higher->read_return(mem_higher_port_index);
			_assert(ret.paddr == _get_block_addr(ret.paddr));

			//Mark the associated lse as filled and put it in the return queue
			Bank& bank = _banks[bank_index];
			for(uint i = 0; i < bank.mshrs.size(); ++i)
			{
				MSHR& mshr = bank.mshrs[i];
				if(mshr.block_addr == ret.paddr)
				{
					_assert(mshr.state == MSHR::State::MISSED);
					std::memcpy(mshr.block_data, ret.data, _block_size);
					mshr.state = MSHR::State::FILLED;
					bank.mshr_return_queue.push(i);
					break;
				}
			}

			//Insert block
			log.tag_array_access++;
			log.data_array_writes++;
			_insert_block(ret.paddr, ret.data);

			if(bank.data_array_pipline.lantecy() != 0)
				bank.data_array_pipline.write(~0u);
	
			return true;
		}
	}
	return false;
}

bool UnitNonBlockingCache::_proccess_request(uint bank_index)
{
	if(!_request_cross_bar.is_read_valid(bank_index)) return false;

	Bank& bank = _banks[bank_index];
	const MemoryRequest& request = _request_cross_bar.peek(bank_index);
	paddr_t block_addr = _get_block_addr(request.paddr);
	paddr_t block_offset = _get_block_offset(request.paddr);

	if(request.type == MemoryRequest::Type::LOAD || request.type == MemoryRequest::Type::PREFECTH)
	{
		//Try to fetch an mshr for the line or allocate a new mshr for the line
		uint mshr_index = _fetch_or_allocate_mshr(bank_index, block_addr, MSHR::Type::READ);

		//In parallel access the tag array to check for the line
		uint8_t* block_data = _get_block(block_addr);
		log.tag_array_access++;

		//If the data array access is zero cycle then that means we did it in parallel with th tag lookup
		if(bank.data_array_pipline.lantecy() == 0)
		{
			log.data_array_reads++;
		}

		if(mshr_index != ~0)
		{
			MSHR& mshr = bank.mshrs[mshr_index];
			_push_request(mshr, request);
			log.profile_counters[block_addr]++;

			if(mshr.state == MSHR::State::EMPTY)
			{
				if(block_data)
				{
					std::memcpy(mshr.block_data, block_data, _block_size);

					//Copy line from data array to LFB
					if(bank.data_array_pipline.lantecy() == 0)
					{
						mshr.state = MSHR::State::FILLED;
						bank.mshr_return_queue.push(mshr_index);
					}
					else
					{
						mshr.state = MSHR::State::DATA_ARRAY;
						bank.data_array_pipline.write(mshr_index);
						log.data_array_reads++;
					}

					log.hits++;
				}
				else
				{
					//Missed the cache queue up a request to mem higher
					mshr.state = MSHR::State::MISSED;
					bank.mshr_request_queue.push(mshr_index);
					log.misses++;
				}
			}
			else if(mshr.state == MSHR::State::MISSED)
			{
				log.misses++;
				log.half_misses++;
			}
			else if(mshr.state == MSHR::State::FILLED)
			{
				log.hits++;
				log.lfb_hits++;
			}
			else if(mshr.state == MSHR::State::RETIRED)
			{
				//Wake up retired LFB and add it to the return queue
				mshr.state = MSHR::State::FILLED;
				bank.mshr_return_queue.push(mshr_index);
				log.hits++;
				log.lfb_hits++;
			}
			
			log.requests++;
			_request_cross_bar.read(bank_index);
		}
		else log.mshr_stalls++;
	}
	else if(request.type == MemoryRequest::Type::STORE)
	{
		if(_use_lfb)
		{
			//try to allocate an lfb in write combine mode
			uint lfb_index = _fetch_or_allocate_mshr(bank_index, block_addr, MSHR::Type::WRITE_COMBINING);
			if(lfb_index != ~0)
			{
				MSHR& lfb = bank.mshrs[lfb_index];
				lfb.write_mask |= generate_nbit_mask(request.size) << block_offset;

				if(lfb.state == MSHR::State::EMPTY)
				{
					lfb.state = MSHR::State::FILLED; //since we just filled it
					bank.mshr_request_queue.push(lfb_index);
				}

				log.requests++;
				_request_cross_bar.read(bank_index);
			}
		}
		else
		{
			bank.uncached_write_queue.push(request);
			log.requests++;
			_request_cross_bar.read(bank_index);
		}
	}
	else _assert(false);

	return true;
}

bool UnitNonBlockingCache::_try_request_lfb(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	if(bank.mshr_request_queue.empty()) return false;

	MSHR& mshr = bank.mshrs[bank.mshr_request_queue.front()];

	UnitMemoryBase* mem_higher = _get_mem_higher(mshr.block_addr);
	uint mem_higher_port_index = bank_index * _mem_higher_port_stride + _mem_higher_port_offset;

	if(!mem_higher->request_port_write_valid(mem_higher_port_index)) return false;

	if(mshr.type == MSHR::Type::READ)
	{
		_assert(mshr.state == MSHR::State::MISSED);

		MemoryRequest outgoing_request;
		outgoing_request.type = MemoryRequest::Type::LOAD;
		outgoing_request.size = _block_size;
		outgoing_request.port = mem_higher_port_index;
		outgoing_request.paddr = mshr.block_addr;
		mem_higher->write_request(outgoing_request);
		bank.mshr_request_queue.pop();
	}
	else if(mshr.type == MSHR::Type::WRITE)
	{
		_assert(false);
	}
	else if(mshr.type == MSHR::Type::WRITE_COMBINING)
	{
		_assert(mshr.state == MSHR::State::FILLED);

		//TODO fix this:
		MemoryRequest outgoing_request;
		outgoing_request.type = MemoryRequest::Type::STORE;
		outgoing_request.size = _block_size;
		outgoing_request.port = mem_higher_port_index;
		outgoing_request.paddr = mshr.block_addr;
		std::memcpy(outgoing_request.data, mshr.block_data, _block_size);
		mem_higher->write_request(outgoing_request);

		mshr.state = MSHR::State::INVALID;
		bank.mshr_request_queue.pop();
	}

	return true;
}

void UnitNonBlockingCache::_try_forward_writes(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	if(bank.uncached_write_queue.empty()) return;

	MemoryRequest& request = bank.uncached_write_queue.front();

	UnitMemoryBase* mem_higher = _get_mem_higher(request.paddr);
	uint mem_higher_port_index = bank_index * _mem_higher_port_stride + _mem_higher_port_offset;

	if(!mem_higher->request_port_write_valid(mem_higher_port_index)) return;

	request.port = mem_higher_port_index;
	mem_higher->write_request(request);
	bank.uncached_write_queue.pop();
}

void UnitNonBlockingCache::_try_return_lfb(uint bank_index)
{
	Bank& bank = _banks[bank_index];

	//last return isn't complete do nothing
	if(bank.mshr_return_queue.empty() || !_return_cross_bar.is_write_valid(bank_index)) return;

	MSHR& mshr = bank.mshrs[bank.mshr_return_queue.front()];

	//select the next subentry and copy return to interconnect
	MemoryRequest req = _pop_request(mshr);
	uint block_offset = _get_block_offset(req.paddr);
	MemoryReturn ret(req, mshr.block_data + block_offset);

	_return_cross_bar.write(ret, bank_index);
	log.bytes_read += ret.size;

	if(mshr.sub_entries.empty())
	{
		if(_use_lfb) mshr.state = MSHR::State::RETIRED;
		else          mshr.state = MSHR::State::INVALID;
		bank.mshr_return_queue.pop();
	}
}

void UnitNonBlockingCache::clock_rise()
{
	_request_cross_bar.clock();

	for(uint i = 0; i < _banks.size(); ++i)
	{
		_clock_data_array(i);

		//if we select a return it will access the data array and mshr so we can't accept a request on this cycle
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
		if(!_try_request_lfb(i))
		{
			_try_forward_writes(i);
		}
	}

	_return_cross_bar.clock();
}

bool UnitNonBlockingCache::request_port_write_valid(uint port_index)
{
	return _request_cross_bar.is_write_valid(port_index);
}

void UnitNonBlockingCache::write_request(const MemoryRequest& request)
{
	_request_cross_bar.write(request, request.port);
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