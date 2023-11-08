#include "unit-cache-base.hpp"

namespace Arches {namespace Units {

UnitCacheBase::UnitCacheBase(size_t size, uint associativity) : UnitMemoryBase()
{
	_tag_array.resize(size / CACHE_BLOCK_SIZE);
	_data_array.resize(size / CACHE_BLOCK_SIZE);

	_associativity = associativity;

	uint num_sets = size / (CACHE_BLOCK_SIZE * associativity);

	uint offset_bits = log2i(CACHE_BLOCK_SIZE);
	uint set_index_bits = log2i(num_sets);
	uint tag_bits = static_cast<uint>(sizeof(paddr_t) * 8) - (set_index_bits + offset_bits);

	_block_offset_mask = generate_nbit_mask(offset_bits);
	_set_index_mask = generate_nbit_mask(set_index_bits);
	_tag_mask = generate_nbit_mask(tag_bits);

	_set_index_offset = offset_bits;
	_tag_offset = offset_bits + set_index_bits;
}

UnitCacheBase::~UnitCacheBase()
{

}

//update lru and returns data pointer to cache line
UnitCacheBase::BlockData* UnitCacheBase::_get_block(paddr_t paddr)
{
	uint start = _get_set_index(paddr) * _associativity;
	uint end = start + _associativity;

	uint64_t tag = _get_tag(paddr);

	uint found_index = ~0;
	uint found_lru = 0;
	for(uint i = start; i < end; ++i)
	{
		if(_tag_array[i].valid && _tag_array[i].tag == tag)
		{
			found_index = i;
			found_lru = _tag_array[i].lru;
			break;
		}
	}

	if(found_index == ~0) return nullptr; //didn't find line so we will leave lru alone and return nullptr

	for(uint i = start; i < end; ++i)
		if(_tag_array[i].lru < found_lru) _tag_array[i].lru++;

	_tag_array[found_index].lru = 0;

	return &_data_array[found_index];
}

//inserts cacheline associated with paddr replacing least recently used. Assumes cachline isn't already in cache if it is this has undefined behaviour
UnitCacheBase::BlockData* UnitCacheBase::_insert_block(paddr_t paddr, const uint8_t* data)
{
	uint start = _get_set_index(paddr) * _associativity;
	uint end = start + _associativity;

	uint replacement_index = ~0u;
	uint replacement_lru = 0u;
	for(uint i = start; i < end; ++i)
	{
		if(!_tag_array[i].valid)
		{
			replacement_index = i;
			break;
		}
		
		if(_tag_array[i].lru >= replacement_lru)
		{
			replacement_lru = _tag_array[i].lru;
			replacement_index = i;
		}
	}

	for(uint i = start; i < end; ++i)
		_tag_array[i].lru++;

	_tag_array[replacement_index].lru = 0;
	_tag_array[replacement_index].valid = true;
	_tag_array[replacement_index].tag = _get_tag(paddr);

	std::memcpy(_data_array[replacement_index].bytes, data, CACHE_BLOCK_SIZE);
	return &_data_array[replacement_index];
}

}}