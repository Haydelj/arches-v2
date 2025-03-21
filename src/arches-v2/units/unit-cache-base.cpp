#include "unit-cache-base.hpp"

namespace Arches {
namespace Units {

UnitCacheBase::UnitCacheBase(size_t size, uint block_size, uint associativity, uint sector_size) : _tag_array(size / block_size), _data_array(size), _sector_size(sector_size), UnitMemoryBase()
{
	if(_sector_size == 0) _sector_size = block_size;

	//initialize tag array
	for(uint i = 0; i < _tag_array.size(); ++i)
	{
		_tag_array[i].valid = 0;
		_tag_array[i].dirty = 0;
		_tag_array[i].lru = i % associativity;
		_tag_array[i].tag = ~0x0ull;
	}

	_block_size = block_size;
	_associativity = associativity;

	uint num_sets = size / (block_size * associativity);

	uint offset_bits = log2i(block_size);
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

void UnitCacheBase::serialize(std::string file_path)
{
	std::ofstream file_stream(file_path, std::ios::binary);
	file_stream.write((char*)_tag_array.data(), sizeof(BlockMetaData) * _tag_array.size());
	//file_stream.write((char*)_data_array.data(), _data_array.size());

	printf("Write cache success: %s\n", file_path.c_str());

}

bool UnitCacheBase::deserialize(std::string file_path, const UnitMainMemoryBase& main_mem)
{
	printf("Loading cache: %s\n", file_path.c_str());

	bool succeeded = false;
	std::ifstream file_stream(file_path, std::ios::binary);
	if(file_stream.good())
	{
		file_stream.read((char*)_tag_array.data(), sizeof(BlockMetaData) * _tag_array.size());
		//file_stream.read((char*)_data_array.data(), _data_array.size());

		for(uint i = 0; i < _tag_array.size(); ++i)
		{
			if(!_tag_array[i].valid) continue;
			uint64_t tag = _tag_array[i].tag;
			uint64_t set_index = i / _associativity;
			paddr_t block_addr = _get_block_addr(tag, set_index);
			main_mem.direct_read(_data_array.data() + i * _block_size, _block_size, block_addr);
		}

		succeeded = true;

		printf("Loaded cache: %s\n", file_path.c_str());
	}
	else
	{
		printf("Failed to load cache: : Failed to open file\n");
	}

	return succeeded;
}


uint8_t* UnitCacheBase::_read_block(paddr_t block_addr)
{ 
	uint8_t vm;
	return _read_block(block_addr, vm);
}

//update lru and returns data pointer to cache line
uint8_t* UnitCacheBase::_read_block(paddr_t block_addr, uint8_t& valid_mask)
{
	uint64_t tag = _get_tag(block_addr);
	uint set_index = _get_set_index(block_addr);
	uint start = set_index  * _associativity;
	uint end = start + _associativity;

	uint found_index = ~0;
	uint found_lru = 0;
	for(uint i = start; i < end; ++i)
	{
		if(_tag_array[i].tag == tag)
		{
			found_index = i;
			found_lru = _tag_array[i].lru;
			break;
		}
	}

	valid_mask = 0;
	if(found_index == ~0) 
		return nullptr; //Didn't find line so we will leave lru alone and return nullptr

	for(uint i = start; i < end; ++i)
		if(_tag_array[i].lru < found_lru) _tag_array[i].lru++;

	_tag_array[found_index].lru = 0;
	if(!_tag_array[found_index].valid) //Found sector but it was invalid
		return nullptr;

	valid_mask = _tag_array[found_index].valid;
	return &_data_array[found_index * _block_size];
}

//writes data to block and updates valid bit
uint8_t* UnitCacheBase::_write_sector(paddr_t sector_addr, const uint8_t* data, bool set_dirty)
{
	_assert(data);
	uint64_t tag = _get_tag(sector_addr);
	uint set_index = _get_set_index(sector_addr);
	uint sector_index = _get_sector_index(sector_addr);
	uint start = set_index * _associativity;
	uint end = start + _associativity;

	for(uint i = start; i < end; ++i)
	{
		if(_tag_array[i].tag == tag)
		{
			_tag_array[i].valid |= 0x1ull << sector_index;
			if(set_dirty) _tag_array[i].dirty |= 0x1ull << sector_index;
			std::memcpy(_data_array.data() + i * _block_size + sector_index * _sector_size, data, _sector_size);
			return &_data_array[i * _block_size + sector_index * _sector_size];
		}
	}

	return nullptr;
}

//inserts cacheline associated with paddr replacing least recently used. Assumes cachline isn't already in cache if it is this has undefined behaviour
UnitCacheBase::Victim UnitCacheBase::_insert_block(paddr_t block_addr)
{
	uint64_t tag = _get_tag(block_addr);
	uint set_index = _get_set_index(block_addr);
	uint start = set_index * _associativity;
	uint end = start + _associativity;

	//find replacement index
	uint replacement_index = ~0u;
	uint replacement_lru = 0u;
	for(uint i = start; i < end; ++i)
	{
		if(_tag_array[i].lru >= replacement_lru)
		{
			replacement_lru = _tag_array[i].lru;
			replacement_index = i;
		}
	}

	//compute victim block
	Victim victim;
	victim.addr = _get_block_addr(_tag_array[replacement_index].tag, set_index);
	victim.data = _data_array.data() + replacement_index * _block_size;
	victim.dirty = _tag_array[replacement_index].dirty;
	victim.valid = _tag_array[replacement_index].valid;

	//update lru
	for(uint i = start; i < end; ++i)
		if(_tag_array[i].lru < replacement_lru) _tag_array[i].lru++;

	//set block metadata
	_tag_array[replacement_index].lru = 0;
	_tag_array[replacement_index].tag = tag;
	_tag_array[replacement_index].valid = 0;
	_tag_array[replacement_index].dirty = 0;

	return victim;
}

}}