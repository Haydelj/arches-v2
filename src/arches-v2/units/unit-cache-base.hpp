#pragma once 
#include "stdafx.hpp"

#include "unit-main-memory-base.hpp"
#include "util/bit-manipulation.hpp"
#include "util/alignment-allocator.hpp"

namespace Arches { namespace Units {

class UnitCacheBase : public UnitMemoryBase
{
public:
	enum class Policy
	{
		LRU,
		RANDOM
	};

	UnitCacheBase(size_t size, uint block_size, uint associativity, uint sector_size = 0, Policy policy = Policy::LRU);
	virtual ~UnitCacheBase();

	void serialize(std::string file_path);
	bool deserialize(std::string file_path, const UnitMainMemoryBase& main_mem);
	void direct_write(paddr_t block_addr, uint8_t* data);

protected:
	struct BlockMetaData
	{
		uint64_t tag     : 48;
		uint64_t lru     : 8;
		uint64_t dirty   : 4;
		uint64_t valid   : 4;
	};

	struct Victim
	{
		paddr_t addr{0x0};
		uint8_t* data{nullptr};
		uint8_t dirty{0x0};
		uint8_t valid{0x0};
	};

	uint _hash;
	Policy _policy;
	uint _sets, _associativity, _block_size, _sector_size;
	paddr_t _block_offset_bits, _sector_offset_bits;

	std::vector<BlockMetaData, AlignmentAllocator<BlockMetaData, 64>> _tag_array;
	std::vector<uint8_t, AlignmentAllocator<uint8_t, 64>> _data_array;

	uint8_t* _read_sector(paddr_t sector_addr);
	uint8_t* _write_sector(paddr_t sector_addr, const uint8_t* data, bool set_dirty = false);
	Victim _allocate_block(paddr_t block_addr);

	paddr_t _get_sector_index(paddr_t paddr) { return _get_block_offset(paddr) / _sector_size; }
	paddr_t _get_sector_offset(paddr_t paddr) { return paddr & _sector_offset_bits; }
	paddr_t _get_sector_addr(paddr_t paddr) { return paddr & ~_sector_offset_bits; }

	paddr_t _get_block_offset(paddr_t paddr) { return paddr & _block_offset_bits; }
	paddr_t _get_block_addr(paddr_t paddr) { return paddr & ~_block_offset_bits; }
	paddr_t _get_block_addr(uint64_t tag, uint64_t set_index) { return tag * _block_size * _sets + set_index * _block_size; }

	paddr_t _get_set_index(paddr_t paddr) { return paddr / _block_size % _sets; }
	paddr_t _get_tag(paddr_t paddr) { return paddr / _block_size / _sets; }
};

}}