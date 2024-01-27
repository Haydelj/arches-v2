#pragma once 
#include "stdafx.hpp"
#include "bit-manipulation.hpp"

//Uses a 64bit integer and BM2 extension to implement a computational and stoarge efficent arbiter for up to 64 clients
class RoundRobinArbiter
{
protected:
	uint64_t _pending{0x0ull};
	uint32_t _priority_index{0};
	uint32_t _size{64};

public:
	RoundRobinArbiter(uint size = 64) : _size(size) { assert(size <= 64); }

	uint32_t size() { return _size; }

	uint64_t get_mask()
	{
		return _pending;
	}

	uint num_pending()
	{
		return popcnt(_pending);
	}

	void add(uint index)
	{
		_pending |= 0x1ull << index;
	}

	void remove(uint index)
	{
		_pending &= ~(0x1ull << index);

		//Advance the priority index on remove so the grant index is now lowest priority
		if(index == _priority_index)
			if(++_priority_index >= _size)
				_priority_index = 0;
	}

	uint get_index()
	{
		if(!_pending) 
			return ~0u;

		uint64_t rot_mask = rotr(_pending, _priority_index); //rotate the mask so the last index flag is in the 0 bit
		uint64_t offset = ctz(rot_mask); //count the number of 0s till the next 1
		uint64_t grant_index = (_priority_index + offset) & 0x3full; //grant the next set bit
		_priority_index = grant_index; //make the grant bit the highest priority bit so that it will continue to be granted until removed
		return grant_index; 
	}
};

#if 0
template<uint MAX_SIZE>
class RoundRobinArbiter2
{
protected:
	uint64_t _pending[MAX_SIZE];
	uint32_t _priority_index{0};
	uint32_t _size{64};

public:
	RoundRobinArbiter(uint size = 64) : _size(size) { assert(size <= MAX_SIZE * 64); }

	uint32_t size() { return _size; }

	uint num_pending()
	{
		return popcnt(_pending);
	}

	void add(uint index)
	{
		_pending[index >> 6] |= 0x1ull << (index & 0x3f);
	}

	void remove(uint index)
	{
		_pending[index >> 6] &= ~(0x1ull << (index & 0x3f));

		//Advance the priority index on remove so the grant index is now lowest priority
		if(index == _priority_index)
			if(++_priority_index >= _size)
				_priority_index = 0;
	}

	uint get_index()
	{
		uint current_set_index = _priority_index >> 6;
		while(!_pending[current_set_index])
		{
			if(++current_set_index < MAX_SIZE) 
				current_set_index = 0;

			if(current_set_index == (priority_index >> 6)) 
				return ~0u;
		}


		bool pending = false;
		for(uint i = 0; i < MAX_SIZE; ++i)
			if(_pending[i])
				pending = true;

		if(!pending)
			return ~0u;



		uint64_t rot_mask = rotr(_pending, _priority_index); //rotate the mask so the last index flag is in the 0 bit
		uint64_t offset = ctz(rot_mask); //count the number of 0s till the next 1
		uint64_t grant_index = (_priority_index + offset) & 0x3full; //grant the next set bit
		_priority_index = grant_index; //make the grant bit the highest priority bit so that it will continue to be granted until removed
		return grant_index;
	}
};
#endif