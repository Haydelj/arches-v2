#pragma once

#include "int.hpp"

namespace rtm {
    
template<uint BITS>
struct BitArray
{
    uint8_t data[(BITS + 7) / 8];
    
    uint64_t read(uint index, uint size) const
    {
        uint64_t background_data = 0;
        uint start_byte = index >> 3;
        uint end_byte = (index + size + 7) >> 3;

        const uint8_t* cpy_ptr = data + start_byte;
        uint cpy_size = end_byte - start_byte;
        std::memcpy(&background_data, cpy_ptr, cpy_size);

        uint shft = index & 0x7;
        uint64_t mask = (0x1ull << size) - 1;
        return (background_data >> shft) & mask;
    }
    
    void write(uint index, uint size, uint64_t value)
    {
        uint64_t background_data = 0;
        uint start_byte = index >> 3;
        uint end_byte = (index + size + 7) >> 3;

        uint8_t* cpy_ptr = data + start_byte;
        uint cpy_size = end_byte - start_byte;
        std::memcpy(&background_data, cpy_ptr, cpy_size);
        
        uint shft = index & 0x7;
        uint64_t mask = (0x1ull << size) - 1;
        background_data &= ~(mask << shft);
        background_data |= value << shft;
        std::memcpy(cpy_ptr, &background_data, cpy_size);
    }
};

inline uint log2i(uint64_t u)
{
    if(u == 0) return 0;
    for(uint i = 0; i < 64; ++i)
        if((u >> i) == 0) return (i - 1);
    return 63;
}
    
}