#include <vector>

#include "ramulator2/src/base/base.h"
#include "ramulator2/src/dram/dram.h"
#include "ramulator2/src/addr_mapper/addr_mapper.h"
<<<<<<< HEAD
#include "ramulator2/src/memory_system/memory_system.h"
=======
//#include "ramulator2/src/memory_system/memory_system.h"
>>>>>>> 4ba1e3b (integration of ramulator and drampower)
#include "ramulator2/src/addr_mapper/impl/linear_mappers.cpp"

namespace Ramulator {

class RoRaBaChCo final : public LinearMapperBase, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoRaBaChCo, "RoRaBaChCo", "Applies a RoRaBaChCo mapping to the address.");

public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
        req.addr_vec.resize(m_num_levels, -1);
        Addr_t addr = req.addr >> m_tx_offset;
        req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
        req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
        req.addr_vec[2] = slice_lower_bits(addr, m_addr_bits[2]);
        req.addr_vec[1] = slice_lower_bits(addr, m_addr_bits[1]);
        req.addr_vec[3] = slice_lower_bits(addr, m_addr_bits[3]);
    }
};

class RoBgBaRaChCo final : public LinearMapperBase, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoBgBaRaChCo, "RoBgBaRaChCo", "Applies a RoBgBaRaChCo mapping to the address.");

public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
        req.addr_vec.resize(m_num_levels, -1);
        Addr_t addr = req.addr >> m_tx_offset;
        req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
        req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
        req.addr_vec[1] = slice_lower_bits(addr, m_addr_bits[1]);
        req.addr_vec[3] = slice_lower_bits(addr, m_addr_bits[3]);
        req.addr_vec[2] = slice_lower_bits(addr, m_addr_bits[2]);
        req.addr_vec[4] = slice_lower_bits(addr, m_addr_bits[4]);
    }
};

}   // namespace Ramulator