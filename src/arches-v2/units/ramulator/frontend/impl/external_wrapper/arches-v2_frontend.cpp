#include <filesystem>
#include <iostream>
#include <fstream>

#include "units/ramulator/frontend/frontend.h"
#include "units/ramulator/base/exception.h"

namespace Ramulator {


class ARCHESv2 : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, ARCHESv2, "ARCHESv2", "ARCHESv2 frontend.")

  public:
    void init() override { };
    void tick() override { };

    bool receive_external_requests(int req_type_id, Addr_t addr, int source_id, std::function<void(Request&)> callback) override
    {
        return m_memory_system->send({addr, req_type_id, source_id, callback});
    }

  private:
    bool is_finished() override { return true; };
};

}        // namespace Ramulator