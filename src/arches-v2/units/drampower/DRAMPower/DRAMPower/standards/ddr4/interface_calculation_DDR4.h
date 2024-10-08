#ifndef DRAMPOWER_STANDARDS_DDR4_INTERFACE_CALCULATION_DDR4_H
#define DRAMPOWER_STANDARDS_DDR4_INTERFACE_CALCULATION_DDR4_H


#include "units/drampower/DRAMPower/DRAMPower/Types.h"
#include "units/drampower/DRAMPower/DRAMPower/data/energy.h"
#include "units/drampower/DRAMPower/DRAMPower/memspec/MemSpecDDR4.h"
#include "units/drampower/DRAMPower/DRAMPower/data/stats.h"

namespace DRAMPower {

class InterfaceCalculation_DDR4 {
   public:
    InterfaceCalculation_DDR4(const MemSpecDDR4 &memspec);

    interface_energy_info_t calculateEnergy(const SimulationStats &stats);

   private:
    const MemSpecDDR4 &memspec_;
    const MemSpecDDR4::MemImpedanceSpec &impedances_;
    double t_CK_;
    double VDD_;

    interface_energy_info_t calcClockEnergy(const SimulationStats &stats);
    interface_energy_info_t calcDQSEnergy(const SimulationStats &stats);
    interface_energy_info_t calcDQEnergy(const SimulationStats &stats);
    interface_energy_info_t calcCAEnergy(const SimulationStats &stats);
};

}  // namespace DRAMPower

#endif /* DRAMPOWER_STANDARDS_DDR4_INTERFACE_CALCULATION_DDR4_H */
