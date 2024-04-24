#ifndef RAMULATOR_CONTROLLER_CONTROLLER_H
#define RAMULATOR_CONTROLLER_CONTROLLER_H

#include <vector>
#include <deque>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "units/ramulator/base/base.h"
#include "units/ramulator/dram/dram.h"
#include "units/ramulator/dram_controller/scheduler.h"
#include "units/ramulator/dram_controller/plugin.h"
#include "units/ramulator/dram_controller/refresh.h"

#include <fstream>
#include <iostream>


namespace Ramulator {

class IDRAMController : public Clocked<IDRAMController> {
  RAMULATOR_REGISTER_INTERFACE(IDRAMController, "Controller", "Memory Controller Interface");

  public:
    IDRAM*  m_dram = nullptr;          
    IScheduler*   m_scheduler = nullptr;
    IRefreshManager*   m_refresh = nullptr;
    std::fstream* cmd_trace_file = nullptr;
    size_t s_num_read_row_hits = 0;
    size_t s_num_read_row_misses = 0;
    size_t s_num_read_row_conflicts = 0;
    size_t s_num_write_row_hits = 0;
    size_t s_num_write_row_misses = 0;
    size_t s_num_write_row_conflicts = 0;
    size_t s_num_write_merge = 0;
    size_t s_sum_read_latency = 0;
    size_t s_sum_read_queue_latency = 0;
    size_t s_sum_write_latency = 0;

    int m_channel_id = -1;
  public:
    /**
     * @brief       Send a request to the memory controller.
     * 
     * @param    req        The request to be enqueued.
     * @return   true       Successful.
     * @return   false      Failed (e.g., buffer full).
     */
    virtual bool send(Request& req) = 0;

    /**
     * @brief       Send a high-priority request to the memory controller.
     * 
     */
    virtual bool priority_send(Request& req) = 0;

    /**
     * @brief       Ticks the memory controller.
     * 
     */
    virtual void tick() = 0;

    virtual bool is_busy() = 0;
   
};

}       // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_CONTROLLER_H