#include "units/ramulator/memory_system/memory_system.h"
#include "units/ramulator/translation/translation.h"
#include "units/ramulator/dram_controller/controller.h"
#include "units/ramulator/addr_mapper/addr_mapper.h"
#include "units/ramulator/dram/dram.h"
#include "units/drampower/DRAMPower/DRAMPower/command/Command.h"
#include "units/drampower/DRAMPower/DRAMPower/standards/ddr5/DDR5.h"
#include "units/drampower/DRAMPower/DRAMPower/standards/ddr5/core_calculation_DDR5.h"
#include "units/drampower/DRAMPower/DRAMPower/standards/ddr5/interface_calculation_DDR5.h"

using namespace DRAMPower;

namespace Ramulator {

class GenericDRAMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IMemorySystem, GenericDRAMSystem, "GenericDRAM", "A generic DRAM-based memory system.");

  protected:
    Clk_t m_clk = 0;
    IDRAM*  m_dram;
    IAddrMapper*  m_addr_mapper;
    std::vector<IDRAMController*> m_controllers;
    std::vector<std::fstream> cmd_trace_files;

  public:
    int s_num_read_requests = 0;
    int s_num_write_requests = 0;
    int s_num_other_requests = 0;
    std::vector<double> dram_power;
    double total_dram_power = 0;
    std::vector<int> num_read_requests;
    std::vector<int> num_write_requests;

  public:
    void init() override { 
      // Create device (a top-level node wrapping all channel nodes)
      m_dram = create_child_ifce<IDRAM>();
      m_addr_mapper = create_child_ifce<IAddrMapper>();

      int num_channels = m_dram->get_level_size("channel");   
      cmd_trace_files.resize(num_channels);
      dram_power.resize(num_channels, 0);
      num_read_requests.resize(num_channels, 0);
      num_write_requests.resize(num_channels, 0);

      // Create memory controllers
      for (int i = 0; i < num_channels; i++) {
        IDRAMController* controller = create_child_ifce<IDRAMController>();
        controller->m_impl->set_id(fmt::format("Channel {}", i));
        controller->m_channel_id = i;
        m_controllers.push_back(controller);
        //cmd_trace_files[i].open("cmd-trace-channel-" + std::to_string(i) + ".cmdtrace", std::ios::out | std::ios::in | std::ios::trunc);
        //if (!cmd_trace_files[i].is_open())
        //{
        //    std::cerr << "Unable to open file: cmd-trace-channel-" << std::to_string(i) << ".cmdtrace" << std::endl;
        //    exit(1);
        //}
        //controller->cmd_trace_file = &cmd_trace_files[i];
      }

      m_clock_ratio = param<uint>("clock_ratio").required();

      register_stat(m_clk).name("memory_system_cycles");
      register_stat(s_num_read_requests).name("total_num_read_requests");
      register_stat(s_num_write_requests).name("total_num_write_requests");
      register_stat(s_num_other_requests).name("total_num_other_requests");
      register_stat(dram_power).name("dram_power_per_channel(W)");
      register_stat(total_dram_power).name("total_dram_power(W)");
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { }

    bool send(Request req) override {
      m_addr_mapper->apply(req);
      int channel_id = req.addr_vec[0];
      bool is_success = m_controllers[channel_id]->send(req);

      if (is_success) {
        switch (req.type_id) {
          case Request::Type::Read: {
            s_num_read_requests++;
            num_read_requests[channel_id]++;
            break;
          }
          case Request::Type::Write: {
            s_num_write_requests++;
            num_write_requests[channel_id]++;
            break;
          }
          default: {
            s_num_other_requests++;
            break;
          }
        }
      }

      return is_success;
    };
    
    void tick() override {
      m_clk++;
      m_dram->tick();
      for (auto controller : m_controllers) {
        controller->tick();
      }
    };

    float get_tCK() override {
      return m_dram->m_timing_vals("tCK_ps") / 1000.0f;
    }

    bool ramu_is_busy() override
    {
        for (auto controller : m_controllers)
        {
            if (controller->is_busy())
                return true;
        }
        return false;
    }

    energy_info_t calculate_power(std::fstream* trace_file)
    {
        std::vector<Command> commandList;
        std::string line;
        while (std::getline((*trace_file), line))
        {
            std::vector<std::string> tokens;
            std::istringstream strStream(line);
            std::string token;
            while (std::getline(strStream, token, ','))
            {
                tokens.push_back(token);
            }
            assert(tokens.size() == 5);
            timestamp_t timestamp = std::stoull(tokens[0]);
            auto cmdType = tokens[1];
            auto bank_id = std::stoull(tokens[2]);
            auto bank_group_id = std::stoull(tokens[3]);
            auto rank_id = std::stoull(tokens[4]);
            if (cmdType == "ACT") {
                commandList.push_back({ timestamp, CmdType::ACT, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PRE") {
                commandList.push_back({ timestamp, CmdType::PRE, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PREA") {
                commandList.push_back({ timestamp, CmdType::PREA, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PRESB") {
                commandList.push_back({ timestamp, CmdType::PRESB, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "REFA" || cmdType == "REF") {
                commandList.push_back({ timestamp, CmdType::REFA, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "REFB") {
                commandList.push_back({ timestamp, CmdType::REFB, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "REFSB") {
                commandList.push_back({ timestamp, CmdType::REFSB, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "RD") {
                commandList.push_back({ timestamp, CmdType::RD, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "RDA") {
                commandList.push_back({ timestamp, CmdType::RDA, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "WR") {
                commandList.push_back({ timestamp, CmdType::WR, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "WRA") {
                commandList.push_back({ timestamp, CmdType::WRA, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PDEA") {
                commandList.push_back({ timestamp, CmdType::PDEA, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PDEP") {
                commandList.push_back({ timestamp, CmdType::PDEP, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PDXA") {
                commandList.push_back({ timestamp, CmdType::PDXA, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "PDXP") {
                commandList.push_back({ timestamp, CmdType::PDXP, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "SREN") {
                commandList.push_back({ timestamp, CmdType::SREFEN, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "SREX") {
                commandList.push_back({ timestamp, CmdType::SREFEX, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "DSMEN") {
                commandList.push_back({ timestamp, CmdType::DSMEN, { bank_id, bank_group_id, rank_id } });
            }
            else if (cmdType == "DSMEX") {
                commandList.push_back({ timestamp, CmdType::DSMEX, { bank_id, bank_group_id, rank_id } });
            }
        }
        commandList.push_back({ static_cast<unsigned long long>(m_clk), CmdType::END_OF_SIMULATION });
        std::ifstream ifs("./config-files/gddr6.json");
        json data = json::parse(ifs);
        DDR5 dram(data["memspec"]);
        for (const auto& command : commandList) {
            dram.doCommand(command);
        };
        auto stats = dram.getStats();
        auto energy = dram.calcEnergy(commandList.back().timestamp);
        auto total_energy = energy.total_energy();
        return total_energy;
    }

    void finalize() override
    {
        /*for (auto component : m_components) {
            component->finalize();
        }*/
        //std::cout << "----------------- DRAM stats -----------------" << std::endl;
        //for (int i = 0; i < cmd_trace_files.size(); ++i)
        //{
        //    cmd_trace_files[i].seekg(0);
        //    auto total_energy = calculate_power(&cmd_trace_files[i]);
        //    cmd_trace_files[i].close();
        //    //std::cout << "Total energy of Channel " << i << ": " << total_energy.total() * 1e-3/ (m_clk * get_tCK()) << " W, total = " << total_energy.total() << ", m_clk = " << m_clk << ", tCK = " << get_tCK() << std::endl;
        //    dram_power[i] = total_energy.total() * 1e-3 / (m_clk * get_tCK());
        //    total_dram_power += dram_power[i];
        //    std::cout << "------------ Channel " << i << " ------------" << std::endl;
        //    std::cout << "Total reads serviced:         " << num_read_requests[i] << std::endl;
        //    std::cout << "Total writes serviced:        " << num_write_requests[i] << std::endl;
        //    std::cout << "Total writes merged:          " << m_controllers[i]->s_num_write_merge << std::endl;
        //    std::cout << "Read row hit rate:            " << m_controllers[i]->s_num_read_row_hits / float(num_read_requests[i]) << std::endl;
        //    std::cout << "Read row miss rate:           " << m_controllers[i]->s_num_read_row_misses / float(num_read_requests[i]) << std::endl;
        //    std::cout << "Read row conflict rate:       " << m_controllers[i]->s_num_read_row_conflicts / float(num_read_requests[i]) << std::endl;
        //    std::cout << "Write row hit rate:           " << m_controllers[i]->s_num_write_row_hits / float(num_write_requests[i] - m_controllers[i]->s_num_write_merge) << std::endl;
        //    std::cout << "Write row miss rate:          " << m_controllers[i]->s_num_write_row_misses / float(num_write_requests[i] - m_controllers[i]->s_num_write_merge) << std::endl;
        //    std::cout << "Write row conflict rate:      " << m_controllers[i]->s_num_write_row_conflicts / float(num_write_requests[i] - m_controllers[i]->s_num_write_merge) << std::endl;
        //    std::cout << "Average read latency:         " << m_controllers[i]->s_sum_read_latency / float(num_read_requests[i]) << std::endl;
        //    std::cout << "Average read queue latency:   " << m_controllers[i]->s_sum_read_queue_latency / float(num_read_requests[i]) << std::endl;
        //    std::cout << "Average write latency:        " << m_controllers[i]->s_sum_write_latency / float(num_write_requests[i] - m_controllers[i]->s_num_write_merge) << std::endl;
        //    std::cout << "Dram power:                   " << dram_power[i] << "W" << std::endl;
        //    std::cout << std::endl;
        //}
        //std::cout << "Total Dram power:             " << total_dram_power << "W" << std::endl;

        /*YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        m_impl->print_stats(emitter);
        emitter << YAML::EndMap;
        std::cout << emitter.c_str() << std::endl;*/
    };

    // const SpecDef& get_supported_requests() override {
    //   return m_dram->m_requests;
    // };
};
  
}   // namespace 

