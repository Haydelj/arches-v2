#include "ramulator2/src/dram/dram.h"
#include "ramulator2/src/dram/lambdas.h"

namespace Ramulator {
//#define USE_DRAMPOWER

class GDDR6A : public IDRAM, public Implementation
{
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, GDDR6A, "GDDR6A", "GDDR6 Device Model for Arches")

  public:
    inline static const std::map<std::string, Organization> org_presets = { //Table 19 for more info
      //name                 density    DQ   Ch Ra Bg Ba Ro     Co
      {"GDDR6_8Gb_x8",       {8<<10,    8,  {2, 1, 4, 4, 1<<14, 1<<11}}}, // 2^30
      {"GDDR6_8Gb_x16",      {8<<10,    16, {2, 1, 4, 4, 1<<14, 1<<10}}}, 
      {"GDDR6_16Gb_x8",      {16<<10,   8,  {2, 1, 4, 4, 1<<15, 1<<11}}},
      {"GDDR6_16Gb_x16",     {16<<10,   16, {2, 1, 4, 4, 1<<14, 1<<11}}},
      {"GDDR6_32Gb_x8",      {32<<10,   8,  {2, 1, 4, 4, 1<<16, 1<<11}}},
      {"GDDR6_32Gb_x16",     {32<<10,   16, {2, 1, 4, 4, 1<<15, 1<<11}}},
      {"GDDR6_16Gb_x16_pch", {16<<10,   16, {1, 1, 4, 4, 1<<15, 1<<11}}},
    };

    
    inline static const std::map<std::string, std::vector<int>> timing_presets = {
      //       name                rate   nBL  nCL  nRCDRD nRCDWD  nRP   nRAS  nRC   nWR  nRTP nCWL nCCDS nCCDL nRRDS nRRDL nWTRS nWTRL nFAW  nRFC nRFCpb nRREFD nREFI  tCK_ps
      {"GDDR6_16000_1350mV_double",{16000,  8,  24,     26,    16,  26,   53,   79,   26,   4,   6,   4,    6,    7,    7,   9,    11,   28,   210,  105,   14,   3333,   570}},
      {"GDDR6_16000_1250mV_double",{16000,  8,  24,     30,    19,  30,   60,   89,   30,   4,   6,   4,    6,   11,   11,   9,    11,   42,   210,  105,   21,   3333,   570}},
      {"GDDR6_16000_1350mV_quad",  {16000,  4,  24,     26,    16,  26,   53,   79,   26,   4,   6,   4,    6,    7,    7,   9,    11,   28,   210,  105,   14,   3333,   570}},
      {"GDDR6_16000_1250mV_quad",  {16000,  4,  24,     30,    19,  30,   60,   89,   30,   4,   6,   4,    6,   11,   11,   9,    11,   42,   210,  105,   21,   3333,   570}}, // from ramulator
      {"GDDR6_14000_1250mV_quad",  {14000,  4,  24,     27,    20,  27,   57,   84,   23,   4,   6,   4,    5,   10,   10,   8,     9,   37,   210,  105,   21,   3333,   570}}, // averaged between DRAMsim3 and ramulator
      {"GDDR6_12000_1250mV_quad",  {12000,  4,  24,     24,    20,  24,   54,   78,   16,   3,  16,   3,    4,    9,    9,   7,     7,   32,   210,  105,   21,   3333,   570}}, // from DRAMsim3

      //scaled from 12000
      {"GDDR6x_21000_1350mV_quad",  {21000, 4,  24,   42,     35,  42,   95,  137,   28,   5,  28,   5,    7,   16,   16,  13,    13,   56,   221,  184,   37,   3333,   570}},
    };
#ifdef USE_DRAMPOWER
    inline static const std::map<std::string, std::vector<double>> voltage_presets = {
      //   name          VDD       VDDQ
      {"Default",       {1.35,     1.35}},
    };

    inline static const std::map<std::string, std::vector<double>> current_presets = {
      // from DRAMsim3
      // name           IDD0  IDD2N   IDD3N   IDD4R   IDD4W   IDD5B   IDD2P  IDD3P  IDD5C  IDD5F  IDD6N  IBETA
      {"Default",       {71,   60,     61,     248,    231,    286,     45,    50,     45,   135,    35,  56.25}},
    };
#else
    inline static const std::map<std::string, std::vector<double>> voltage_presets = {
        //   name          VDD      VPP
        {"Default",       {1.35,     1.8}},
    };

    inline static const std::map<std::string, std::vector<double>> current_presets = {
        // name           IDD0  IDD2N   IDD3N   IDD4R   IDD4W   IDD5B   IPP0  IPP2N  IPP3N  IPP4R  IPP4W  IPP5B
        {"Default",       {71,   60,     61,     248,    231,    286,     3,    3,     3,     3,     3,     48}},
    };
#endif // USE_DRAMPOWER
    
  /************************************************
   *                Organization
   ***********************************************/   
    const int m_internal_prefetch_size = 16;    // 16n prefetch

    inline static constexpr ImplDef m_levels = {
      "channel", "rank", "bankgroup", "bank", "row", "column",    
    };


  /************************************************
   *             Requests & Commands
   ***********************************************/
    inline static constexpr ImplDef m_commands = { //figure 3
      "ACT", 
      "PREA", "PRE",
      "RD",  "WR",  "RDA",  "WRA",
      "REFab", "REFpb", "REFp2b", "REFab_end"
    };

    inline static const ImplLUT m_command_scopes = LUT (
      m_commands, m_levels, {
        {"REFab", "rank"},  {"REFp2b",  "channel"},
        {"ACT",   "row"},
        {"PREA", "bank"},   {"PRE",  "bank"},  {"REFpb", "bank"}, {"REFab_end",  "rank"},
        {"RD",    "column"}, {"WR",   "column"},  {"RDA",  "column"}, {"WRA",   "column"},
      }
    );

    inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta> (
      m_commands, {
                // open?   close?   access?  refresh?
        {"ACT",   {true,   false,   false,   false}},
        {"PREA",  {false,  true,    false,   false}},
        {"PRE",   {false,  true,    false,   false}},
        {"RD",    {false,  false,   true,    false}},
        {"WR",    {false,  false,   true,    false}},
        {"RDA",   {false,  true,    true,    false}},
        {"WRA",   {false,  true,    true,    false}},
        {"REFab", {false,  false,   false,   true} }, //double check
        {"REFpb", {false,  false,   false,   true} },
        {"REFab_end",   {false,  true,    false,   false}},
        {"REFp2b",{false,  false,   false,   true} },
      }
    );

    inline static constexpr ImplDef m_requests = {
      "read", "write", "all-bank-refresh", "PREsb",
    };

    inline static const ImplLUT m_request_translations = LUT (
      m_requests, m_commands, {
        {"read", "RD"}, {"write", "WR"}, {"all-bank-refresh", "REFab"}, {"PREsb", "PRE"}
      }
    );

   
  /************************************************
   *                   Timing
   ***********************************************/
  //delete nCS
    inline static constexpr ImplDef m_timings = {
      "rate", 
      "nBL", "nCL", "nRCDRD", "nRCDWD", "nRP", "nRAS", "nRC", "nWR", "nRTP", "nCWL",
      "nCCDS", "nCCDL",
      "nRRDS", "nRRDL",
      "nWTRS", "nWTRL",
      "nFAW",
      "nRFC", "nRFCpb",  "nRREFD", "nREFI",
      "tCK_ps"
    };
   
  /************************************************
   *                   Power
   ***********************************************/
#ifdef USE_DRAMPOWER
    inline static constexpr ImplDef m_voltages = {
      "VDD", "VDDQ"
    };

    inline static constexpr ImplDef m_currents = {
      "IDD0", "IDD2N", "IDD3N", "IDD4R", "IDD4W", "IDD5B",
      "IDD2P", "IDD3P", "IDD5C", "IDD5F", "IDD6N", "IBETA"
    };
#else
    inline static constexpr ImplDef m_voltages = {
      "VDD", "VPP"
    };

    inline static constexpr ImplDef m_currents = {
      "IDD0", "IDD2N", "IDD3N", "IDD4R", "IDD4W", "IDD5B",
      "IPP0", "IPP2N", "IPP3N", "IPP4R", "IPP4W", "IPP5B"
    };
#endif // USE_DRAMPOWER

    inline static constexpr ImplDef m_cmds_counted = {
      "ACT", "PRE", "RD", "WR", "REF"
    };

  /************************************************
   *                 Node States
   ***********************************************/
    inline static constexpr ImplDef m_states = {
       "Opened", "Closed", "PowerUp", "N/A", "Refreshing"
    };

    inline static const ImplLUT m_init_states = LUT (
      m_levels, m_states, {
        {"channel",   "N/A"}, 
        {"rank",      "PowerUp"},
        {"bankgroup", "N/A"},
        {"bank",      "Closed"},
        {"row",       "Closed"},
        {"column",    "N/A"},
      }
    );

  public:
    struct Node : public DRAMNodeBase<GDDR6A> {
      Node(GDDR6A* dram, Node* parent, int level, int id) : DRAMNodeBase<GDDR6A>(dram, parent, level, id) {};
    };
    std::vector<Node*> m_channels;
    
    FuncMatrix<ActionFunc_t<Node>>  m_actions;
    FuncMatrix<PreqFunc_t<Node>>    m_preqs;
    FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
    FuncMatrix<RowopenFunc_t<Node>> m_rowopens;
    FuncMatrix<PowerFunc_t<Node>>   m_powers;

    float total_power;

  public:
    void tick() override {
      m_clk++;

      // Check if there is any future action at this cycle
      for (int i = m_future_actions.size() - 1; i >= 0; i--) {
        auto& future_action = m_future_actions[i];
        if (future_action.clk == m_clk) {
          handle_future_action(future_action.cmd, future_action.addr_vec);
          m_future_actions.erase(m_future_actions.begin() + i);
        }
      }
    };

    void init() override {
      RAMULATOR_DECLARE_SPECS();
      set_organization();
      set_timing_vals();

      set_actions();
      set_preqs();
      set_rowhits();
      set_rowopens();
      set_powers();
      
      create_nodes();
      total_power = 0;
    };

    void issue_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
      m_channels[channel_id]->update_powers(command, addr_vec, m_clk);
      m_channels[channel_id]->update_states(command, addr_vec, m_clk);
    
      // Check if the command requires future action
      check_future_action(command, addr_vec);
    };

    void check_future_action(int command, const AddrVec_t& addr_vec) {
      switch (command) {
        case m_commands("REFab"):
            m_future_actions.push_back({command, addr_vec, m_clk + m_timing_vals("nRFC") - 1});
          break;
        default:
          // Other commands do not require future actions
          break;
      }
    }

    void handle_future_action(int command, const AddrVec_t& addr_vec) {
      int channel_id = addr_vec[m_levels["channel"]];
      switch (command) {
        case m_commands("REFab"):
          m_channels[channel_id]->update_powers(m_commands("REFab_end"), addr_vec, m_clk);
          m_channels[channel_id]->update_states(m_commands("REFab_end"), addr_vec, m_clk);
          break;
        default:
          // Other commands do not require future actions
          break;
      }
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
    };

    bool check_ready(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
    };
    
    bool check_node_open(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_node_open(command, addr_vec, m_clk);
    };

  private:
    void set_organization() {
      // Channel width
      m_channel_width = param_group("org").param<int>("channel_width").default_val(16);

      // Organization
      m_organization.count.resize(m_levels.size(), -1);

      // Load organization preset if provided
      if (auto preset_name = param_group("org").param<std::string>("preset").optional()) {
        if (org_presets.count(*preset_name) > 0) {
          m_organization = org_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized organization preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Override the preset with any provided settings
      if (auto dq = param_group("org").param<int>("dq").optional()) {
        m_organization.dq = *dq;
      }

      for (int i = 0; i < m_levels.size(); i++){
        auto level_name = m_levels(i);
        if (auto sz = param_group("org").param<int>(level_name).optional()) {
          m_organization.count[i] = *sz;
        }
      }

      if (auto density = param_group("org").param<int>("density").optional()) {
        m_organization.density = *density;
      }

      // Sanity check: is the calculated chip density the same as the provided one?
      size_t _density = size_t(m_organization.count[m_levels["channel"]]) *
                        size_t(m_organization.count[m_levels["bankgroup"]]) *
                        size_t(m_organization.count[m_levels["bank"]]) *
                        size_t(m_organization.count[m_levels["row"]]) *
                        size_t(m_organization.count[m_levels["column"]]) *
                        size_t(m_organization.dq);
      _density >>= 20;
      if (m_organization.density != _density) {
        throw ConfigurationError(
            "Calculated {} chip density {} Mb does not equal the provided density {} Mb!", 
            get_name(),
            _density, 
            m_organization.density
        );
      }

    };

    void set_timing_vals() {
      m_timing_vals.resize(m_timings.size(), -1);

      // Load timing preset if provided
      bool preset_provided = false;
      if (auto preset_name = param_group("timing").param<std::string>("preset").optional()) {
        if (timing_presets.count(*preset_name) > 0) {
          m_timing_vals = timing_presets.at(*preset_name);
          preset_provided = true;
        } else {
          throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Check for rate (in MT/s), and if provided, calculate and set tCK (in picosecond)
      if (auto dq = param_group("timing").param<int>("rate").optional()) {
        if (preset_provided) {
          throw ConfigurationError("Cannot change the transfer rate of {} when using a speed preset !", get_name());
        }
        m_timing_vals("rate") = *dq;
      }
      int tCK_ps = 1E6 / (m_timing_vals("rate") / 8);
      m_timing_vals("tCK_ps") = tCK_ps;

      // Load the organization specific timings
      int dq_id = [](int dq) -> int {
        switch (dq) {
          case 8:  return 0;
          case 16: return 1;
          default: return -1;
        }
      }(m_organization.dq);

      int rate_id = [](int rate) -> int { //should low voltage operation be added here?
        switch (rate) {
          case 2000:  return 0;
          default:    return -1;
        }
      }(m_timing_vals("rate"));

      // Tables for secondary timings determined by the frequency, density, and DQ width.
      // Defined in the JEDEC standard (e.g., Table 169-170, JESD79-4C).

      //update these values
      constexpr int nRRDS_TABLE[2][1] = {
      // 2000
        { 4 },   // x8
        { 5 },   // x16
      };
      constexpr int nRRDL_TABLE[2][1] = {
      // 2000
        { 5 },  // x8
        { 6 },  // x16
      };
      constexpr int nFAW_TABLE[2][1] = {
      // 2000
        { 20 },  // x8
        { 28 },  // x16
      };

      if (dq_id != -1 && rate_id != -1) {
        m_timing_vals("nRRDS") = nRRDS_TABLE[dq_id][rate_id];
        m_timing_vals("nRRDL") = nRRDL_TABLE[dq_id][rate_id];
        m_timing_vals("nFAW")  = nFAW_TABLE [dq_id][rate_id];
      }

      // Refresh timings
      // tRFC table (unit is nanosecond!)
      constexpr int tRFC_TABLE[3][4] = {
      //  4Gb   8Gb  16Gb  32Gb
        { 260,  360,  550, 750}, // Normal refresh (tRFC1)
        { 160,  260,  350, 550}, // FGR 2x (tRFC2)
        { 110,  160,  260, 350}, // FGR 4x (tRFC4)
      };

      // tREFI(base) table (unit is nanosecond!)
      constexpr int tREFI_BASE = 7800;
      int density_id = [](int density_Mb) -> int { 
        switch (density_Mb) {
          case 4096:  return 0;
          case 8192:  return 1;
          case 16384: return 2;
          case 32768: return 3;
          default:    return -1;
        }
      }(m_organization.density);

      m_timing_vals("nRFC")  = JEDEC_rounding(tRFC_TABLE[0][density_id], tCK_ps);
      m_timing_vals("nREFI") = JEDEC_rounding(tREFI_BASE, tCK_ps);

      // Overwrite timing parameters with any user-provided value
      // Rate and tCK should not be overwritten
      for (int i = 1; i < m_timings.size() - 1; i++) {
        auto timing_name = std::string(m_timings(i));

        if (auto provided_timing = param_group("timing").param<int>(timing_name).optional()) {
          // Check if the user specifies in the number of cycles (e.g., nRCD)
          m_timing_vals(i) = *provided_timing;
        } else if (auto provided_timing = param_group("timing").param<float>(timing_name.replace(0, 1, "t")).optional()) {
          // Check if the user specifies in nanoseconds (e.g., tRCD)
          m_timing_vals(i) = JEDEC_rounding(*provided_timing, tCK_ps);
        }
      }

      // Check if there is any uninitialized timings
      for (int i = 0; i < m_timing_vals.size(); i++) {
        if (m_timing_vals(i) == -1) {
          throw ConfigurationError("In \"{}\", timing {} is not specified!", get_name(), m_timings(i));
        }
      }      

      // Set read latency
      m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL");

      // Populate the timing constraints
      #define V(timing) (m_timing_vals(timing))
      populate_timingcons(this, {
          /*** Channel ***/ 
          // CAS <-> CAS
          /// Data bus occupancy
          {.level = "channel", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nBL")},
          {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nBL")},

          /*** Rank (or different BankGroup) ***/
          // changed from rank to channel, some duplicates, what takes
          // CAS <-> CAS
          /// nCCDS is the minimal latency for column commands 
          {.level = "channel", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nCCDS")},
          {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nCCDS")},
          /// RD <-> WR, Minimum Read to Write, Assuming tWPRE = 1 tCK                          
          {.level = "channel", .preceding = {"RD", "RDA"}, .following = {"WR", "WRA"}, .latency = V("nCL") + V("nBL") + 3 - V("nCWL") + 1}, //+ 1 is assuming bus turn around time
          /// WR <-> RD, Minimum Read after Write
          {.level = "channel", .preceding = {"WR", "WRA"}, .following = {"RD", "RDA"}, .latency = V("nCWL") + V("nBL") + V("nWTRS")},
          /// CAS <-> PREA
          {.level = "channel", .preceding = {"RD"}, .following = {"PREA"}, .latency = V("nRTP")},
          {.level = "channel", .preceding = {"WR"}, .following = {"PREA"}, .latency = V("nCWL") + V("nBL") + V("nWR")},          
          /// RAS <-> RAS
          {.level = "channel", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nRRDS")},          
          {.level = "channel", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nFAW"), .window = 4},       
          {.level = "channel", .preceding = {"ACT"}, .following = {"PRE"}, .latency = V("nRAS")},          
          {.level = "channel", .preceding = {"PRE"}, .following = {"ACT"}, .latency = V("nRP")},          
          /// RAS <-> REF
          {.level = "channel", .preceding = {"ACT"}, .following = {"REFab"}, .latency = V("nRC")},          
          {.level = "channel", .preceding = {"PRE"}, .following = {"REFab"}, .latency = V("nRP")},          
          {.level = "channel", .preceding = {"RDA"}, .following = {"REFab"}, .latency = V("nRP") + V("nRTP")},          
          {.level = "channel", .preceding = {"WRA"}, .following = {"REFab"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},          
          {.level = "channel", .preceding = {"REFab"}, .following = {"ACT"}, .latency = V("nRFC")},          
          
          /// RAS <-> REFp2b
          {.level = "channel", .preceding = {"ACT"}, .following = {"REFp2b"}, .latency = V("nRRDL")},          
          {.level = "channel", .preceding = {"PRE"}, .following = {"REFp2b"}, .latency = V("nRP")},          
          {.level = "channel", .preceding = {"RDA"}, .following = {"REFp2b"}, .latency = V("nRP") + V("nRTP")},          
          {.level = "channel", .preceding = {"WRA"}, .following = {"REFp2b"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},          
          {.level = "channel", .preceding = {"REFp2b"}, .following = {"ACT"}, .latency = V("nRREFD")},   

          /// RAS <-> REFpb
          {.level = "channel", .preceding = {"ACT"}, .following = {"REFpb"}, .latency = V("nRRDL")},          
          {.level = "channel", .preceding = {"PRE"}, .following = {"REFpb"}, .latency = V("nRP")},          
          {.level = "channel", .preceding = {"RDA"}, .following = {"REFpb"}, .latency = V("nRP") + V("nRTP")},          
          {.level = "channel", .preceding = {"WRA"}, .following = {"REFpb"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},          
          {.level = "channel", .preceding = {"REFpb"}, .following = {"ACT"}, .latency = V("nRREFD")},   


          /*** Same Bank Group ***/ 
          /// CAS <-> CAS
          {.level = "bankgroup", .preceding = {"RD", "RDA"}, .following = {"RD", "RDA"}, .latency = V("nCCDL")},          
          {.level = "bankgroup", .preceding = {"WR", "WRA"}, .following = {"WR", "WRA"}, .latency = V("nCCDL")},          
          {.level = "bankgroup", .preceding = {"WR", "WRA"}, .following = {"RD", "RDA"}, .latency = V("nCWL") + V("nBL") + V("nWTRL")},
          /// RAS <-> RAS
          {.level = "bankgroup", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nRRDL")},  

          /*** Bank ***/ 
          /// CAS <-> RAS
          {.level = "bank", .preceding = {"ACT"}, .following = {"RD", "RDA"}, .latency = V("nRCDRD")}, 
          {.level = "bank", .preceding = {"ACT"}, .following = {"WR", "WRA"}, .latency = V("nRCDWD")},
          {.level = "bank", .preceding = {"ACT"}, .following = {"PRE"}, .latency = V("nRAS")},  
          {.level = "bank", .preceding = {"PRE"}, .following = {"ACT"}, .latency = V("nRP")},  
          {.level = "bank", .preceding = {"RD"},  .following = {"PRE"}, .latency = V("nRTP")},  
          {.level = "bank", .preceding = {"WR"},  .following = {"PRE"}, .latency = V("nCWL") + V("nBL") + V("nWR")},  
          {.level = "bank", .preceding = {"RDA"}, .following = {"ACT"}, .latency = V("nRTP") + V("nRP")},  
          {.level = "bank", .preceding = {"WRA"}, .following = {"ACT"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},  

          /// RAS <-> REFpb
          {.level = "bank", .preceding = {"ACT"}, .following = {"REFpb"}, .latency = V("nRC")},          
          {.level = "bank", .preceding = {"PRE"}, .following = {"REFpb"}, .latency = V("nRP")},          
          {.level = "bank", .preceding = {"RDA"}, .following = {"REFpb"}, .latency = V("nRP") + V("nRTP")},          
          {.level = "bank", .preceding = {"WRA"}, .following = {"REFpb"}, .latency = V("nCWL") + V("nBL") + V("nWR") + V("nRP")},          
          {.level = "bank", .preceding = {"REFpb"}, .following = {"ACT"}, .latency = V("nRFCpb")},   

          /// RAS <-> RAS
          //{.level = "bank", .preceding = {"ACT"}, .following = {"ACT"}, .latency = V("nRC")}, //should this be added?
        }
      );
      #undef V

    };

    void set_actions() {
      m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));

      // Channel Actions 
      m_actions[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Action::Rank::PREab<GDDR6A>;
      m_actions[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Action::Rank::REFab<GDDR6A>;
      m_actions[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Action::Rank::REFab_end<GDDR6A>;

      // Bank actions
      m_actions[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Action::Bank::ACT<GDDR6A>;
      m_actions[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Action::Bank::PRE<GDDR6A>;
      m_actions[m_levels["bank"]][m_commands["RDA"]] = Lambdas::Action::Bank::PRE<GDDR6A>;
      m_actions[m_levels["bank"]][m_commands["WRA"]] = Lambdas::Action::Bank::PRE<GDDR6A>;
    };

    void set_preqs() {
      m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));

      // Channel Actions 
      m_preqs[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<GDDR6A>;

      // Bank actions
      m_preqs[m_levels["bank"]][m_commands["RD"]] = Lambdas::Preq::Bank::RequireRowOpen<GDDR6A>;
      m_preqs[m_levels["bank"]][m_commands["WR"]] = Lambdas::Preq::Bank::RequireRowOpen<GDDR6A>;
      //m_preqs[m_levels["channel"]][m_commands["REFpb"]] = Lambdas::Preq::Bank::RequireAllBanksClosed<GDDR6A>; // can RequireSameBanksClosed be used, or is RequireBankClosed needed?
      //m_preqs[m_levels["channel"]][m_commands["REFp2b"]] = Lambdas::Preq::Bank::RequireAllBanksClosed<GDDR6A>; 
    };

    void set_rowhits() {
      m_rowhits.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowhits[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowHit::Bank::RDWR<GDDR6A>;
      m_rowhits[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowHit::Bank::RDWR<GDDR6A>;
    }


    void set_rowopens() {
      m_rowopens.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));

      m_rowopens[m_levels["bank"]][m_commands["RD"]] = Lambdas::RowOpen::Bank::RDWR<GDDR6A>;
      m_rowopens[m_levels["bank"]][m_commands["WR"]] = Lambdas::RowOpen::Bank::RDWR<GDDR6A>;
    }

    void set_powers() {
      
      m_drampower_enable = param<bool>("drampower_enable").default_val(false);

      if (!m_drampower_enable)
        return;

      m_voltage_vals.resize(m_voltages.size(), -1);

      if (auto preset_name = param_group("voltage").param<std::string>("preset").optional()) {
        if (voltage_presets.count(*preset_name) > 0) {
          m_voltage_vals = voltage_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized voltage preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      m_current_vals.resize(m_currents.size(), -1);

      if (auto preset_name = param_group("current").param<std::string>("preset").optional()) {
        if (current_presets.count(*preset_name) > 0) {
          m_current_vals = current_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized current preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      m_power_debug = param<bool>("power_debug").default_val(false);

      // TODO: Check for multichannel configs.
      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      m_power_stats.resize(num_channels * num_ranks);
      for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_ranks; j++) {
          m_power_stats[i * num_ranks + j].rank_id = i * num_ranks + j;
          m_power_stats[i * num_ranks + j].cmd_counters.resize(m_cmds_counted.size(), 0);
        }
      }

      m_powers.resize(m_levels.size(), std::vector<PowerFunc_t<Node>>(m_commands.size()));

      m_powers[m_levels["bank"]][m_commands["ACT"]] = Lambdas::Power::Bank::ACT<GDDR6A>;
      m_powers[m_levels["bank"]][m_commands["PRE"]] = Lambdas::Power::Bank::PRE<GDDR6A>;
      m_powers[m_levels["bank"]][m_commands["RD"]]  = Lambdas::Power::Bank::RD<GDDR6A>;
      m_powers[m_levels["bank"]][m_commands["WR"]]  = Lambdas::Power::Bank::WR<GDDR6A>;

      m_powers[m_levels["rank"]][m_commands["ACT"]] = Lambdas::Power::Rank::ACT<GDDR6A>;
      m_powers[m_levels["rank"]][m_commands["PRE"]] = Lambdas::Power::Rank::PRE<GDDR6A>;
      m_powers[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Power::Rank::PREA<GDDR6A>;
      m_powers[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Power::Rank::REFab<GDDR6A>;
      m_powers[m_levels["rank"]][m_commands["REFab_end"]] = Lambdas::Power::Rank::REFab_end<GDDR6A>;

      // register stats
      /*register_stat(s_total_background_energy).name("total_background_energy");
      register_stat(s_total_cmd_energy).name("total_cmd_energy");
      register_stat(s_total_energy).name("total_energy");*/
      register_stat(total_power).name("total_power(W)");
            
      /*for (auto& power_stat : m_power_stats){
        register_stat(power_stat.total_background_energy).name("total_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.total_cmd_energy).name("total_cmd_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.total_energy).name("total_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.act_background_energy).name("act_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.pre_background_energy).name("pre_background_energy_rank{}", power_stat.rank_id);
        register_stat(power_stat.active_cycles).name("active_cycles_rank{}", power_stat.rank_id);
        register_stat(power_stat.idle_cycles).name("idle_cycles_rank{}", power_stat.rank_id);
      }*/
    }

    void create_nodes() {
      int num_channels = m_organization.count[m_levels["channel"]];
      for (int i = 0; i < num_channels; i++) {
        Node* channel = new Node(this, nullptr, 0, i);
        m_channels.push_back(channel);
      }
    }
    
    void finalize() override {
      if (!m_drampower_enable)
        return;

      int num_channels = m_organization.count[m_levels["channel"]];
      int num_ranks = m_organization.count[m_levels["rank"]];
      for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_ranks; j++) {
          process_rank_energy(m_power_stats[i * num_ranks + j], m_channels[i]->m_child_nodes[j]);
        }
      }
      total_power = s_total_energy / ((float)m_clk * (float)m_timing_vals("tCK_ps") / 1000.0);
    }

#ifdef USE_DRAMPOWER
    double E_BG_pre(std::size_t B, double VDD, double IDD2_N, double T_BG_pre) {
        return (1.0 / B) * VDD * IDD2_N * T_BG_pre;
    };

    double E_BG_act_shared(double VDD, double I_rho, double T_BG_act) {
        return VDD * I_rho * T_BG_act;
    }

    double E_BG_act_star(std::size_t B, double VDD, double IDD3_N, double I_rho, double T_BG_act_star) {
        return VDD * (1.0 / B) * (IDD3_N - I_rho) * T_BG_act_star;
    }

    double E_pre(double VDD, double IBeta, double IDD2_N, double t_RP, uint64_t N_pre) {
        return VDD * (IBeta - IDD2_N) * t_RP * N_pre;
    }

    double E_act(double VDD, double I_theta, double I_1, double t_RAS, uint64_t N_act) {
        return VDD * (I_theta - I_1) * t_RAS * N_act;
    }

    double E_RD(double VDD, double IDD4_R, double IDD3_N, double t_CK, std::size_t BL, std::size_t DR,
                                  uint64_t N_RD) {
        return VDD * (IDD4_R - IDD3_N) * (double(BL) / DR) * t_CK * N_RD;
    }

    double E_WR(double VDD, double IDD4_W, double IDD3_N, double t_CK, std::size_t BL, std::size_t DR,
                                  uint64_t N_WR) {
        return VDD * (IDD4_W - IDD3_N) * (BL / DR) * t_CK * N_WR;
    }

    double E_ref_ab(std::size_t B, double VDD, double IDD5B, double IDD3_N, double tRFC, uint64_t N_REF) {
        return (1.0 / B) * VDD * (IDD5B - IDD3_N) * tRFC * N_REF;
    }
#endif // USE_DRAMPOWER

    void process_rank_energy(PowerStats& rank_stats, Node* rank_node) {
      
      Lambdas::Power::Rank::finalize_rank<GDDR6A>(rank_node, 0, AddrVec_t(), m_clk);

      size_t num_bankgroups = m_organization.count[m_levels["bankgroup"]];

      auto TS = [&](std::string_view timing) { return m_timing_vals(timing); };
      auto VE = [&](std::string_view voltage) { return m_voltage_vals(voltage); };
      auto CE = [&](std::string_view current) { return m_current_vals(current); };
      double tCK_ns = (double)TS("tCK_ps") / 1000.0;

#ifdef USE_DRAMPOWER
      float rho = 0.5;
      int B = m_organization.count[m_levels["bank"]];
      int BG = m_organization.count[m_levels["bankgroup"]];

      auto VXX = VE("VDD");
      auto IBeta = CE("IBETA");
      auto IXX2N = CE("IDD2N");
      auto IXX4R = CE("IDD4R");
      auto IXX3N = CE("IDD3N");
      auto IXX4W = CE("IDD4W");
      auto IXX5X = CE("IDD5B");
      auto t_CK_us = tCK_ns / 1000.0;
      auto t_RFC = TS("nRFC") * t_CK_us;
      auto t_RP = TS("nRP") * t_CK_us;
      auto t_RAS = TS("nRAS") * t_CK_us;
      auto BL = TS("nBL");
      int DR = 4;
      auto I_rho = rho * (CE("IDD3N") - CE("IDD2N")) + CE("IDD2N");
      auto I_theta = (CE("IDD0") * (t_RP + t_RAS) - CE("IBETA") * t_RP) * (1 / t_RAS);
      auto I_1 = (1.0 / B) * (CE("IDD3N") + (B - 1) * (rho * (CE("IDD3N") - CE("IDD2N")) + CE("IDD2N")));
      auto I_BG = I_rho + (I_1 - I_rho) * BG;


      rank_stats.act_background_energy = E_BG_act_shared(VE("VDD"), I_rho, rank_stats.active_cycles * t_CK_us);
      rank_stats.pre_background_energy = E_BG_pre(B, VE("VDD"), CE("IDD2N"), rank_stats.idle_cycles * t_CK_us);
      double act_cmd_energy = E_act(VXX, I_theta, I_1, t_RAS, rank_stats.cmd_counters[m_cmds_counted("ACT")]);
      double pre_cmd_energy = E_pre(VXX, IBeta, IXX2N, t_RP, rank_stats.cmd_counters[m_cmds_counted("PRE")]);
      double rd_cmd_energy = E_RD(VXX, IXX4R, IXX3N, t_CK_us, BL, DR, rank_stats.cmd_counters[m_cmds_counted("RD")]);
      double wr_cmd_energy = E_WR(VXX, IXX4W, IXX3N, t_CK_us, BL, DR, rank_stats.cmd_counters[m_cmds_counted("WR")]);
      double ref_cmd_energy = E_ref_ab(B, VXX, IXX5X, IXX3N, t_RFC, rank_stats.cmd_counters[m_cmds_counted("REF")]);
#else
      rank_stats.act_background_energy = (VE("VDD") * CE("IDD3N") + VE("VPP") * CE("IPP3N"))
          * rank_stats.active_cycles * tCK_ns / 1E3;

      rank_stats.pre_background_energy = (VE("VDD") * CE("IDD2N") + VE("VPP") * CE("IPP2N")) 
                                            * rank_stats.idle_cycles * tCK_ns / 1E3;


      double act_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD3N")) + VE("VPP") * (CE("IPP0") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("ACT")] * TS("nRAS") * tCK_ns / 1E3;

      double pre_cmd_energy  = (VE("VDD") * (CE("IDD0") - CE("IDD2N")) + VE("VPP") * (CE("IPP0") - CE("IPP2N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("PRE")] * TS("nRP")  * tCK_ns / 1E3;

      double rd_cmd_energy   = (VE("VDD") * (CE("IDD4R") - CE("IDD3N")) + VE("VPP") * (CE("IPP4R") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("RD")] * TS("nBL") * tCK_ns / 1E3;

      double wr_cmd_energy   = (VE("VDD") * (CE("IDD4W") - CE("IDD3N")) + VE("VPP") * (CE("IPP4W") - CE("IPP3N"))) 
                                      * rank_stats.cmd_counters[m_cmds_counted("WR")] * TS("nBL") * tCK_ns / 1E3;

      double ref_cmd_energy = (VE("VDD") * (CE("IDD5B")) + VE("VPP") * (CE("IPP5B")))
          * rank_stats.cmd_counters[m_cmds_counted("REF")] * TS("nRFC") * tCK_ns / 1E3;
#endif // USE_DRAMPOWER
      //printf("refresh count = %d\n", rank_stats.cmd_counters[m_cmds_counted("REF")]);

      rank_stats.total_background_energy = rank_stats.act_background_energy + rank_stats.pre_background_energy;
      rank_stats.total_cmd_energy = act_cmd_energy 
                                    + pre_cmd_energy 
                                    + rd_cmd_energy
                                    + wr_cmd_energy 
                                    + ref_cmd_energy;

      rank_stats.total_energy = rank_stats.total_background_energy + rank_stats.total_cmd_energy;

      s_total_background_energy += rank_stats.total_background_energy;
      s_total_cmd_energy += rank_stats.total_cmd_energy;
      s_total_energy += rank_stats.total_energy;
    }
};


}        // namespace Ramulator

