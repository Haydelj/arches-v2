#ifndef __MEMORY_CONTROLLER_H__
#define __MEMORY_CONTROLLER_H__
//TODO: Implement arches instructions to memory_Controller


//These includes will have to change for arches implementation
//#include "ThreadState.h"
//#include "SimpleRegisterFile.h"
//#include "Instruction.h"
//#include "StatsCollector.h"

#include <vector>
#include <list>
#include <stdlib.h>
#include "stdafx.hpp"

#define MAX_QUEUE_LENGTH 80
#define MAX_NUM_CHANNELS 16
#define MAX_NUM_RANKS    16
#define MAX_NUM_BANKS    32

#define BIG_ACTIVATION_WINDOW 1000000

#define UNKNOWN_LATENCY       10000000000
#define DRAM_CLOCK_MULTIPLIER 2


// Moved here from main.c 
extern long long int *committed; // total committed instructions in each core
extern long long int *fetched;   // total fetched instructions in each core

extern int           max_write_queue_length         [MAX_NUM_CHANNELS];
extern int           max_read_queue_length          [MAX_NUM_CHANNELS];
extern long long int accumulated_read_queue_length  [MAX_NUM_CHANNELS];

extern long long int update_mem_count;

// General stats
//Not sure this is needed for our current implementation
struct UsimmUsageStats_t
{
    inline void Initialize()
    {
        //statsCacheLineLoads.Clear();
        //statsCacheLineStores.Clear();

        numCacheLineLoadsDups  = 0;
        numCacheLineStoresDups = 0;
    }

    // Distribution of cache lines that were requested by different ops
    // Note: these will duplicate, so if multiple ops use the same cache line
    //       numCacheLine{Loads, Stores}Dups will be incremented
    //StatDistribution<uint32_t op_code, unsigned long long int> statsCacheLineLoads;
    //StatDistribution<uint32_t op_code, unsigned long long int> statsCacheLineStores;

    unsigned long long int numCacheLineLoadsDups;
    unsigned long long int numCacheLineStoresDups;
};

//////////////////////////////////////////////////
//      Memory Controller Data Structures       //
//////////////////////////////////////////////////


// Single request structure self-explanatory
typedef struct arches_request_t
{
    uint return_id;
    uint channel;
} arches_request_t;

// Call-back for TRaX from USIMM
struct UsimmListener
{
    virtual void UsimmNotifyEvent(Arches::cycles_t write_cycle, const arches_request_t& req) = 0;
};

// DRAM Address Structure
typedef struct draddr
{
    Arches::paddr_t actual_address; // physical_address being accessed
    int channel;                  // channel id
    int rank;                     // rank id
    int bank;                     // bank id
    long long int row;            // row/page id
    int column;                   // column id
} dram_address_t;

// DRAM Commands
typedef enum
{
    ACT_CMD,
    COL_READ_CMD,
    PRE_CMD,
    COL_WRITE_CMD,
    PWR_DN_SLOW_CMD,
    PWR_DN_FAST_CMD,
    PWR_UP_CMD,
    REF_CMD,
    NOP
} command_t;

// Request Types
typedef enum
{
    READ,
    WRITE
} optype_t;


//DK: update this struct to hold the PC of issuing instruction
typedef struct req
{
    unsigned long long int  physical_address;
    dram_address_t   dram_addr;
    int64_t          arrival_time;
    int64_t          dispatch_time;      // when COL_RD or COL_WR is issued for this request
    int64_t          completion_time;    // final completion time
    int64_t          latency;            // dispatch_time-arrival_time
//    int                     thread_id;          // core that issued this request
    command_t        next_command;       // what command needs to be issued to make forward progress with this request
    optype_t         operation_type;     // Read/Write
    bool             command_issuable;   // can this request be issued in the current cycle
    bool             request_served;     // if request has it's final command issued or not
//    int                     instruction_id;     // 0 to ROBSIZE-1
//    long long int           instruction_pc;     // phy address of instruction that generated this request (valid only for reads)

    //TRaX stuff
    std::vector<arches_request_t> arches_reqs;
} request_t;

// Returned when inserting a read or a write
typedef struct reqInsertRet_tt
{
    enum REQ_RET_TYPE
    {
        RRT_UNKNOWN = 0,            // just in case
        RRT_READ_QUEUE,             // request found in read queue
        RRT_WRITE_QUEUE,            // request found in write queue
        RRT_READ_QUEUE_FULL,        // no space in read queue
        RRT_WRITE_QUEUE_FULL,       // no space in write queue
    };

    long long int completionTime;
    REQ_RET_TYPE  retType;
    bool          retLatencyKnown;

    reqInsertRet_tt() :
        completionTime(UNKNOWN_LATENCY),
        retType(RRT_UNKNOWN),
        retLatencyKnown(false)
    {
    }
} reqInsertRet_t;

// Bankstates
typedef enum
{
    IDLE,
    PRECHARGING,
    REFRESHING,
    ROW_ACTIVE,
    PRECHARGE_POWER_DOWN_FAST,
    PRECHARGE_POWER_DOWN_SLOW,
    ACTIVE_POWER_DOWN
} bankstate_t;

// Structure to hold the state of a bank
typedef struct bnk
{
    bankstate_t   state;
    int64_t active_row;
    int64_t next_pre;
    int64_t next_act;
    int64_t next_read;
    int64_t next_write;
    int64_t next_powerdown;
    int64_t next_powerup;
    int64_t next_refresh;
} bank_t;

extern struct robstructure * ROB;

extern bool activation_record[MAX_NUM_CHANNELS][MAX_NUM_RANKS][BIG_ACTIVATION_WINDOW];

// contains the states of all banks in the system 
extern bank_t dram_state[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

extern long long int total_col_reads        [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int total_pre_cmds         [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int total_single_col_reads [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int current_col_reads      [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

//[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
// total number of col reads
// total number of pre cmds
// total number of single col reads

// command issued this cycle to this channel
extern bool command_issued_current_cycle[MAX_NUM_CHANNELS];

// cas command issued this cycle to this channel
typedef enum
{
    CIC_NONE = 0,
    CIC_COL_READ,
    CIC_COL_WRITE,
} casIssCyc_t;

extern casIssCyc_t cas_issued_current_cycle[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS]; // 1/2 for COL_READ/COL_WRITE

// Per channel read queue
extern std::list<request_t> read_queue_head[MAX_NUM_CHANNELS];

// Per channel write queue
extern std::list<request_t> write_queue_head[MAX_NUM_CHANNELS];

// issuables_for_different commands
extern bool cmd_precharge_issuable          [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern bool cmd_all_bank_precharge_issuable [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern bool cmd_powerdown_fast_issuable     [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern bool cmd_powerdown_slow_issuable     [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern bool cmd_powerup_issuable            [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern bool cmd_refresh_issuable            [MAX_NUM_CHANNELS][MAX_NUM_RANKS];


// refresh variables
extern long long int next_refresh_completion_deadline[MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int last_refresh_completion_deadline[MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern bool          forced_refresh_mode_on          [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern int           refresh_issue_deadline          [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
//extern int issued_forced_refresh_commands[MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern int           num_issued_refreshes            [MAX_NUM_CHANNELS][MAX_NUM_RANKS];

extern long long int read_queue_length [MAX_NUM_CHANNELS];
extern long long int write_queue_length[MAX_NUM_CHANNELS];

// Stats
extern long long int num_read_merge;
extern long long int num_write_merge;
extern long long int stats_reads_merged_per_channel [MAX_NUM_CHANNELS];
extern long long int stats_writes_merged_per_channel[MAX_NUM_CHANNELS];
extern long long int stats_reads_seen               [MAX_NUM_CHANNELS];
extern long long int stats_writes_seen              [MAX_NUM_CHANNELS];
extern long long int stats_reads_completed          [MAX_NUM_CHANNELS];
extern long long int stats_writes_completed         [MAX_NUM_CHANNELS];

extern double stats_average_read_latency            [MAX_NUM_CHANNELS];
extern double stats_average_read_queue_latency      [MAX_NUM_CHANNELS];
extern double stats_average_write_latency           [MAX_NUM_CHANNELS];
extern double stats_average_write_queue_latency     [MAX_NUM_CHANNELS];

extern long long int stats_page_hits        [MAX_NUM_CHANNELS];
extern double        stats_read_row_hit_rate[MAX_NUM_CHANNELS];

// Time spent in various states
extern long long int stats_time_spent_in_active_standby                 [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_time_spent_in_active_power_down              [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_time_spent_in_precharge_power_down_fast      [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_time_spent_in_precharge_power_down_slow      [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_time_spent_in_power_up                       [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int last_activate                                      [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int last_refresh                                       [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern double        average_gap_between_activates                      [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern double        average_gap_between_refreshes                      [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_time_spent_terminating_reads_from_other_ranks[MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_time_spent_terminating_writes_to_other_ranks [MAX_NUM_CHANNELS][MAX_NUM_RANKS];

// Command Counters
extern long long int stats_num_activate_read    [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int stats_num_activate_write   [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int stats_num_activate_spec    [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int stats_num_activate         [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_num_precharge        [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int stats_num_read             [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int stats_num_write            [MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
extern long long int stats_num_powerdown_slow   [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_num_powerdown_fast   [MAX_NUM_CHANNELS][MAX_NUM_RANKS];
extern long long int stats_num_powerup          [MAX_NUM_CHANNELS][MAX_NUM_RANKS];



// functions

// to get log with base 2
unsigned int log_base2(unsigned int new_value);

// initialize memory_controller variables
void init_memory_controller_vars();

// called every cycle to update the read/write queues
void update_memory();

// activate to bank allowed or not
bool is_activate_allowed(const int channel,
                         const int rank,
                         const int bank);

// precharge to bank allowed or not
bool is_precharge_allowed(const int channel,
                          const int rank,
                          const int bank);

// all bank precharge allowed or not
bool is_all_bank_precharge_allowed(const int channel,
                                   const int rank);

// autoprecharge allowed or not
bool is_autoprecharge_allowed(const int channel,
                              const int rank,
                              const int bank);

// power_down fast allowed or not
bool is_powerdown_fast_allowed(const int channel,
                               const int rank);

// power_down slow allowed or not
bool is_powerdown_slow_allowed(const int channel,
                               const int rank);

// powerup allowed or not
bool is_powerup_allowed(const int channel,
                        const int rank);

// refresh allowed or not
bool is_refresh_allowed(const int channel,
                        const int rank);


// issues command to make progress on a request
bool issue_request_command(request_t * req);

// power_down command
bool issue_powerdown_command(const int channel,
                             const int rank,
                             const command_t cmd);

// powerup command
bool issue_powerup_command(const int channel,
                           const int rank);

// precharge a bank
bool issue_activate_command(const int channel,
                            const int rank,
                            const int bank,
                            const long long int row);

// precharge a bank
bool issue_precharge_command(const int channel,
                             const int rank,
                             const int bank);

// precharge all banks in a rank
bool issue_all_bank_precharge_command(const int channel,
                                      const int rank);

// refresh all banks
bool issue_refresh_command(const int channel,
                           const int rank);

// autoprecharge all banks
bool issue_autoprecharge(const int channel,
                         const int rank,
                         const int bank);

// find if there is a matching write request
reqInsertRet_tt::REQ_RET_TYPE read_exists_in_write_or_read_queue(const dram_address_t &physical_address,
                                                                 request_t*& foundRequest);

// find if there is a matching request in the write queue
bool write_exists_in_write_queue(const dram_address_t &physical_address,
                                 request_t*& foundRequest);

// enqueue a read into the corresponding read queue (returns ptr to new node)
reqInsertRet_t insert_read(const dram_address_t &dram_address,
                           const arches_request_t &arches_request,
                           Arches::cycles_t arrival_time);
//                           const int instruction_id,
//                           const long long int instruction_pc);

// enqueue a write into the corresponding write queue (returns ptr to new_node)
reqInsertRet_t insert_write(const dram_address_t &dram_address,
                            const arches_request_t &arches_request,
                            Arches::cycles_t arrival_time);
//                            const int instruction_id,
//                            const long long int instruction_pc);

int numDramChannels();
dram_address_t calcDramAddr(Arches::paddr_t physical_address);
void registerUsimmListener(UsimmListener* listener);

// convert the TRaX address to byte-addressed, cache-line-aligned
inline long long int traxAddrToUsimm(const int address, const int lineSize)
{
    long long int retVal = address;

    // multiply by 4 (word -> byte), then mask off bits to get cache line number
    retVal *= 4;
    retVal &= (0xFFFFFFFF << (lineSize + 2)); // +2 here to convert from word line-size to byte line-size

    return retVal;
}

// gets how many LSBs from trax address are masked to compute row address
int traxAddrGetBitsToRowBuffer();

// update stats counters
void gather_stats(const int channel);

// print statistics
extern void print_stats();

// calculate power for each channel
float calculate_power(const int channel,
                      const int rank,
                      const int print_stats_type,
                      const int chips_per_rank,
                      const bool print = false);

#endif // __MEM_CONTROLLER_HH__
