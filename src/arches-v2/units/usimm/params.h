#ifndef __PARAMS_H__
#define __PARAMS_H__

#include "stdafx.hpp"
/********************/
/* Processor params */
/********************/

// number of cores in mulicore 
extern uint32_t NUMCORES;

// processor clock frequency multiplier : multiplying the
// DRAM_CLK_FREQUENCY by the following parameter gives the processor
// clock frequency 
extern uint32_t PROCESSOR_CLK_MULTIPLIER;
extern uint32_t ROBSIZE;                     // size of ROB
extern uint32_t MAX_RETIRE;                  // maximum commit width
extern uint32_t MAX_FETCH;                   // maximum instruction fetch width
extern uint32_t PIPELINEDEPTH;               // depth of pipeline


/*****************************/
/* DRAM System Configuration */
/*****************************/
extern int NUM_CHANNELS;                // total number of channels in the system
extern int NUM_RANKS;                   // number of ranks per channel
extern int NUM_BANKS;                   // number of banks per rank
extern int NUM_ROWS;                    // number of rows per bank
extern int NUM_COLUMNS;                 // number of columns per rank
extern int CACHE_LINE_SIZE;             // cache-line size (bytes)
extern int ADDRESS_BITS;                // total number of address bits (i.e. indicates size of memory)


/****************************/
/* DRAM Chip Specifications */
/****************************/
extern int DRAM_CLK_FREQUENCY;          // dram frequency (not datarate) in MHz

// All the following timing parameters should be
// entered in the config file in terms of memory
// clock cycles.
extern uint32_t T_RCD;                       // RAS to CAS delay
extern uint32_t T_RP;                        // PRE to RAS
extern uint32_t T_CAS;                       // ColumnRD to Data burst
extern uint32_t T_RAS;                       // RAS to PRE delay
extern uint32_t T_RC;                        // Row Cycle time
extern uint32_t T_CWD;                       // ColumnWR to Data burst
extern uint32_t T_WR;                        // write recovery time (COL_WR to PRE)
extern uint32_t T_WTR;                       // write to read turnaround
extern uint32_t T_RTRS;                      // rank to rank switching time
extern uint32_t T_DATA_TRANS;                // Data transfer
extern uint32_t T_RTP;                       // Read to PRE
extern uint32_t T_CCD;                       // CAS to CAS
extern uint32_t T_XP;                        // Power UP time fast
extern uint32_t T_XP_DLL;                    // Power UP time slow
extern uint32_t T_CKE;                       // Power down entry
extern uint32_t T_PD_MIN;                    // Minimum power down duration
extern uint32_t T_RRD;                       // rank to rank delay (ACTs to same rank)
extern uint32_t T_FAW;                       // four bank activation window
extern uint32_t T_REFI;                      // refresh extern interval
extern uint32_t T_RFC;                       // refresh cycle time


/****************************/
/* VOLTAGE & CURRENT VALUES */
/****************************/
extern float VDD;
extern float IDD0;
extern float IDD1;
extern float IDD2P0;
extern float IDD2P1;
extern float IDD2N;
extern float IDD3P;
extern float IDD3N;
extern float IDD4R;
extern float IDD4W;
extern float IDD5;


/******************************/
/* MEMORY CONTROLLER Settings */
/******************************/
extern int WQ_CAPACITY;                 // maximum capacity of write queue (per channel)
extern int WQ_LOOKUP_LATENCY;           // WQ associative lookup 

// Address mapping mode
// 1 is consecutive cache-lines to same row
// 2 is consecutive cache-lines striped across different banks 
extern int ADDRESS_MAPPING;

#endif // __PARAMS_H__
