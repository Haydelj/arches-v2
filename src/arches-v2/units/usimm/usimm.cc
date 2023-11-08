/*
DK notes:

What to do about multiple requests returning at the same time?

Extreme example is every core issues read to same address. Trax will have a lot of bank conflicts here, but a few of them will get through to dram.
By the time the dram request is ready, most of the rest of the requests will also have made it through the bank conflicts, and will be waiting on the dram return.

Will need to keep a list of ALL requests going to the same usimm request structure (request_t). Once the request finish time is know, walk that list and update all 
affected threads' ready cycles for the given register.
*/


#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<assert.h>

#include "processor.h"
#include "configfile.h"
#include "memory_controller.h"
#include "scheduler.h"
#include "params.h"


#define MAXTRACELINESIZE 64

// relative paths between install (bin) folder for simtrax and scene folder
// CMAKE will define this parameter during generation, but if we use Makefiles,
// then this should be just ../
#ifndef REL_PATH_BIN_TO_SAMPLES
#  define REL_PATH_BIN_TO_SAMPLES "../"
#endif


extern int arches_verbosity;
extern UsimmUsageStats_t usimmUsageStats;

// Tons of globals from params.h
/********************/
/* Processor params */
/********************/
uint32_t NUMCORES;                           // number of cores in mulicore 

// processor clock frequency multiplier : multiplying the
// DRAM_CLK_FREQUENCY by the following parameter gives the processor
uint32_t PROCESSOR_CLK_MULTIPLIER;           // clock frequency 
uint32_t ROBSIZE;                            // size of ROB (128)
uint32_t MAX_RETIRE;                         // maximum commit width (2)
uint32_t MAX_FETCH;                          // maximum instruction fetch width (4)
uint32_t PIPELINEDEPTH;                      // depth of pipeline (5)


/*****************************/
/* DRAM System Configuration */
/*****************************/
int NUM_CHANNELS;                       // total number of channels in the system (1)
int NUM_RANKS;                          // number of ranks per channel (2)
int NUM_BANKS;                          // number of banks per rank (8)
int NUM_ROWS;                           // number of rows per bank (32768)
int NUM_COLUMNS;                        // number of columns per rank (128)
int CACHE_LINE_SIZE;                    // cache-line size (bytes) (64)
int ADDRESS_BITS;                       // total number of address bits (i.e. indicates size of memory) (32)


/****************************/
/* DRAM Chip Specifications */
/****************************/
int DRAM_CLK_FREQUENCY;                 // dram frequency (not datarate) in MHz (800)

// All the following timing parameters should be 
// entered in the config file in terms of memory 
// clock cycles.
uint32_t T_RCD;                              // RAS to CAS delay (44)
uint32_t T_RP;                               // PRE to RAS (44)
uint32_t T_CAS;                              // ColumnRD to Data burst (44)
uint32_t T_RAS;                              // RAS to PRE delay (112)
uint32_t T_RC;                               // Row Cycle time (156)
uint32_t T_CWD;                              // ColumnWR to Data burst (20)
uint32_t T_WR;                               // write recovery time (COL_WR to PRE) (48)
uint32_t T_WTR;                              // write to read turnaround (24)
uint32_t T_RTRS;                             // rank to rank switching time (8)
uint32_t T_DATA_TRANS;                       // Data transfer (16)
uint32_t T_RTP;                              // Read to PRE (24)
uint32_t T_CCD;                              // CAS to CAS (16)
uint32_t T_XP;                               // Power UP time fast (20)
uint32_t T_XP_DLL;                           // Power UP time slow (40)
uint32_t T_CKE;                              // Power down entry (16)
uint32_t T_PD_MIN;                           // Minimum power down duration (16)
uint32_t T_RRD;                              // rank to rank delay (ACTs to same rank) (20)
uint32_t T_FAW;                              // four bank activation window (128)
uint32_t T_REFI;                             // refresh extern interval
uint32_t T_RFC;                              // refresh cycle time


/****************************/
/* VOLTAGE & CURRENT VALUES */
/****************************/
float VDD;
float IDD0;
float IDD1;
float IDD2P0;
float IDD2P1;
float IDD2N;
float IDD3P;
float IDD3N;
float IDD4R;
float IDD4W;
float IDD5;


/******************************/
/* MEMORY CONTROLLER Settings */
/******************************/
int WQ_CAPACITY;                        // maximum capacity of write queue (per channel) (64)
int WQ_LOOKUP_LATENCY;                  // WQ associative lookup

// 1 is consecutive cache-lines to same row
// 2 is consecutive cache-lines striped across different banks 
int ADDRESS_MAPPING;                    // Address mapping mode (1)

//--------------------------end params.h globals



long long int BIGNUM    = 1000000;
 Arches::cycles_t CYCLE_VAL = 0;

int expt_done = 0;


//DK: Making these global so that printUsimmStats can use them
uint32_t numc            = 0;
int chips_per_rank  = -1;
uint32_t num_ret         = 0;

 Arches::cycles_t get_current_cycle()
{
    return CYCLE_VAL;
}

//struct robstructure *ROB;

FILE **tif;             // The handles to the trace input files
FILE *config_file;
FILE *vi_file;

int *prefixtable;

// Moved the following to memory_controller.h so that they are visible
// from the scheduler.
long long int *committed;
long long int *fetched;
long long int *time_done;
long long int total_time_done;
float core_power = 0;

int usimm_setup(char* config_filename,
                char* usimm_vi_file)
{
    printf("Initializing usimm memory module.\n");

    printf("---------------------------------------------\n");
    printf("-- USIMM: the Utah SImulated Memory Module --\n");
    printf("--              Version: 1.3               --\n");
    printf("---------------------------------------------\n");

    // DK: unused
    //int num_fetch=0;
    //int num_done=0;
    //int numch=0;
    //int writeqfull=0;
    //long long int maxtd;
    //int maxcr;
    //char newstr[MAXTRACELINESIZE];

    max_write_queue_length[0] = 0;
    max_write_queue_length[1] = 0;
    max_write_queue_length[2] = 0;
    max_write_queue_length[3] = 0;

    max_read_queue_length[0] = 0;
    max_read_queue_length[1] = 0;
    max_read_queue_length[2] = 0;
    max_read_queue_length[3] = 0;

    accumulated_read_queue_length[0] = 0;
    accumulated_read_queue_length[1] = 0;
    accumulated_read_queue_length[2] = 0;
    accumulated_read_queue_length[3] = 0;

    //update_mem_count = 0;
    //schedule_count = 0;

    memset(total_col_reads,        0, sizeof(long long int) * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS);
    memset(total_pre_cmds,         0, sizeof(long long int) * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS);
    memset(total_single_col_reads, 0, sizeof(long long int) * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS);
    memset(current_col_reads,      0, sizeof(long long int) * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS);

    //int fnstart;
    int currMTapp;
    int pow_of_2_cores;
    int *nonmemops;
    char *opertype;
    long long int *addr;
    long long int *instrpc;

    config_file = fopen(config_filename, "r");
    if (!config_file)
    {
        printf("Usimm: Can't open configuration file %s.  Quitting.\n", config_filename);
        return -4;
    }

    //DK
    //  NUMCORES = argc-2;
    NUMCORES = 1;

    ROB         = (struct robstructure *)malloc(sizeof(struct robstructure)*NUMCORES);
    tif         = (FILE **)malloc(sizeof(FILE *)*NUMCORES);
    committed   = (long long int *)malloc(sizeof(long long int)*NUMCORES);
    fetched     = (long long int *)malloc(sizeof(long long int)*NUMCORES);
    time_done   = (long long int *)malloc(sizeof(long long int)*NUMCORES);
    nonmemops   = (int *)malloc(sizeof(int)*NUMCORES);
    opertype    = (char *)malloc(sizeof(char)*NUMCORES);
    addr        = (long long int *)malloc(sizeof(long long int)*NUMCORES);
    instrpc     = (long long int *)malloc(sizeof(long long int)*NUMCORES);
    prefixtable = (int *)malloc(sizeof(int)*NUMCORES);
    currMTapp   = -1;

    for (numc = 0; numc < NUMCORES; numc++)
    {
        // DK Taking this out, seems to be opening trace files
        /*
         tif[numc] = fopen(argv[numc+2], "r");
         if (!tif[numc]) {
         printf("Missing input trace file %d.  Quitting. \n",numc);
         return -5;
         }
         */

        /* The addresses in each trace are given a prefix that equals
           their core ID.  If the input trace starts with "MT", it is
           assumed to be part of a multi-threaded app.  The addresses
           from this trace file are given a prefix that equals that of
           the last seen input trace file that starts with "MT0".  For
           example, the following is an acceptable set of inputs for
           multi-threaded apps CG (4 threads) and LU (2 threads):
           usimm 1channel.cfg MT0CG MT1CG MT2CG MT3CG MT0LU MT1LU */
        prefixtable[numc] = numc;

        // DK Taking this out, seems to be dealing with trace files
        /* Find the start of the filename.  It's after the last "/". */
        /*
        for (fnstart = strlen(argv[numc+2]) ; fnstart >= 0; fnstart--) {
        if (argv[numc+2][fnstart] == '/') {
        break;
        }
        }
        */

        //fnstart++;  /* fnstart is either the letter after the last / or the 0th letter. */


        // DK Taking this out, seems to be dealing with trace files
        /*
        if ((strlen(argv[numc+2])-fnstart) > 2) {
        if ((argv[numc+2][fnstart+0] == 'M') && (argv[numc+2][fnstart+1] == 'T')) {
        if (argv[numc+2][fnstart+2] == '0') {
        currMTapp = numc;
        }
        else {
        if (currMTapp < 0) {
        printf("Poor set of input parameters.  Input file %s starts with \"MT\", but there is no preceding input file starting with \"MT0\".  Quitting.\n", argv[numc+2]);
        return -6;
        }
        else
        prefixtable[numc] = currMTapp;
        }
        }
        }
        */

        // DK Taking this out, seems to be dealing with trace files
        //printf("Core %d: Input trace file %s : Addresses will have prefix %d\n", numc, argv[numc+2], prefixtable[numc]);

        committed[numc]     = 0;
        fetched[numc]       = 0;
        time_done[numc]     = 0;
        ROB[numc].head      = 0;
        ROB[numc].tail      = 0;
        ROB[numc].inflight  = 0;
        ROB[numc].tracedone = 0;
    }

    read_config_file(config_file);

    //TODO: Get rid of this "switch" statement, and just leave the .vi file up to the user.

    // Try reading passed in vi file first
/*    if (usimm_vi_file != NULL)
    {
        vi_file = fopen(usimm_vi_file, "r");
        chips_per_rank = 16;
        printf("Reading vi file: %s\t\n%d Chips per Rank\n", usimm_vi_file, chips_per_rank);
    }
    else
    {
    // Find the appropriate .vi file to read
    if (NUM_CHANNELS == 1 && NUMCORES == 1) {
    vi_file = fopen("../samples/configs/usimm_configs/1Gb_x4.vi", "r");
    chips_per_rank= 16;
    printf("Reading vi file: 1Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 1 && NUMCORES == 2) {
    vi_file = fopen("../samples/configs/usimm_configs/2Gb_x4.vi", "r");
    chips_per_rank= 16;
    printf("Reading vi file: 2Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 1 && (NUMCORES > 2) && (NUMCORES <= 4)) {
    vi_file = fopen("../samples/configs/usimm_configs/4Gb_x4.vi", "r");
    chips_per_rank= 16;
    printf("Reading vi file: 4Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 4 && NUMCORES == 1) {
    vi_file = fopen("../samples/configs/usimm_configs/1Gb_x16.vi", "r");
    chips_per_rank= 4;
    printf("Reading vi file: 1Gb_x16.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 4 && NUMCORES == 2) {
    vi_file = fopen("../samples/configs/usimm_configs/1Gb_x8.vi", "r");
    chips_per_rank= 8;
    printf("Reading vi file: 1Gb_x8.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 4 && (NUMCORES > 2) && (NUMCORES <= 4)) {
    vi_file = fopen("../samples/configs/usimm_configs/2Gb_x8.vi", "r");
    chips_per_rank= 8;
    printf("Reading vi file: 2Gb_x8.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 4 && (NUMCORES > 4) && (NUMCORES <= 8)) {
    vi_file = fopen("../samples/configs/usimm_configs/4Gb_x8.vi", "r");
    chips_per_rank= 8;
    printf("Reading vi file: 4Gb_x8.vi\t\n%d Chips per Rank\n",chips_per_rank);
    } else if (NUM_CHANNELS == 4 && (NUMCORES > 8) && (NUMCORES <= 16)) {
    vi_file = fopen("../samples/configs/usimm_configs/4Gb_x4.vi", "r");
    chips_per_rank= 16;
    printf("Reading vi file: 4Gb_x4.vi\t\n%d Chips per Rank\n",chips_per_rank);
    }
    else
    */

    //if (NUM_CHANNELS > 4) {
    if (NUM_CHANNELS >= 4)
    {
        chips_per_rank = 16;
        if (usimm_vi_file != NULL)
        {
            vi_file = fopen(usimm_vi_file, "r");
            printf("Reading vi file: %s\t\n%d Chips per Rank\n", usimm_vi_file, chips_per_rank);
        }
        else
        {
            vi_file = fopen((char*)REL_PATH_BIN_TO_SAMPLES"samples/configs/usimm_configs/1Gb_x16_amd2GHz.vi", "r");
            printf("Reading vi file: 1Gb_x16_amd2GHz.vi\t\n%d Chips per Rank\n", chips_per_rank);
        }
    }
    else
    {
        printf("PANIC:: Channel - Core configuration not supported\n");
        assert(-1);
    }

    if (!vi_file)
    {
        printf("Missing DRAM chip parameter file. Quitting. \n");
        printf("Use --vi-file <path> to specify the full path to 1Gb_x16_amd2GHz.vi (should be in samples/configs/usimm_configs), or any other valid .vi file.\n");
        return -5;
    }


    assert((log_base2(NUM_CHANNELS) +
            log_base2(NUM_RANKS) +
            log_base2(NUM_BANKS) +
            log_base2(NUM_ROWS) +
            log_base2(NUM_COLUMNS) +
            log_base2(CACHE_LINE_SIZE)) == (unsigned)ADDRESS_BITS);

    // Increase the address space and rows per bank depending on the number of input traces
    ADDRESS_BITS = ADDRESS_BITS + log_base2(NUMCORES);
    if (NUMCORES == 1)
    {
        pow_of_2_cores = 1;
    }
    else
    {
        pow_of_2_cores = 1 << ((int)log_base2(NUMCORES - 1) + 1);
    }
    NUM_ROWS = NUM_ROWS * pow_of_2_cores;

    read_config_file(vi_file);
    if (arches_verbosity)
    {
        print_params();
    }

    for (int i = 0; (uint32_t)i < NUMCORES; i++)
    {
        ROB[i].comptime    = (long long int*)malloc(sizeof(long long int)*ROBSIZE);
        ROB[i].mem_address = (long long int*)malloc(sizeof(long long int)*ROBSIZE);
        ROB[i].instrpc     = (long long int*)malloc(sizeof(long long int)*ROBSIZE);
        ROB[i].optype      = (int*)malloc(sizeof(int)*ROBSIZE);
    }
    init_memory_controller_vars();
    init_scheduler_vars();

    // stats
    usimmUsageStats.Initialize();

    // Done initializing


    //DK: can ignore this for trax (not trace based)

    /* Must start by reading one line of each trace file. */

    /*
      for(numc=0; numc<NUMCORES; numc++)
      {
      if (fgets(newstr,MAXTRACELINESIZE,tif[numc])) {
      if (sscanf(newstr,"%d %c",&nonmemops[numc],&opertype[numc]) > 0) {
      if (opertype[numc] == 'R') {
      if (sscanf(newstr,"%d %c %Lx %Lx",&nonmemops[numc],&opertype[numc],&addr[numc],&instrpc[numc]) < 1) {
      printf("Panic.  Poor trace format.\n");
      return -4;
      }
      }
      else {
      if (opertype[numc] == 'W') {
      if (sscanf(newstr,"%d %c %Lx",&nonmemops[numc],&opertype[numc],&addr[numc]) < 1) {
      printf("Panic.  Poor trace format.\n");
      return -3;
      }
      }
      else {
      printf("Panic.  Poor trace format.\n");
      return -2;
      }
      }
      }
      else {
      printf("Panic.  Poor trace format.\n");
      return -1;
      }
      }
      else {
      if (ROB[numc].inflight == 0) {
      num_done++;
      if (!time_done[numc]) time_done[numc] = 1;
      }
      ROB[numc].tracedone=1;
      }
      }
      */


    // DK: The rest of this is all running the simulation based on traces. In our case, TRaX will drive usimm

#if 0
    printf("Starting simulation.\n");
    while (!expt_done) {

        /* For each core, retire instructions if they have finished. */
        for (numc = 0; numc < NUMCORES; numc++) {
            num_ret = 0;
            while ((num_ret < MAX_RETIRE) && ROB[numc].inflight) {
                /* Keep retiring until retire width is consumed or ROB is empty. */
                if (ROB[numc].comptime[ROB[numc].head] < CYCLE_VAL) {  
                    /* Keep retiring instructions if they are done. */
                    ROB[numc].head = (ROB[numc].head + 1) % ROBSIZE;
                    ROB[numc].inflight--;
                    committed[numc]++;
                    num_ret++;
                }
                else  /* Instruction not complete.  Stop retirement for this core. */
                    break;
            }  /* End of while loop that is retiring instruction for one core. */
        }  /* End of for loop that is retiring instructions for all cores. */


        if(CYCLE_VAL % PROCESSOR_CLK_MULTIPLIER == 0)
        { 
            /* Execute function to find ready instructions. */
            update_memory();

            /* Execute user-provided function to select ready instructions for issue. */
            /* Based on this selection, update DRAM data structures and set 
           instruction completion times. */
            for(int c=0; c < NUM_CHANNELS; c++)
            {
                schedule(c);
                gather_stats(c);	
            }
        }

        /* For each core, bring in new instructions from the trace file to
           fill up the ROB. */
        num_done = 0;
        writeqfull =0;
        for(int c=0; c<NUM_CHANNELS; c++){
            if(write_queue_length[c] == WQ_CAPACITY)
            {
                writeqfull = 1;
                break;
            }
        }

        for (numc = 0; numc < NUMCORES; numc++) {
            if (!ROB[numc].tracedone) { /* Try to fetch if EOF has not been encountered. */
                num_fetch = 0;
                while ((num_fetch < MAX_FETCH) && (ROB[numc].inflight != ROBSIZE) && (!writeqfull)) {
                    /* Keep fetching until fetch width or ROB capacity or WriteQ are fully consumed. */
                    /* Read the corresponding trace file and populate the tail of the ROB data structure. */
                    /* If Memop, then populate read/write queue.  Set up completion time. */

                    if (nonmemops[numc]) {  /* Have some non-memory-ops to consume. */
                        ROB[numc].optype[ROB[numc].tail] = 'N';
                        ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL+PIPELINEDEPTH;
                        nonmemops[numc]--;
                        ROB[numc].tail = (ROB[numc].tail +1) % ROBSIZE;
                        ROB[numc].inflight++;
                        fetched[numc]++;
                        num_fetch++;
                    }
                    else { /* Done consuming non-memory-ops.  Must now consume the memory rd or wr. */
                        if (opertype[numc] == 'R') {
                            addr[numc] = addr[numc] + (long long int)((long long int)prefixtable[numc] << (ADDRESS_BITS - log_base2(NUMCORES)));    // Add MSB bits so each trace accesses a different address space.
                            ROB[numc].mem_address[ROB[numc].tail] = addr[numc];
                            ROB[numc].optype[ROB[numc].tail] = opertype[numc];
                            ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL + BIGNUM;
                            ROB[numc].instrpc[ROB[numc].tail] = instrpc[numc];

                            // Check to see if the read is for buffered data in write queue - 
                            // return constant latency if match in WQ
                            // add in read queue otherwise
                            int lat = read_matches_write_or_read_queue(addr[numc]);
                            if(lat) {
                                ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL+lat+PIPELINEDEPTH;
                            }
                            else {
                                // DK call these functions
                                insert_read(addr[numc], CYCLE_VAL, numc, ROB[numc].tail, instrpc[numc]);
                            }
                        }
                        else {  /* This must be a 'W'.  We are confirming that while reading the trace. */
                            if (opertype[numc] == 'W') {
                                addr[numc] = addr[numc] + (long long int)((long long int)prefixtable[numc] << (ADDRESS_BITS - log_base2(NUMCORES)));    // Add MSB bits so each trace accesses a different address space.
                                ROB[numc].mem_address[ROB[numc].tail] = addr[numc];
                                ROB[numc].optype[ROB[numc].tail] = opertype[numc];
                                ROB[numc].comptime[ROB[numc].tail] = CYCLE_VAL+PIPELINEDEPTH;
                                /* Also, add this to the write queue. */

                                if(!write_exists_in_write_queue(addr[numc]))
                                    // DK call these functions
                                    // address, current cycle, which core, 
                                    // see insert_read: /uusoc/res/arch/students/nil/simics_workspace/osprey_wrkspcs/simics_usimm_latest/modules/trans-staller


                                    // Modify for my purposes, send in which instruction issued
                                    insert_write(addr[numc], CYCLE_VAL, numc, ROB[numc].tail);

                                for(int c=0; c<NUM_CHANNELS; c++){
                                    if(write_queue_length[c] == WQ_CAPACITY)
                                    {
                                        writeqfull = 1;
                                        break;
                                    }
                                }
                            }
                            else {
                                printf("Panic.  Poor trace format. \n");
                                return -1;
                            }
                        }
                        ROB[numc].tail = (ROB[numc].tail +1) % ROBSIZE;
                        ROB[numc].inflight++;
                        fetched[numc]++;
                        num_fetch++;

                        /* Done consuming one line of the trace file.  Read in the next. */
                        if (fgets(newstr,MAXTRACELINESIZE,tif[numc])) {
                            if (sscanf(newstr,"%d %c",&nonmemops[numc],&opertype[numc]) > 0) {
                                if (opertype[numc] == 'R') {
                                    if (sscanf(newstr,"%d %c %Lx %Lx",&nonmemops[numc],&opertype[numc],&addr[numc],&instrpc[numc]) < 1) {
                                        printf("Panic.  Poor trace format.\n");
                                        return -4;
                                    }
                                }
                                else {
                                    if (opertype[numc] == 'W') {
                                        if (sscanf(newstr,"%d %c %Lx",&nonmemops[numc],&opertype[numc],&addr[numc]) < 1) {
                                            printf("Panic.  Poor trace format.\n");
                                            return -3;
                                        }
                                    }
                                    else {
                                        printf("Panic.  Poor trace format.\n");
                                        return -2;
                                    }
                                }
                            }
                            else {
                                printf("Panic.  Poor trace format.\n");
                                return -1;
                            }
                        }
                        else {
                            if (ROB[numc].inflight == 0) {
                                num_done++;
                                if (!time_done[numc]) time_done[numc] = CYCLE_VAL;
                            }
                            ROB[numc].tracedone=1;
                            break;  /* Break out of the while loop fetching instructions. */
                        }

                    }  /* Done consuming the next rd or wr. */

                } /* One iteration of the fetch while loop done. */
            } /* Closing brace for if(trace not done). */
            else { /* Input trace is done.  Check to see if all inflight instrs have finished. */
                if (ROB[numc].inflight == 0) {
                    num_done++;
                    if (!time_done[numc]) time_done[numc] = CYCLE_VAL;
                }
            }
        } /* End of for loop that goes through all cores. */


        if (num_done == NUMCORES) {
            /* Traces have been consumed and in-flight windows are empty.  Must confirm that write queues have been drained. */
            for (numch=0;numch<NUM_CHANNELS;numch++) {
                if (write_queue_length[numch]) break;
            }
            if (numch == NUM_CHANNELS) expt_done=1;  /* All traces have been consumed and the write queues are drained. */
        }

        /* Printing details for testing.  Remove later. */
        //printf("Cycle: %lld\n", CYCLE_VAL);
        //for (numc=0; numc < NUMCORES; numc++) {
        // printf("C%d: Inf %d : Hd %d : Tl %d : Comp %lld : type %c : addr %x : TD %d\n", numc, ROB[numc].inflight, ROB[numc].head, ROB[numc].tail, ROB[numc].comptime[ROB[numc].head], ROB[numc].optype[ROB[numc].head], ROB[numc].mem_address[ROB[numc].head], ROB[numc].tracedone);
        //}

        CYCLE_VAL++;  /* Advance the simulation cycle. */
    }


    /* Code to make sure that the write queue drain time is included in
       the execution time of the thread that finishes last. */
    maxtd = time_done[0];
    maxcr = 0;
    for (numc=1; numc < NUMCORES; numc++) {
        if (time_done[numc] > maxtd) {
            maxtd = time_done[numc];
            maxcr = numc;
        }
    }
    time_done[maxcr] = CYCLE_VAL;

    core_power = 0;
    for (numc=0; numc < NUMCORES; numc++) {
        /* A core has peak power of 10 W in a 4-channel config.  Peak power is consumed while the thread is running, else the core is perfectly power gated. */
        core_power = core_power + (10*((float)time_done[numc]/(float)CYCLE_VAL));
    }
    if (NUM_CHANNELS == 1) {
        /* The core is more energy-efficient in our single-channel configuration. */
        core_power = core_power/2.0 ;
    }

    // DK: End of usimm trace-based simulation
    //     Put this stat-printing in a function in TRaX's main.cc
#endif

    // Clean up
    free(nonmemops);
    free(opertype);
    free(addr);
    free(instrpc);

    // DK to close up the setup function, printstats will be below
    return 0;
}


float getUsimmPower()
{
    float total_system_power = 0;
    for (int c = 0; c < NUM_CHANNELS; ++c)
    {
        for (int r = 0; r < NUM_RANKS; ++r)
        {
            total_system_power += calculate_power(c, r, 1, chips_per_rank, false);
        }
    }
    return total_system_power;
}


void printUsimmStats(uint32_t const L2_line_size,
                     uint32_t const word_size,
                      Arches::cycles_t cycle_count)
{
    printf("-------------DRAM stats-------------\n");
    printf("Cycles %lld\n", CYCLE_VAL);
    total_time_done = 0;
    if (arches_verbosity)
    {
        for (numc = 0; numc < NUMCORES; ++numc)
        {
            printf("Done: Core %d: Fetched %lld : Committed %lld : At time  : %lld\n",
                   numc,
                   fetched[numc],
                   committed[numc],
                   time_done[numc]);

            total_time_done += time_done[numc];
        }

        printf("Sum of execution times for all programs: %lld\n", total_time_done);
        printf("Num reads merged:                        %lld\n", num_read_merge);
        printf("Num writes merged:                       %lld\n", num_write_merge);
    }

    // Print Access stats
    /*
    printf("\n#---------------------------------- OpCode Access Stats ------------------------------------------\n");
    typedef std::map<Instruction::Opcode, unsigned long long int> opCountMap_t;

    opCountMap_t const &loadOccRef    = usimmUsageStats.statsCacheLineLoads.GetOccurences();
    unsigned long long int totalLoads = 0;
    for (opCountMap_t::const_iterator lIter = loadOccRef.begin(); lIter != loadOccRef.end(); ++lIter)
    {
        totalLoads += lIter->second;
    }
    printf("Cache lines used by Loads: (Total %llu, %f GB/s at 1000MHz)\n",
           totalLoads,
           static_cast<float>(totalLoads) * L2_line_size * word_size / cycle_count);
    printf("  op\tcount\t%% of total\tbandwidth at 1000MHz (GB/s)\n");
    for (opCountMap_t::const_iterator lIter = loadOccRef.begin(); lIter != loadOccRef.end(); ++lIter)
    {
        printf("  %s\t%llu\t(%2.4f %%)\t%f\n",
               Instruction::Opnames[lIter->first].c_str(),
               lIter->second,
               static_cast<float>(lIter->second) / totalLoads * 100.f,
               static_cast<float>(lIter->second) * L2_line_size * word_size / cycle_count);
    }
    printf("  -Total:      %llu\n", totalLoads);
    printf("  -Duplicated: %llu\n", usimmUsageStats.numCacheLineLoadsDups);
    printf("  -Net:        %llu\n", (totalLoads - usimmUsageStats.numCacheLineLoadsDups));
    printf("\n");

    opCountMap_t const &storeOccRef    = usimmUsageStats.statsCacheLineStores.GetOccurences();
    unsigned long long int totalStores = 0;
    for (opCountMap_t::const_iterator sIter = storeOccRef.begin(); sIter != storeOccRef.end(); ++sIter)
    {
        totalStores += sIter->second;
    }
    printf("Cache lines used by Stores: (Total %llu, %f GB/s at 1000MHz)\n",
           totalStores,
           static_cast<float>(totalStores) * L2_line_size * word_size / cycle_count);
    printf("  op\tcount\t%% of total\tbandwidth at 1000MHz (GB/s)\n");
    for (opCountMap_t::const_iterator sIter = storeOccRef.begin(); sIter != storeOccRef.end(); ++sIter)
    {
        printf("  %s\t%llu\t(%2.4f %%)\t%f\n",
               Instruction::Opnames[sIter->first].c_str(),
               sIter->second,
               static_cast<float>(sIter->second) / totalStores * 100.f,
               static_cast<float>(sIter->second) * L2_line_size * word_size / cycle_count);
    }
    printf("  -Total:      %llu\n", totalStores);
    printf("  -Duplicated: %llu\n", usimmUsageStats.numCacheLineStoresDups);
    printf("  -Net:        %llu\n", (totalStores - usimmUsageStats.numCacheLineStoresDups));
    printf("\n");
    */
    // Print all other memory system stats.
    scheduler_stats();
    print_stats();

    // Print Cycle Stats
    for (int c = 0; c < NUM_CHANNELS; ++c)
    {
        for (int r = 0; r < NUM_RANKS; ++r)
        {
            calculate_power(c, r, 0, chips_per_rank);
        }
    }

    if (arches_verbosity)
    {
        printf("\n#-------------------------------------- Power Stats ----------------------------------------------\n");
        printf("Note:  1. termRoth/termWoth is the power dissipated in the ODT resistors when Read/Writes terminate \n");
        printf("          in other ranks on the same channel\n");
        printf("#-------------------------------------------------------------------------------------------------\n\n");
    }

    /*Print Power Stats*/
    float total_system_power = 0;
    for (int c = 0; c < NUM_CHANNELS; ++c)
    {
        for (int r = 0; r < NUM_RANKS; ++r)
        {
            total_system_power += calculate_power(c, r, 1, chips_per_rank);
        }
    }

    printf("\n#-------------------------------------------------------------------------------------------------\n");
    if (NUM_CHANNELS == 4)
    {
        printf("Total memory system power  = %f W\n", total_system_power / 1000);
        printf("Miscellaneous system power = 40 W  # Processor uncore power, disk, I/O, cooling, etc.\n");
        printf("Processor core power       = %f W  # Assuming that each core consumes 10 W when running\n",core_power);
        printf("Total system power         = %f W # Sum of the previous three lines\n", 40 + core_power + total_system_power/1000);
        printf("Energy Delay product (EDP) = %2.9f J.s\n", (40 + core_power + total_system_power/1000)*(float)((double)CYCLE_VAL/(double)3200000000) * (float)((double)CYCLE_VAL/(double)3200000000));
    }
    else
    {
        printf("Total memory system power  = %f W\n", total_system_power / 1000);
        printf("Miscellaneous system power = 10 W  # Processor uncore power, disk, I/O, cooling, etc.\n");      // The total 40 W misc power will be split across 4 channels, only 1 of which is being considered in the 1-channel experiment.
        printf("Processor core power       = %f W  # Assuming that each core consumes 5 W\n",core_power);       // Assuming that the cores are more lightweight.
        printf("Total system power         = %f W # Sum of the previous three lines\n", 10 + core_power + total_system_power/1000);
        printf("Energy Delay product (EDP) = %2.9f J.s\n", (10 + core_power + total_system_power/1000)*(float)((double)CYCLE_VAL/(double)3200000000) * (float)((double)CYCLE_VAL/(double)3200000000));
    }
}


// Call this function once per TRaX global cycle
void usimmClock()
{
#if 0
    for (int c = 0; c < NUM_CHANNELS; ++c)
    {
        if(write_queue_length[c] == WQ_CAPACITY)
        {
            writeqfull = 1;
            break;
        }

        printf("WQ[%d] size: %d\n", c, write_queue_length[c]);
    }

    // For each core, retire instructions if they have finished.
    for (numc = 0; numc < NUMCORES; ++numc)
    {
        num_ret = 0;
        while ((num_ret < MAX_RETIRE) && ROB[numc].inflight)
        {
            // Keep retiring until retire width is consumed or ROB is empty.
            if (ROB[numc].comptime[ROB[numc].head] < CYCLE_VAL)
            {
                // Keep retiring instructions if they are done.
                ROB[numc].head = (ROB[numc].head + 1) % ROBSIZE;
                ROB[numc].inflight--;
                committed[numc]++;
                num_ret++;
            }
            // Instruction not complete.  Stop retirement for this core.
            else
                break;
        }   // End of while loop that is retiring instruction for one core.
    }       // End of for loop that is retiring instructions for all cores.
#endif

    // Execute function to find ready instructions.
    update_memory();

    // Execute user-provided function to select ready instructions for issue.
    // Based on this selection, update DRAM data structures and set instruction completion times.
    for (int c = 0; c < NUM_CHANNELS; ++c)
    {
        schedule(c);
        gather_stats(c);
    }
    CYCLE_VAL++;
}


bool usimmIsBusy()
{
    for (int channel = 0; channel < NUM_CHANNELS; ++channel)
    {
        if (read_queue_head[channel].size() > 0)
        {
            return true;
        }

        if (write_queue_head[channel].size() > 0)
        {
            return true;
        }
    }

    return false;
}


void usimmDestroy()
{
    for (int i = 0; i < (int32_t)NUMCORES; i++)
    {
        free(ROB[i].comptime);
        free(ROB[i].mem_address);
        free(ROB[i].instrpc);
        free(ROB[i].optype);
    }
    free(ROB);
    free(tif);
    free(committed);
    free(fetched);
    free(time_done);
    free(prefixtable);
}
