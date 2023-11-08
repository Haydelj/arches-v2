#include <stdio.h>
#include <string.h>
#include "utils.h"

#include "memory_controller.h"
#include "params.h"

extern Arches::cycles_t CYCLE_VAL;

int BANK_CAN_BE_CLOSED[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int schedule_count;


void init_scheduler_vars()
{
    // initialize all scheduler variables here
    for (int i = 0; i < MAX_NUM_CHANNELS; ++i)
    {
        for (int j = 0; j < MAX_NUM_RANKS; ++j)
        {
            for (int k = 0; k < MAX_NUM_BANKS; ++k)
            {
                BANK_CAN_BE_CLOSED[i][j][k] = 0;
            }
        }
    }
    return;
}


// write queue high water mark; begin draining writes if write queue exceeds this value
#define HI_WM 40

// end write queue drain once write queue has this many writes in it
#define LO_WM 20

// 1 means we are in write-drain mode for that channel
int drain_writes[MAX_NUM_CHANNELS];


/* Each cycle it is possible to issue a valid command from the read or write queues
   OR
   a valid precharge command to any bank (issue_precharge_command())
   OR
   a valid precharge_all bank command to a rank (issue_all_bank_precharge_command())
   OR
   a power_down command (issue_powerdown_command()), programmed either for fast or slow exit mode
   OR
   a refresh command (issue_refresh_command())
   OR
   a power_up command (issue_powerup_command())
   OR
   an activate to a specific row (issue_activate_command()).

   If a COL-RD or COL-WR is picked for issue, the scheduler also has the
   option to issue an auto-precharge in this cycle (issue_autoprecharge()).

   Before issuing a command it is important to check if it is issuable. For the RD/WR queue resident commands,
   checking the "command_issuable" flag is necessary. To check if the other commands (mentioned above) can be
   issued, it is important to check one of the following functions: is_precharge_allowed,
   is_all_bank_precharge_allowed, is_powerdown_fast_allowed, is_powerdown_slow_allowed, is_powerup_allowed,
   is_refresh_allowed, is_autoprecharge_allowed, is_activate_allowed.
*/

void schedule(int channel)
{
    schedule_count++;

#define Priority_factor  1

    // we need to initialize the scheduler's variable
    //if (Debug == 1)
    //      print_read_queue();

    memset(BANK_CAN_BE_CLOSED[channel], 0, sizeof(int) * MAX_NUM_RANKS * MAX_NUM_BANKS);

    // if in write drain mode, keep draining writes until the
    // write queue occupancy drops to LO_WM
    if (drain_writes[channel] && (write_queue_length[channel] > LO_WM))
    {
        drain_writes[channel] = 1; // Keep draining.
    }
    else
    {
        drain_writes[channel] = 0; // No need to drain.
    }

    // initiate write drain if either the write queue occupancy
    // has reached the HI_WM , OR, if there are no pending read
    // requests
    if (write_queue_length[channel] > HI_WM)
    {
        drain_writes[channel] = 1;
    }
    else
    {
        if (!read_queue_length[channel])
            drain_writes[channel] = 1;
    }

    // If in write drain mode, look through all the write queue
    // elements (already arranged in the order of arrival), and
    // issue the command for the first request that is ready
    if (/*SCHEDULING ==*/ 1)  // Apply FR_FCFS
    {
        //printf("FR-FCFS \n");
        const int choose = 0;  // 1  => Meysam  0 => Ali
        if (choose == 1)
        {
            if (drain_writes[channel])
            {
                /// printf("enter to write queue \n");
                std::list<request_t> &queueRef      = write_queue_head[channel];
                std::list<request_t>::iterator iter = queueRef.begin();
                for (; iter != queueRef.end(); ++iter)
                {
                    const bool issuable = (iter->dram_addr.row == dram_state[channel][iter->dram_addr.rank][iter->dram_addr.bank].active_row);
                    BANK_CAN_BE_CLOSED[channel][iter->dram_addr.rank][iter->dram_addr.bank] = (issuable ? 0 : 1);
                }

                // we check all requests to find which one has the issuable Col_Write_CMD for the current open row
                //now if we could not find this command we need to do according to FCFS but we are awared of this fact
                // that if there is any Col_Write_CMD for the current open row but it is not issuable we need to make
                //all precharge commands hold to that unissuable command gets issuable.
                for (iter = queueRef.begin(); iter != queueRef.end(); ++iter)
                {
                    if (iter->command_issuable)
                    {
                        if (iter->next_command == PRE_CMD)
                        {
                            if (BANK_CAN_BE_CLOSED[channel][iter->dram_addr.rank][iter->dram_addr.bank])
                            {
                                issue_request_command(&(*iter));
                                break;
                            }
                        }
                        else
                        {
                            issue_request_command(&(*iter));
                            break;
                        }
                    }
                }
            }
        } // end choose == 1

        else
        {
            if (drain_writes[channel])
            {
                std::list<request_t> &queueRef      = write_queue_head[channel];
                std::list<request_t>::iterator iter = queueRef.begin();
                for (; iter != queueRef.end(); ++iter)
                {
                    if (iter->command_issuable)
                    {
                        if (iter->next_command == PRE_CMD)
                        {
                            bool row_is_still_needed             = false;
                            std::list<request_t>::iterator iter2 = queueRef.begin();
                            for (; iter2 != queueRef.end(); ++iter2)
                            {
                                if ((iter2->dram_addr.row == dram_state[channel][iter->dram_addr.rank][iter->dram_addr.bank].active_row) &&
                                    (iter2->next_command == COL_WRITE_CMD))
                                {
                                    row_is_still_needed = true;
                                }
                            }
                            if (!row_is_still_needed)
                            {
                                issue_request_command(&(*iter));
                                break;
                            }
                        }
                        else
                        {
                            issue_request_command(&(*iter));
                            break;
                        }
                    }
                } // end iter for
            }

            // Draining Reads
            // look through the queue and find the first request whose
            // command can be issued in this cycle and issue it
            // Simple FCFS

            // ali
            if (!drain_writes[channel])
            {
                std::list<request_t> &queueRef      = read_queue_head[channel];
                std::list<request_t>::iterator iter = queueRef.begin();
                for (; iter != queueRef.end(); ++iter)
                {
                    if (iter->command_issuable)
                    {
                        // TODO: take this out
                        //issue_request_command(rd_ptr);
                        //break;

                        //if(rd_ptr->
                        //if(really old)
                        //issue and break;

                        if (iter->next_command == PRE_CMD)
                        {
                            bool row_is_still_needed             = false;
                            std::list<request_t>::iterator iter2 = queueRef.begin();
                            for (; iter2 != queueRef.end(); ++iter2)
                            {
                                if ((iter2->dram_addr.row == dram_state[channel][iter->dram_addr.rank][iter->dram_addr.bank].active_row) &&
                                    (iter2->next_command == COL_READ_CMD))
                                {
                                    row_is_still_needed = true;
                                }
                            }
                            if (!row_is_still_needed)
                            {
                                issue_request_command(&(*iter));
                                break;
                            }
                        }
                        else
                        {
                            issue_request_command(&(*iter));
                            break;
                        }
                    }
                } // end iter for
            }
        } //else choose
        // end ali

        if (choose == 1)
        {
            if (!drain_writes[channel])
            {
                std::list<request_t> &queueRef      = read_queue_head[channel];
                std::list<request_t>::iterator iter = queueRef.begin();
                for (; iter != queueRef.end(); ++iter)
                {
                    const bool issuable = (iter->dram_addr.row == dram_state[channel][iter->dram_addr.rank][iter->dram_addr.bank].active_row);
                    BANK_CAN_BE_CLOSED[channel][iter->dram_addr.rank][iter->dram_addr.bank] = (issuable ? 0 : 1);
                }

                // we check all requests to find which one has the issuable Col_read_CMD for the current open row 
                //now if we could not find this command we need to do according to FCFS but we are awared of this fact
                // that if there is any Col_Read_CMD for the current open row but it is not issuable we need to make 
                //all precharge commands hold to that unissuable command gets issuable.
                for (iter = queueRef.begin(); iter != queueRef.end(); ++iter)
                {
                    if (iter->command_issuable)
                    {
                        // to add fairness to FR-FCFS and prevent from starvation 
                        // hereby we check the latency of rquest resident in read queue 
                        // to find any request which is waited for more than Priority_factor
                        // times average latency of read queue.
                        if (iter->next_command == PRE_CMD)
                        {
                            if (BANK_CAN_BE_CLOSED[channel][iter->dram_addr.rank][iter->dram_addr.bank])
                            {
                                issue_request_command(&(*iter));
                                break;
                            }
                        }
                        else
                        {
                            issue_request_command(&(*iter));
                            break;
                        }
                    }
                }
            }
        } // end choose == 1
    } // FR_FCFS

#if 0
    else if (SCHEDULING == 0) //Apply FCFS
    {
        //printf("FCFS \n");
        if(drain_writes[channel])
        {
            LL_FOREACH(write_queue_head[channel], wr_ptr)
            {
                if(wr_ptr->command_issuable)
                {
                    issue_request_command(wr_ptr);
                    break;
                }
            }

            return;
        }

        // Draining Reads
        // look through the queue and find the first request whose
        // command can be issued in this cycle and issue it
        // Simple FCFS
        if(!drain_writes[channel])
        {
            LL_FOREACH(read_queue_head[channel],rd_ptr)
            {
                if(rd_ptr->command_issuable)
                {
                    issue_request_command(rd_ptr);
                    break;
                }
            }
            return;
        }
    }
#endif
}

void scheduler_stats()
{
    // Nothing to print for now.
}
