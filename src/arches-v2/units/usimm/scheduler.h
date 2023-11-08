#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "stdafx.hpp"

void init_scheduler_vars(); // called from main
void scheduler_stats();     // called from main
void schedule(int);         // scheduler function called every cycle

extern Arches::cycles_t CYCLE_VAL;
extern long long int schedule_count;

#endif //__SCHEDULER_H__
