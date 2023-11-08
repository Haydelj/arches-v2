#ifndef USIMM_H_
#define USIMM_H_

#include "stdafx.hpp"

//#ifndef REL_PATH_BIN_TO_SAMPLES
//#  define REL_PATH_BIN_TO_SAMPLES "../../config-files/usimm/"
//#endif

#ifndef REL_PATH_BIN_TO_SAMPLES
#  define REL_PATH_BIN_TO_SAMPLES "./config-files/"
#endif

int usimm_setup(char* config_filename, char* usimm_vi_file);
float getUsimmPower();
void usimmClock();
bool usimmIsBusy();
void usimmDestroy();

void printUsimmStats(uint32_t const L2_line_size,
                     uint32_t const word_size,
                     Arches::cycles_t cycle_count);

#endif
