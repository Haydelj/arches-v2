#ifndef __UTILS_H__
#define __UTILS_H__

// Utility functions

// Turn on the following flag to see debug messages
//#define CMD_DEBUG
//#define SCHEDULER_DEBUG

#ifdef CMD_DEBUG
#  define UT_MEM_DEBUG(...) printf(__VA_ARGS__)
#else
#  define UT_MEM_DEBUG(...)
#endif


#define SCHEDULER_DEBUG

#ifdef SCHEDULER_DEBUG
#  define SCHEDELUR_DEBUG_MSG(...) printf(__VA_ARGS__)
#else
#  define SCHEDULER_DEBUG_MSG(...)
#endif

#endif // __UTILS_H__

