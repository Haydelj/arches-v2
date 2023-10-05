#pragma once

#if defined __x86_64__ || defined _M_X64
#define ARCH_X86
#endif

#ifdef __riscv
#define ARCH_RISCV
#endif

#ifdef ARCH_RISCV
#define INLINE __attribute__((always_inline))
//#define INLINE inline
#endif

#ifdef ARCH_X86
#define INLINE inline
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#endif

#include <cfloat>
#include <cstdint>

#include "../../include/rtm/rtm.hpp"

typedef unsigned int uint;