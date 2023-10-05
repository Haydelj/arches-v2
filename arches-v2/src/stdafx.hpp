#pragma once

//Determine debug/release
#if defined _WIN16 || defined WIN32 || defined _WIN32 || defined WIN64 || defined _WIN64 || defined __WIN32__ || defined __TOS_WIN__ || defined __WINDOWS__
	#define BUILD_PLATFORM_WINDOWS

	#if defined _DEBUG || defined DEBUG
		#define BUILD_DEBUG
	#else
		#define BUILD_RELEASE
	#endif

	//#define _CRT_SECURE_NO_WARNINGS //The std library is not deprecated

	#define unreachable __assume(0)
#else
	#ifdef NDEBUG
		#define BUILD_RELEASE
	#else
		#define BUILD_DEBUG
	#endif

	#define unreachable __builtin_unreachable()
#endif

//Detect Memmory Leaks
#ifdef _DEBUG
	#define _CRTDBG_MAP_ALLOC
	#define _new new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
#else
	#define _new new
#endif

//Determine architecture
#if defined __amd64__ || defined __amd64 || defined __x86_64__ || defined __x86_64 || defined _M_X64 || defined _M_AMD64
	#define BUILD_ARCH_64
	#define BUILD_ARCH_x86_64
	#define BUILD_ARCH_ENDIAN_LITTLE
#elif defined i386 || defined __i386 || defined __i386__ || defined __i486__ || defined __i586__ || defined __i686__ || defined _M_IX86 || defined _X86_ || defined __X86__
	#define BUILD_ARCH_32
	#define BUILD_ARCH_x86
	#define BUILD_ARCH_ENDIAN_LITTLE
#else
	#error "Define Endianness"
#endif

//Determine compiler
#if   defined __INTEL_COMPILER || defined __ICC //Note: "__ICC" symbol deprecated
	//Note that check for ICC comes before the check for MSVC; ICC still defines "_MSC_VER".
	#define BUILD_COMPILER_INTEL
	#define BUILD_COMPILER_VERSION_MAJOR (__INTEL_COMPILER/100)
	#define BUILD_COMPILER_VERSION_MINOR (__INTEL_COMPILER%100)

	#define MESSAGE(TEXT) __pragma(message(__FILE__ "(" STRINGIFY_VALUE(__LINE__) "): " TEXT)) //TODO: correct?
	#define WARNING(TEXT) MESSAGE(TEXT)
#elif defined _MSC_VER
	#define BUILD_COMPILER_MSVC
	//major=11, minor= 0   =>   MSVC++ 5.0
	//major=12, minor= 0   =>   MSVC++ 6.0
	//major=13, minor= 0   =>   MSVC++ 7.0
	//major=13, minor=10   =>   MSVC++ 7.1  (VS 2003)
	//major=14, minor= 0   =>   MSVC++ 8.0  (VS 2005)
	//major=15, minor= 0   =>   MSVC++ 9.0  (VS 2008)
	//major=16, minor= 0   =>   MSVC++ 10.0 (VS 2010)
	//major=17, minor= 0   =>   MSVC++ 11.0 (VS 2012)
	//major=18, minor= 0   =>   MSVC++ 12.0 (VS 2013)
	//major=19, minor= 0   =>   MSVC++ 14.0 (VS 2015)
	#define BUILD_COMPILER_VERSION_MAJOR (_MSC_VER/100)
	#define BUILD_COMPILER_VERSION_MINOR (_MSC_VER%100)
	#ifdef _MSC_FULL_VER
		#define BUILD_COMPILER_VERSION_PATCH (_MSC_FULL_VER%100000)
	#endif

	#define MESSAGE(TEXT) __pragma(message(__FILE__ "(" STRINGIFY_VALUE(__LINE__) "): " TEXT))
	#define WARNING(TEXT) MESSAGE(TEXT)
#elif defined __clang__
	//Note that check for Clang comes before check for GNU; Clang defines "__GUNC__" at least some of the time.
	#define BUILD_COMPILER_CLANG
	#define BUILD_COMPILER_VERSION_MAJOR __clang_major__
	#define BUILD_COMPILER_VERSION_MINOR __clang_minor__
	#define BUILD_COMPILER_VERSION_PATCH __clang_patchlevel__

	#define IB_PRAGMA(x) _Pragma(#x)
	#define MESSAGE(TEXT) IB_PRAGMA(message TEXT)
	#define WARNING(TEXT) IB_PRAGMA(GCC warning TEXT) //Yes, "GCC".  It ignores e.g. "clang".
#elif defined __GNUC__
	#define BUILD_COMPILER_GNU
	#define BUILD_COMPILER_VERSION_MAJOR __GNUC__
	#define BUILD_COMPILER_VERSION_MINOR __GNUC_MINOR__
	#ifdef __GNUC_PATCHLEVEL__
		#define BUILD_COMPILER_VERSION_PATCH __GNUC_PATCHLEVEL__
	#endif

	#define IB_PRAGMA(x) _Pragma(#x)
	#define MESSAGE(TEXT) IB_PRAGMA(message TEXT)
	#define WARNING(TEXT) IB_PRAGMA(GCC warning TEXT)
#else
	#warning "No supported compiler detected!"
#endif

//Useful error macros
#ifdef BUILD_DEBUG
	#define implerr assert(false); throw -1
	#define nodefault default: implerr;
#else
	#define implerr unreachable
	#define nodefault default: unreachable
#endif
#define notimpl implerr

//Language facts
#if -2>>1 == -1
	#define SHIFTRIGHT_ARITHMETIC //Pads with sign bit
#else
	#define SHIFTRIGHT_LOGICAL    //Pads with 0
#endif

#define NOMINMAX

#ifndef _DEBUG
#include "oneapi/tbb.h"
#endif

#include <cassert>
#include <cstring>

#include <atomic>
#include <mutex>
#include <thread>

#include <deque>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <iostream>
#include <new>

#include <intrin.h>
#include <xmmintrin.h>

#include "../../include/rtm/rtm.hpp"

typedef unsigned int uint;

namespace Arches {

typedef uint64_t paddr_t;
typedef uint64_t vaddr_t;
typedef uint64_t addr_t;

typedef uint64_t frame_number_t;
typedef uint64_t page_number_t;

typedef uint64_t cycles_t;

typedef uint32_t uint18_t;
typedef  int32_t  int18_t;
typedef uint32_t uint28_t;
typedef  int32_t  int28_t;
typedef uint64_t uint48_t;
typedef  uint64_t  int48_t;

static_assert(sizeof(float)==4&&sizeof(double)==8,"Not Implemented!");

#define ARCHES_LOGGING

}