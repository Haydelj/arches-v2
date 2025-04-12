#pragma once

#include <cstdint>

typedef unsigned int uint;

inline float min(int a, int b) { return (b < a) ? b : a; }
inline float max(int a, int b) { return (a < b) ? b : a; }