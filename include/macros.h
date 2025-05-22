#ifndef MACROS_H
#define MACROS_H

#include <types.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))

#define COUNT_OF(x)                                                            \
  ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

inline float map_int_to_float(i32 x, i32 in_min, i32 in_max, float out_min,
                              float out_max) {
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

#endif
