// SPDX-License-Identifier: MIT
#ifndef CIRCUITPYTHON_MATH_H
#define CIRCUITPYTHON_MATH_H
#include <float.h>
#include <stdint.h>
#include <string.h>

// CircuitPython 9.2.4 on RP2040 uses IEEE binary32 with the bottom two
// mantissa bits removed when boxing a float (MICROPY_OBJ_REPR_C).
// Keep each Python arithmetic result rounded at the same boundary.
// Reference: circuitpython/9.2.4/py/{circuitpy_mpconfig.h,obj.h}.
_Static_assert(sizeof(float)==sizeof(uint32_t) && FLT_RADIX==2 && FLT_MANT_DIG==24,
               "CircuitPython compatibility requires IEEE binary32");
static inline float cp_float(float value) {
    uint32_t bits;
    memcpy(&bits,&value,sizeof(bits));
    bits &= ~UINT32_C(3);
    memcpy(&value,&bits,sizeof(value));
    return value;
}
static inline float cp_add(float a, float b) { return cp_float(a+b); }
static inline float cp_multiply(float a, float b) { return cp_float(a*b); }
static inline float cp_divide(float a, float b) { return cp_float(a/b); }
#endif
