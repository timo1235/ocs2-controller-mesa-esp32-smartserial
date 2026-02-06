#pragma once
// Minimal Arduino stub for native test compilation.
// Provides only the types and macros needed by project headers.

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define HIGH 1
#define LOW  0

typedef uint8_t byte;

inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
