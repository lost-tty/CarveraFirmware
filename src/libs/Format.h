#pragma once

#include <cstdarg>
#include <cstddef>

// The printf subset the firmware uses, and number parsing, without newlib's shared state.
namespace fmt {

int vformat(char *buf, size_t size, const char *format, va_list args);
int format(char *buf, size_t size, const char *format, ...) __attribute__((format(printf, 3, 4)));
double parse(const char *s, char **end);

}
