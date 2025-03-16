#ifndef STDIO_H
#define STDIO_H

#include "stdarg.h"

int sprintf(char *str, const char *format, ...);
int vsprintf(char *str, const char *format, va_list ap);

#endif // STDIO_H
