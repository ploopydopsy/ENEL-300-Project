#include "stdio.h"

// Convert integer to decimal string (base 10).
static void itoa_simple(int value, char *str) {
    char temp[12];
    int i = 0, neg = 0;

    if (value < 0) {
        neg = 1;
        value = -value;
    }
    do {
        temp[i++] = (char)('0' + (value % 10));
        value /= 10;
    } while(value > 0);

    if (neg) temp[i++] = '-';

    int j;
    for (j = 0; j < i; j++) {
        str[j] = temp[i - j - 1];
    }
    str[i] = '\0';
}

int vsprintf(char *out, const char *format, va_list ap) {
    char *s = out;
    while(*format) {
        if (*format == '%') {
            format++;
            switch(*format) {
                case 'd': {
                    int val = (int)va_arg(ap, int);
                    char numStr[12];
                    itoa_simple(val, numStr);
                    for (char *p = numStr; *p; p++) {
                        *s++ = *p;
                    }
                    break;
                }
                case 'c': {
                    char c = (char)va_arg(ap, int);
                    *s++ = c;
                    break;
                }
                case 's': {
                    char *str = va_arg(ap, char*);
                    while(*str) {
                        *s++ = *str++;
                    }
                    break;
                }
                default:
                    // unknown format specifier, just print it
                    *s++ = *format;
                    break;
            }
            format++;
        } else {
            *s++ = *format++;
        }
    }
    *s = '\0';
    return (int)(s - out);
}

int sprintf(char *str, const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    int len = vsprintf(str, format, ap);
    va_end(ap);
    return len;
}
