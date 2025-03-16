#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

void sensor_init(void);
uint16_t measure_distance_cm(void);

#endif
