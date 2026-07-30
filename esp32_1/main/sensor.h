#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>

// Distance sensor on GPIO 2
#define SENSOR_PIN 2

// Function declarations
void init_proximity_sensor(void);
bool is_robot_detected(void);

#endif // SENSOR_H