#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>

// Distance sensor on GPIO 15
#define SENSOR_PIN 15

// Function declarations
void init_proximity_sensor(void);
bool is_robot_detected(void);

#endif // SENSOR_H