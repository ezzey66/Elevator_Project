#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>

// Define your GPIO Pin here (D15 maps to GPIO 15)
#define SENSOR_PIN 15  

// Function declarations
void init_proximity_sensor(void);
bool is_robot_detected(void);

#endif // SENSOR_H