#include "sensor.h"
#include "driver/gpio.h"

// 1. Initialize GPIO 15 as an Input pin
void init_proximity_sensor(void) {
    gpio_reset_pin(SENSOR_PIN);
    gpio_set_direction(SENSOR_PIN, GPIO_MODE_INPUT);
    
    // Note: DFRobot's SEN0239 provides its own clean HIGH/LOW voltage logic.
    // We disable pull-ups/pull-downs to let the sensor drive the pin cleanly.
    gpio_set_pull_mode(SENSOR_PIN, GPIO_FLOATING); 
}

// 2. Read and evaluate the sensor state
bool is_robot_detected(void) {
    // 1 = Robot present, 0 = Clear path
    return (gpio_get_level(SENSOR_PIN) == 1);
}