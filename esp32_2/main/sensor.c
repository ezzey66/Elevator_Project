#include "sensor.h"
#include "driver/gpio.h"

void init_proximity_sensor(void) {
    gpio_reset_pin(SENSOR_PIN);
    gpio_set_direction(SENSOR_PIN, GPIO_MODE_INPUT);

    // Let the sensor drive the line cleanly without internal pull-ups.
    gpio_set_pull_mode(SENSOR_PIN, GPIO_FLOATING);
}

bool is_robot_detected(void) {
    // 1 = object detected near the floor sensor, 0 = clear path
    return (gpio_get_level(SENSOR_PIN) == 1);
}