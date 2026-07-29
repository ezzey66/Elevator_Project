#include "reed_switch.h"
#include "driver/gpio.h"

void init_reed_switch(void) {
    gpio_reset_pin(REED_PIN);
    gpio_set_direction(REED_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(REED_PIN, GPIO_PULLUP_ONLY);
}

bool is_magnet_present(void) {
    // Magnet nearby pulls the line low, so the reed switch is considered closed.
    return (gpio_get_level(REED_PIN) == 0);
}