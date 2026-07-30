#ifndef REED_SWITCH_H
#define REED_SWITCH_H

#include <stdbool.h>

// Reed switch on GPIO 15 (D15)
#define REED_PIN 15

void init_reed_switch(void);
bool is_magnet_present(void);

#endif // REED_SWITCH_H