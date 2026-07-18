#ifndef REED_SWITCH_H
#define REED_SWITCH_H

#include <stdbool.h>

// Reed switch on GPIO 2
#define REED_PIN 2

void init_reed_switch(void);
bool is_magnet_present(void);

#endif // REED_SWITCH_H