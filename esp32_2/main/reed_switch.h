#ifndef REED_SWITCH_H
#define REED_SWITCH_H

#include <stdbool.h>

// Change this from 2 to 4
#define REED_PIN 4  

void init_reed_switch(void);
bool is_magnet_present(void);

#endif // REED_SWITCH_H