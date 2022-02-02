#pragma once

uint64_t cycleCount(void);

enum { CLOCKSPEED_HZ = 72000000, C_US = CLOCKSPEED_HZ / 1000000 };

// usec = 10...1000000 (.01ms ..  1s)
void delay(uint32_t usec);
