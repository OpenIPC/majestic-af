// The actuator registry: map isp.autofocus.actuator to a backend.

#include "actuator.h"

#include <string.h>

extern const Actuator act_uart;      // src/act_uart.c — Pelco / XiongMai over a tty
extern const Actuator act_ms41908;   // src/act_ms41908.c — MS41908M SPI stepper

const Actuator *actuator_select(const char *name) {
    if (name && !strcmp(name, act_ms41908.name)) {
        return &act_ms41908;
    }
    // "pelco-xm", "pelco-d", empty and anything unknown resolve to the UART
    // family (what the key has always meant); the exact protocol is picked
    // inside that backend at open().
    return &act_uart;
}
