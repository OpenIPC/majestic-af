// The actuator registry: map ptz.actuator to a backend.

#include "actuator.h"

#include <stddef.h>
#include <string.h>

extern const Actuator act_uart;      // src/act_uart.c — Pelco / XiongMai over a tty
extern const Actuator act_ms41908;   // src/act_ms41908.c — MS41908M SPI stepper
extern const Actuator act_gpiostep;  // src/act_gpiostep.c — GPIO pan/tilt steppers

// Every value the key may take, the backend that serves it, and its word on
// the settings page. Adding a driver is a row here: the core asks for this
// list ("schema") and offers what it is told, so nothing in majestic changes.
// Both UART rows are one backend; it picks the exact protocol at open().
static const struct {
    ActuatorChoice choice;
    const Actuator *act;
} choices[] = {
    {{"pelco-xm", "XiongMai lens MCU (UART)"}, &act_uart},
    {{"pelco-d", "Pelco-D (UART)"}, &act_uart},
    {{"ms41908", "MS41908 lens stepper (SPI)"}, &act_ms41908},
    {{"gpiostep", "GPIO pan/tilt steppers"}, &act_gpiostep},
};

const ActuatorChoice *actuator_choice(unsigned i) {
    return i < sizeof choices / sizeof *choices ? &choices[i].choice : NULL;
}

const Actuator *actuator_select(const char *name) {
    for (size_t i = 0; name && i < sizeof choices / sizeof *choices; i++) {
        if (!strcmp(name, choices[i].choice.name)) {
            return choices[i].act;
        }
    }
    // Empty and anything unknown resolve to the UART family (what the key has
    // always meant); the exact protocol is picked inside that backend at open().
    return &act_uart;
}

bool actuator_names_other(const char *name) {
    for (size_t i = 0; name && i < sizeof choices / sizeof *choices; i++) {
        if (!strcmp(name, choices[i].choice.name)) {
            return choices[i].act != &act_uart;
        }
    }
    return false;   // unknown resolves to the UART family (actuator_select)
}

// The heads: only backends that carry pan/tilt. pelco-xm is not one -- it is
// the XiongMai zoom block's wire. The UART backend picks pelco-d from ptz.head
// itself at open() when ptz.actuator names no UART protocol (act_uart.c).
static const struct {
    ActuatorChoice choice;
    const Actuator *act;
} heads[] = {
    {{"none", "No pan/tilt head"}, NULL},
    {{"pelco-d", "Pelco-D (UART)"}, &act_uart},
};

const ActuatorChoice *actuator_head_choice(unsigned i) {
    return i < sizeof heads / sizeof *heads ? &heads[i].choice : NULL;
}

const Actuator *actuator_select_head(const char *name, const Actuator *lens,
                                     const char **why) {
    if (why) {
        *why = NULL;
    }
    const Actuator *head = NULL;
    for (size_t i = 0; name && i < sizeof heads / sizeof *heads; i++) {
        if (!strcmp(name, heads[i].choice.name)) {
            head = heads[i].act;
        }
    }
    if (!head) {
        return NULL;   // none, empty or unknown: no head, as before ptz.head existed
    }
    if (head == lens) {
        if (why) {
            *why = "it is the lens's own transport";
        }
        return NULL;
    }
    if (lens && lens->has(PTZ_LEFT)) {
        if (why) {
            *why = "the lens already pans";
        }
        return NULL;
    }
    return head;
}
