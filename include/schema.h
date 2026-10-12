// The settings this plugin has, described for the core (cmd "schema" in
// af_plugin_abi.h): the core declares them in its config schema, which is how
// the settings page learns to draw them and the setter to accept them.

#ifndef MAJESTIC_AF_SCHEMA_H
#define MAJESTIC_AF_SCHEMA_H

#include <stdbool.h>
#include <stddef.h>

#include "actuator.h"

// Write the description into buf: today the list of actuators, taken from
// `choice` (actuator_choice, or a test's own list) until it returns NULL.
// False, with buf left empty, when it does not fit -- the core then keeps its
// built-in list, which beats offering half of this one. Pure: no HAL seam, no
// motor, so the core may ask while the engine is down.
bool schema_write(char *buf, size_t cap,
                  const ActuatorChoice *(*choice)(unsigned i));

#endif
