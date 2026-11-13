// Test runner for the actuator suite (tests/actuator_test.c). It links only the
// MS41908M backend's pure logic (src/ms41908_calc.c) — no SPI, no /dev/mem, no
// threads, no HAL seams — so it runs natively with nothing stubbed.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(actuator_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(actuator_suite);
    GREATEST_MAIN_END();
}
