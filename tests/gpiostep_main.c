// Test runner for the gpiostep suite (tests/gpiostep_test.c). It links only the
// backend's pure logic (src/gpiostep_calc.c) -- no /dev/motorDev, no thread, no
// HAL seams -- so it runs natively with nothing stubbed.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(gpiostep_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(gpiostep_suite);
    GREATEST_MAIN_END();
}
