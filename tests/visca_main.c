// Test runner for the VISCA suite (tests/visca_test.c). It links only the
// backend's pure logic (src/visca_calc.c) -- no tty, no thread, no
// HAL seams -- so it runs natively with nothing stubbed.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(visca_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(visca_suite);
    GREATEST_MAIN_END();
}
