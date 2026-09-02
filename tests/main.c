// Test runner for the majestic-af offline model suite. The suite itself lives in
// af2_model.c; it drives the real src/af2.c engine against a synthetic parfocal
// lens+scene with a virtual clock, so it runs in milliseconds and needs no
// hardware. Built natively (host gcc) by CMake when not cross-compiling.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(af2_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(af2_suite);
    GREATEST_MAIN_END();
}
