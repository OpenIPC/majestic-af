// Test runner for the af3 (step-based bracket-and-return AF) offline model. The suite lives in
// af3_model.c; it drives the real src/af3.c against a synthetic stepper lens with a settle
// transient and reversal backlash on a virtual clock — milliseconds, no hardware. Built natively
// (host gcc) by CMake when not cross-compiling.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(af3_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(af3_suite);
    GREATEST_MAIN_END();
}
