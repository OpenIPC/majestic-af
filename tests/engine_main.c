// Test runner for engine.c's state file and its honesty rules (engine_state.c).
// The HAL seams and motion.c are stubbed there; engine.c and af2.c are real.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(engine_state_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(engine_state_suite);
    GREATEST_MAIN_END();
}
