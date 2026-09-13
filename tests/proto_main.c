// Test runner for the wire-frame suite (tests/proto_test.c). proto.c is pure —
// it touches no HAL seam and no port — so it links and runs natively with
// nothing stubbed.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(proto_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(proto_suite);
    GREATEST_MAIN_END();
}
