// Test runner for the settings description (tests/schema_test.c). schema.c is
// pure, so it links and runs natively with nothing stubbed.

#include <greatest.h>

GREATEST_MAIN_DEFS();

extern SUITE(schema_suite);

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(schema_suite);
    GREATEST_MAIN_END();
}
