// The description the core turns into settings-page controls. The core's
// parser refuses a malformed one outright, so the bytes are pinned here.

#include <greatest.h>

#include <string.h>

#include "schema.h"

static const ActuatorChoice two[] = {
    {"pelco-xm", "XiongMai lens MCU (UART)"},
    {"newlens", "New \"lens\" (SPI)"},
};

static const ActuatorChoice *two_at(unsigned i) {
    return i < 2 ? &two[i] : NULL;
}

static const ActuatorChoice *none_at(unsigned i) {
    (void)i;
    return NULL;
}

TEST lists_every_choice_with_its_title(void) {
    char buf[512];
    ASSERT(schema_write(buf, sizeof buf, two_at));
    ASSERT_STR_EQ(
        "{\"properties\":{\"actuator\":{"
        "\"enum\":[\"pelco-xm\",\"newlens\"],"
        "\"x-enum-titles\":{\"pelco-xm\":\"XiongMai lens MCU (UART)\","
        "\"newlens\":\"New \\\"lens\\\" (SPI)\"}}}}",
        buf);
    PASS();
}

// Half a list would have the core offer some drivers and refuse the rest, so
// a description that does not fit is no description: the core keeps its own.
TEST a_buffer_too_small_says_nothing(void) {
    char buf[40];
    memset(buf, 'x', sizeof buf);
    ASSERT_FALSE(schema_write(buf, sizeof buf, two_at));
    ASSERT_STR_EQ("", buf);
    PASS();
}

TEST an_empty_list_is_still_an_object(void) {
    char buf[128];
    ASSERT(schema_write(buf, sizeof buf, none_at));
    ASSERT_STR_EQ(
        "{\"properties\":{\"actuator\":{\"enum\":[],\"x-enum-titles\":{}}}}",
        buf);
    PASS();
}

SUITE(schema_suite) {
    RUN_TEST(lists_every_choice_with_its_title);
    RUN_TEST(a_buffer_too_small_says_nothing);
    RUN_TEST(an_empty_list_is_still_an_object);
}
