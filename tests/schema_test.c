// The description the core turns into settings-page controls. The core's
// parser refuses a malformed one outright, so the bytes are pinned here.

#include <greatest.h>

#include <stdio.h>
#include <string.h>

#include "schema.h"

// src/actuator.c's table points at the three backends; the test links the
// table, not the drivers.
const Actuator act_uart = {.name = "pelco"};
const Actuator act_ms41908 = {.name = "ms41908"};
const Actuator act_gpiostep = {.name = "gpiostep"};

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
    ASSERT(schema_write(buf, sizeof buf, two_at, NULL));
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
    ASSERT_FALSE(schema_write(buf, sizeof buf, two_at, NULL));
    ASSERT_STR_EQ("", buf);
    PASS();
}

TEST an_empty_list_is_still_an_object(void) {
    char buf[128];
    ASSERT(schema_write(buf, sizeof buf, none_at, NULL));
    ASSERT_STR_EQ(
        "{\"properties\":{\"actuator\":{\"enum\":[],\"x-enum-titles\":{}}}}",
        buf);
    PASS();
}

// The real list, from src/actuator.c, against the core's limit. The backends
// are the test's own empty ones (below): only the names and titles are read.
TEST the_real_list_fits_and_names_every_driver(void) {
    static char buf[SCHEMA_MAX];
    ASSERT(schema_write(buf, sizeof buf, actuator_choice, actuator_head_choice));
    for (unsigned i = 0; actuator_choice(i); i++) {
        char quoted[64];
        snprintf(quoted, sizeof quoted, "\"%s\"", actuator_choice(i)->name);
        ASSERT(strstr(buf, quoted) != NULL);
    }
    PASS();
}

// ptz.head is the plugin's own key, so it must carry what the core requires of
// one -- a type, a default and a title -- and its default must be a choice.
TEST the_head_key_is_a_whole_plugin_key(void) {
    static char buf[SCHEMA_MAX];
    ASSERT(schema_write(buf, sizeof buf, actuator_choice, actuator_head_choice));
    const char *h = strstr(buf, "\"head\":{");
    ASSERT(h != NULL);
    ASSERT(strstr(h, "\"type\":\"string\"") != NULL);
    ASSERT(strstr(h, "\"default\":\"none\"") != NULL);
    ASSERT(strstr(h, "\"title\":\"Pan/tilt head\"") != NULL);
    ASSERT(strstr(h, "\"enum\":[\"none\",\"pelco-d\"]") != NULL);
    ASSERT_STR_EQ("none", actuator_head_choice(0)->name);
    // The object closes: the head is the last key, then properties, then the whole.
    size_t len = strlen(buf);
    ASSERT_EQ('}', buf[len - 1]);
    ASSERT_EQ('}', buf[len - 2]);
    PASS();
}

// ---- which head a camera gets (actuator_select_head) ---------------------

static bool pans(enum PtzVerb v) { (void)v; return true; }
static bool zooms(enum PtzVerb v) { return v == PTZ_TELE || v == PTZ_WIDE; }
static const Actuator panning_lens = {.name = "gpiostep", .has = pans};
static const Actuator zoom_lens = {.name = "ms41908", .has = zooms};

TEST no_head_unless_one_is_named(void) {
    const char *why = "unset";
    ASSERT_EQ(NULL, actuator_select_head(NULL, &zoom_lens, &why));
    ASSERT_EQ(NULL, why);
    ASSERT_EQ(NULL, actuator_select_head("", &zoom_lens, &why));
    ASSERT_EQ(NULL, actuator_select_head("none", &zoom_lens, &why));
    ASSERT_EQ(NULL, actuator_select_head("pelco-xm", &zoom_lens, &why));   // a lens wire
    ASSERT_EQ(NULL, why);   // nothing asked for, nothing to say
    PASS();
}

TEST a_pelco_d_head_beside_an_spi_lens(void) {
    const char *why = "unset";
    ASSERT_EQ(&act_uart, actuator_select_head("pelco-d", &zoom_lens, &why));
    ASSERT_EQ(NULL, why);
    PASS();
}

// One tty cannot be both the lens and the head, and a lens that pans would
// give pan two owners: both are refused, and say why.
TEST a_head_the_lens_cannot_have_is_refused_with_a_reason(void) {
    const char *why = NULL;
    ASSERT_EQ(NULL, actuator_select_head("pelco-d", &act_uart, &why));
    ASSERT(why != NULL);
    why = NULL;
    ASSERT_EQ(NULL, actuator_select_head("pelco-d", &panning_lens, &why));
    ASSERT(why != NULL);
    PASS();
}

SUITE(schema_suite) {
    RUN_TEST(the_real_list_fits_and_names_every_driver);
    RUN_TEST(lists_every_choice_with_its_title);
    RUN_TEST(a_buffer_too_small_says_nothing);
    RUN_TEST(an_empty_list_is_still_an_object);
    RUN_TEST(the_head_key_is_a_whole_plugin_key);
    RUN_TEST(no_head_unless_one_is_named);
    RUN_TEST(a_pelco_d_head_beside_an_spi_lens);
    RUN_TEST(a_head_the_lens_cannot_have_is_refused_with_a_reason);
}
