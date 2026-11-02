// The fallback HAL seams (src/seams_fallback.c): the config reader a plugin
// loaded by a core that exports no seams reads its settings with.
//
// Links src/seams_fallback.c alone, with MAJESTIC_YAML pointed at a file in the
// build tree that each test writes.

#include <greatest.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <majestic/af_plugin_abi.h>

static void write_cfg(const char *text) {
    FILE *f = fopen(MAJESTIC_YAML, "w");
    if (f) {
        fputs(text, f);
        fclose(f);
    }
}

// Shaped like the file majestic writes: two levels, two-space indent, and
// keys of the same name in other sections.
static const char *const CFG =
    "system:\n"
    "  webPort: 80\n"
    "  logLevel: debug\n"
    "video0:\n"
    "  enabled: true\n"
    "  #codec: h265\n"
    "  fps: 20\n"
    "ptz:\n"
    "  enabled: true\n"
    "  actuator: gpiostep\n"
    "  pulse: 300\n"
    "  presetMax: 0\n"
    "  panInvert: false\n"
    "audio:\n"
    "  enabled: false\n"
    "  pulse: 7\n";

TEST a_key_is_read_from_the_section_asked_for(void) {
    write_cfg(CFG);
    ASSERT_STR_EQ("gpiostep", config_get_string("ptz", "actuator"));
    ASSERT_EQ(300, config_get_int("ptz", "pulse"));
    ASSERT_EQ(0, config_get_int("ptz", "presetMax"));
    ASSERT(config_get_boolean("ptz", "enabled"));
    ASSERT_FALSE(config_get_boolean("ptz", "panInvert"));
    ASSERT_FALSE(config_get_boolean("audio", "enabled"));
    ASSERT_EQ(7, config_get_int("audio", "pulse"));
    ASSERT_EQ(20, config_get_int("video0", "fps"));
    PASS();
}

TEST a_missing_key_section_or_file_is_unset(void) {
    write_cfg(CFG);
    ASSERT_EQ(NULL, config_get_string("ptz", "port"));
    ASSERT_EQ(-1, config_get_int("ptz", "baud"));
    ASSERT_FALSE(config_get_boolean("ptz", "tiltInvert"));
    ASSERT_EQ(NULL, config_get_string("motors", "actuator"));
    // A key under another section is not this one's.
    ASSERT_EQ(-1, config_get_int("ptz", "fps"));
    ASSERT_EQ(NULL, config_get_string(NULL, "actuator"));
    ASSERT_EQ(NULL, config_get_string("ptz", NULL));

    unlink(MAJESTIC_YAML);
    ASSERT_EQ(NULL, config_get_string("ptz", "actuator"));
    ASSERT_EQ(-1, config_get_int("ptz", "pulse"));
    ASSERT_FALSE(config_get_boolean("ptz", "enabled"));
    PASS();
}

TEST an_int_that_is_not_a_whole_number_is_unset(void) {
    write_cfg("ptz:\n"
              "  pulse: 300ms\n"
              "  baud: 1.5\n"
              "  address: \"12\"\n"
              "  presetMs: 99999999999\n"
              "  presetMax: -3\n");
    ASSERT_EQ(-1, config_get_int("ptz", "pulse"));
    ASSERT_EQ(-1, config_get_int("ptz", "baud"));
    ASSERT_EQ(12, config_get_int("ptz", "address"));
    ASSERT_EQ(-1, config_get_int("ptz", "presetMs"));
    ASSERT_EQ(-3, config_get_int("ptz", "presetMax"));
    PASS();
}

TEST booleans_read_as_yaml_spells_them(void) {
    write_cfg("ptz:\n"
              "  a: true\n  b: True\n  c: yes\n  d: on\n"
              "  e: false\n  f: no\n  g: 1\n  h: maybe\n");
    ASSERT(config_get_boolean("ptz", "a"));
    ASSERT(config_get_boolean("ptz", "b"));
    ASSERT(config_get_boolean("ptz", "c"));
    ASSERT(config_get_boolean("ptz", "d"));
    ASSERT_FALSE(config_get_boolean("ptz", "e"));
    ASSERT_FALSE(config_get_boolean("ptz", "f"));
    ASSERT_FALSE(config_get_boolean("ptz", "g"));
    ASSERT_FALSE(config_get_boolean("ptz", "h"));
    PASS();
}

TEST quotes_and_comments_are_not_part_of_the_value(void) {
    write_cfg("# majestic.yaml\n"
              "ptz:   # the motors\n"
              "  port: \"/dev/ttyS1\"   # the lens board\n"
              "  head: 'pelco-d'\n"
              "  actuator: pelco-xm # wire\n"
              "  focusMode: semi#auto\n"
              "  # pulse: 9\n"
              "  presetMs:   4000   \r\n"
              "  address: # unset\n");
    ASSERT_STR_EQ("/dev/ttyS1", config_get_string("ptz", "port"));
    ASSERT_STR_EQ("pelco-d", config_get_string("ptz", "head"));
    ASSERT_STR_EQ("pelco-xm", config_get_string("ptz", "actuator"));
    // '#' starts a comment only after a blank.
    ASSERT_STR_EQ("semi#auto", config_get_string("ptz", "focusMode"));
    ASSERT_EQ(-1, config_get_int("ptz", "pulse"));
    ASSERT_EQ(4000, config_get_int("ptz", "presetMs"));
    ASSERT_EQ(NULL, config_get_string("ptz", "address"));
    PASS();
}

TEST a_nested_key_is_not_the_sections_own(void) {
    write_cfg("ptz:\n"
              "  limits:\n"
              "    pulse: 9\n"
              "  pulse: 300\n"
              "  actuator: gpiostep\n");
    ASSERT_EQ(300, config_get_int("ptz", "pulse"));
    ASSERT_EQ(NULL, config_get_string("ptz", "limits"));
    PASS();
}

TEST an_overlong_line_is_skipped_whole(void) {
    // Its tail would otherwise start a "section" at the margin and end ptz
    // before the keys below it.
    char text[2048];
    int n = snprintf(text, sizeof text, "ptz:\n  note: ");
    memset(text + n, 'x', 900);
    n += 900;
    snprintf(text + n, sizeof text - (size_t)n,
             " pulse: 9\n  pulse: 300\n  actuator: gpiostep\n");
    write_cfg(text);
    ASSERT_EQ(300, config_get_int("ptz", "pulse"));
    ASSERT_STR_EQ("gpiostep", config_get_string("ptz", "actuator"));
    ASSERT_EQ(NULL, config_get_string("ptz", "note"));
    PASS();
}

TEST a_saved_change_is_read_and_handed_out_values_survive_it(void) {
    write_cfg("ptz:\n  actuator: pelco-d\n  pulse: 300\n");
    const char *before = config_get_string("ptz", "actuator");
    ASSERT_STR_EQ("pelco-d", before);

    // Same size, within the same second: a cache keyed on the file's stat
    // would go on answering the old value.
    write_cfg("ptz:\n  actuator: ms41908\n  pulse: 400\n");
    ASSERT_STR_EQ("ms41908", config_get_string("ptz", "actuator"));
    ASSERT_EQ(400, config_get_int("ptz", "pulse"));
    // A caller still holding the old value holds the old value.
    ASSERT_STR_EQ("pelco-d", before);

    // Saved back, it is the same storage: switching to and fro costs nothing.
    write_cfg("ptz:\n  actuator: pelco-d\n");
    ASSERT_EQ(before, config_get_string("ptz", "actuator"));
    PASS();
}

SUITE(seams_fallback_suite) {
    RUN_TEST(a_key_is_read_from_the_section_asked_for);
    RUN_TEST(a_missing_key_section_or_file_is_unset);
    RUN_TEST(an_int_that_is_not_a_whole_number_is_unset);
    RUN_TEST(booleans_read_as_yaml_spells_them);
    RUN_TEST(quotes_and_comments_are_not_part_of_the_value);
    RUN_TEST(a_nested_key_is_not_the_sections_own);
    RUN_TEST(an_overlong_line_is_skipped_whole);
    RUN_TEST(a_saved_change_is_read_and_handed_out_values_survive_it);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(seams_fallback_suite);
    GREATEST_MAIN_END();
}
