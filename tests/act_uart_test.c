// The UART backend's open, against a pseudo-terminal standing in for the lens
// board's port: what it reports as the line (Actuator.line) is what the tty
// was really set to, and a port whose line cannot be set is not opened.
//
// Links the real src/act_uart.c and src/proto.c; the core's config and log
// seams are stubbed, the config pointing at whatever port a test chooses.

#define _XOPEN_SOURCE 600
#include <greatest.h>

#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "actuator.h"
#include <majestic/af_plugin_abi.h>
#include <majestic/log.h>

extern const Actuator act_uart;

// ---- the core, stubbed ---------------------------------------------------

static const char *cfg_port = "/dev/null";
static int cfg_speed = 115200;

const char *config_get_string(const char *path, const char *param) {
    (void)path;
    if (param && !strcmp(param, "port")) {
        return cfg_port;
    }
    return "pelco-xm";   // actuator
}
int config_get_int(const char *path, const char *param) {
    (void)path;
    return param && !strcmp(param, "speed") ? cfg_speed : 0;
}
bool config_get_boolean(const char *path, const char *param) {
    (void)path; (void)param;
    return false;
}
void log_log(enum LogType level, const char *file, int line, const char *func,
             const char *fmt, ...) {
    (void)level; (void)file; (void)line; (void)func; (void)fmt;
}

// ---- a pseudo-terminal for a port ----------------------------------------

static int master = -1;
static char slave[64];

static bool pty_up(void) {
    master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) || unlockpt(master)) {
        return false;
    }
    snprintf(slave, sizeof slave, "%s", ptsname(master));
    return true;
}

static void pty_down(void) {
    act_uart.close();
    if (master >= 0) {
        close(master);
        master = -1;
    }
}

// The rate the tty itself is set to, read back through the backend's own fd.
static speed_t tty_speed(void) {
    struct termios t;
    return tcgetattr(act_uart.fd(), &t) == 0 ? cfgetospeed(&t) : (speed_t)-1;
}

TEST the_line_is_the_port_and_rate_the_tty_was_set_to(void) {
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_speed = 9600;
    ASSERT(act_uart.open());
    char port[128];
    int baud = 0;
    ASSERT(act_uart.line(port, sizeof port, &baud));
    ASSERT_STR_EQ(slave, port);
    ASSERT_EQ(9600, baud);
    ASSERT_EQ(B9600, tty_speed());
    pty_down();
    PASS();
}

// 1500 is inside the range the config accepts and has no termios constant,
// so the line runs at 115200 -- and must say so.
TEST a_rate_with_no_constant_is_reported_as_the_one_set(void) {
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_speed = 1500;
    ASSERT(act_uart.open());
    char port[128];
    int baud = 0;
    ASSERT(act_uart.line(port, sizeof port, &baud));
    ASSERT_EQ(115200, baud);
    ASSERT_EQ(B115200, tty_speed());
    pty_down();
    PASS();
}

// /dev/null opens, but it has no line to set: not an open port, and no line.
TEST a_port_that_is_not_a_tty_is_not_opened(void) {
    cfg_port = "/dev/null";
    cfg_speed = 9600;
    ASSERT_FALSE(act_uart.open());
    ASSERT_EQ(-1, act_uart.fd());
    char port[128];
    int baud = 0;
    ASSERT_FALSE(act_uart.line(port, sizeof port, &baud));
    PASS();
}

TEST a_closed_port_reports_no_line(void) {
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_speed = 9600;
    ASSERT(act_uart.open());
    act_uart.close();
    char port[128];
    int baud = 0;
    ASSERT_FALSE(act_uart.line(port, sizeof port, &baud));
    pty_down();
    PASS();
}

SUITE(act_uart_suite) {
    RUN_TEST(the_line_is_the_port_and_rate_the_tty_was_set_to);
    RUN_TEST(a_rate_with_no_constant_is_reported_as_the_one_set);
    RUN_TEST(a_port_that_is_not_a_tty_is_not_opened);
    RUN_TEST(a_closed_port_reports_no_line);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(act_uart_suite);
    GREATEST_MAIN_END();
}
