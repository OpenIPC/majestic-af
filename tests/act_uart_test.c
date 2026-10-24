// The UART backend's open, against a pseudo-terminal standing in for the lens
// board's port: what it reports as the line (Actuator.line) is what the tty
// was really set to, and a port whose line cannot be set is not opened.
//
// Links the real src/act_uart.c and src/proto.c; the core's config and log
// seams are stubbed, the config pointing at whatever port a test chooses.

#define _XOPEN_SOURCE 600
#include <greatest.h>

#include <fcntl.h>
#include <limits.h>
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
static int cfg_addr = PTZ_ADDR_DEFAULT;
static const char *cfg_actuator = "pelco-xm";
static const char *cfg_head = NULL;

const char *config_get_string(const char *path, const char *param) {
    if (strcmp(path, "ptz")) {
        return NULL;
    }
    if (!strcmp(param, "port")) {
        return cfg_port;
    }
    if (!strcmp(param, "head")) {
        return cfg_head;
    }
    return !strcmp(param, "actuator") ? cfg_actuator : NULL;
}
int config_get_int(const char *path, const char *param) {
    if (strcmp(path, "ptz")) {
        return INT_MIN;
    }
    if (!strcmp(param, "baud")) {
        return cfg_speed;
    }
    return !strcmp(param, "address") ? cfg_addr : INT_MIN;
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
    int baud = 0, addr = 0;
    ASSERT(act_uart.line(port, sizeof port, &baud, &addr));
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
    int baud = 0, addr = 0;
    ASSERT(act_uart.line(port, sizeof port, &baud, &addr));
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
    int baud = 0, addr = 0;
    ASSERT_FALSE(act_uart.line(port, sizeof port, &baud, &addr));
    PASS();
}

TEST a_closed_port_reports_no_line(void) {
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_speed = 9600;
    ASSERT(act_uart.open());
    act_uart.close();
    char port[128];
    int baud = 0, addr = 0;
    ASSERT_FALSE(act_uart.line(port, sizeof port, &baud, &addr));
    pty_down();
    PASS();
}

// What the lens board would read: everything the backend has written so far.
static int pty_read(unsigned char *buf, int n) {
    usleep(20000);
    int got = (int)read(master, buf, (size_t)n);
    return got < 0 ? 0 : got;
}

static void reset_cfg(void) {
    cfg_speed = 9600;
    cfg_addr = PTZ_ADDR_DEFAULT;
}

// ptz.address is the address on the wire, and the line reports it.
TEST the_configured_address_goes_on_the_wire(void) {
    reset_cfg();
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_addr = 9;
    ASSERT(act_uart.open());
    ASSERT(act_uart.emit(PTZ_TELE, 0));
    unsigned char f[16];
    ASSERT_EQ(8, pty_read(f, sizeof f));
    ASSERT_MEM_EQ(((unsigned char[]){0xc5, 0x09, 0x00, 0x20, 0x00, 0x00, 0x29,
                                     0x5c}),
                  f, 8);
    char port[128];
    int baud = 0, addr = 0;
    ASSERT(act_uart.line(port, sizeof port, &baud, &addr));
    ASSERT_EQ(9, addr);
    pty_down();
    reset_cfg();
    PASS();
}

// A value the schema should have refused is not put on the wire as a byte it
// would wrap to: the frames go to the default address instead.
TEST an_address_out_of_range_is_the_default(void) {
    reset_cfg();
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_addr = 300;
    ASSERT(act_uart.open());
    char port[128];
    int baud = 0, addr = 0;
    ASSERT(act_uart.line(port, sizeof port, &baud, &addr));
    ASSERT_EQ(PTZ_ADDR_DEFAULT, addr);
    pty_down();
    reset_cfg();
    PASS();
}

// The tty as the pan/tilt head beside a lens on another wire: ptz.actuator
// names the lens (no protocol of this tty's), so the protocol is ptz.head's,
// and a pan goes out as a Pelco-D frame at half speed.
TEST as_the_head_it_speaks_the_heads_protocol(void) {
    reset_cfg();
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_actuator = "ms41908";
    cfg_head = "pelco-d";
    ASSERT(act_uart.open());
    ASSERT_STR_EQ("pelco-d", act_uart.proto_name());
    ASSERT(act_uart.emit(PTZ_LEFT, 0));
    unsigned char f[16];
    ASSERT_EQ(7, pty_read(f, sizeof f));
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x04, 0x20, 0x00, 0x25}), f, 7);
    pty_down();
    cfg_actuator = "pelco-xm";
    cfg_head = NULL;
    PASS();
}

// Where ptz.actuator names a protocol of this tty's, it is the lens, and a
// ptz.head left in the file changes nothing.
TEST a_uart_lens_keeps_its_own_protocol(void) {
    reset_cfg();
    ASSERT(pty_up());
    cfg_port = slave;
    cfg_actuator = "pelco-xm";
    cfg_head = "pelco-d";
    ASSERT(act_uart.open());
    ASSERT_STR_EQ("pelco-xm", act_uart.proto_name());
    pty_down();
    cfg_head = NULL;
    PASS();
}

SUITE(act_uart_suite) {
    RUN_TEST(the_line_is_the_port_and_rate_the_tty_was_set_to);
    RUN_TEST(a_rate_with_no_constant_is_reported_as_the_one_set);
    RUN_TEST(a_port_that_is_not_a_tty_is_not_opened);
    RUN_TEST(a_closed_port_reports_no_line);
    RUN_TEST(the_configured_address_goes_on_the_wire);
    RUN_TEST(an_address_out_of_range_is_the_default);
    RUN_TEST(as_the_head_it_speaks_the_heads_protocol);
    RUN_TEST(a_uart_lens_keeps_its_own_protocol);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(act_uart_suite);
    GREATEST_MAIN_END();
}
