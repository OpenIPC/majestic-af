// act_uart — the Pelco / XiongMai byte-frame actuator over a serial tty.
//
// This is the transport half that used to live in motion.c: it owns the one
// descriptor, its termios, the frame writer, and the vendor wake sequence, and
// exposes the fd so the engine's magnification reader can share it (the lens MCU
// reports its zoom ratio on the same RX line). The wire itself is proto.c. All of
// motion.c's arbitration — the watchdog, deadlines, manual-preempts-search — stays
// in motion.c and reaches the wire only through the Actuator vtable below, so the
// byte output is unchanged and proto_test still pins every frame.

#include "actuator.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <majestic/af_plugin_abi.h>   // config_get_* — the core's seams
#include <majestic/log.h>

#include "proto.h"

// A leaf lock: every byte this backend puts on the wire is serialised here. It is
// never held across a call back into motion.c, so motion.c's mo_mu -> u_mu is the
// only order that occurs. The magnification reader only READS the fd, on its own
// thread, which does not contend with a write.
static pthread_mutex_t u_mu = PTHREAD_MUTEX_INITIALIZER;
static int u_fd = -1;
static const PtzProto *u_proto;

static speed_t baud_const(int baud) {
    switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    default: return B115200;
    }
}

// The non-blocking write, finished bounded. u_mu is held by the caller.
//
// The descriptor is non-blocking (see uart_open), so a write can come back short
// or with EAGAIN even for eight bytes -- rare on an idle 115200 line, but a
// half-written Pelco frame is a frame the lens will not act on. Finish it,
// bounded, rather than log it and move on.
static bool write_raw(const unsigned char *buf, size_t len) {
    if (u_fd < 0 || !len) {
        return false;
    }
    size_t done = 0;
    for (int tries = 0; done < len && tries < 20; tries++) {
        ssize_t n = write(u_fd, buf + done, len - done);
        if (n > 0) {
            done += (size_t)n;
        } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            usleep(1000);
        } else {
            break;
        }
    }
    if (done != len) {
        log_e("ptz: short UART write (%u of %u): %s", (unsigned)done,
              (unsigned)len, strerror(errno));
        return false;
    }
    return true;
}

static bool uart_open(void) {
    const char *port = config_get_string("isp.autofocus", "port");
    const char *name = config_get_string("isp.autofocus", "actuator");
    int speed = config_get_int("isp.autofocus", "speed");

    pthread_mutex_lock(&u_mu);
    if (u_fd >= 0) {
        pthread_mutex_unlock(&u_mu);
        return true;
    }
    u_proto = ptz_proto(name);
    if (name && *name && strcmp(name, ptz_proto_name(u_proto))) {
        log_w("ptz: unknown actuator '%s', using %s", name,
              ptz_proto_name(u_proto));
    }

    // Read AND write on one descriptor: the magnification reader shares it
    // through fd(). Two opens meant two independent tcsetattr calls on the same
    // tty, the writer's without CLOCAL|CREAD.
    //
    // O_NONBLOCK is load-bearing twice over, both learned the hard way. Opening a
    // tty without it blocks until carrier is asserted, which on a UART with no
    // modem lines need never happen -- and the port is opened before CLOCAL is
    // set, so there is nothing yet to say the line has no carrier to wait for. And
    // the reader drains with `while ((n = read(fd,...)) > 0)`, which terminates on
    // EAGAIN: on a blocking descriptor that loop parks in read() forever, never
    // looks at af_reader_stop again, and the join on the way down hangs the whole
    // process.
    int fd = open(port && *port ? port : "/dev/ttyAMA0",
                  O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        pthread_mutex_unlock(&u_mu);
        return false;   // motion.c owns the rate-limited retry logging
    }
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetspeed(&tio, baud_const(speed));
        tio.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &tio);
    }
    tcflush(fd, TCIFLUSH);
    u_fd = fd;
    pthread_mutex_unlock(&u_mu);
    return true;
}

static bool uart_emit(enum PtzVerb v, int speed) {
    unsigned char f[PTZ_FRAME_MAX];
    int n = ptz_frame(u_proto, v, speed, f);
    if (n <= 0) {
        return false;
    }
    pthread_mutex_lock(&u_mu);
    bool ok = write_raw(f, (size_t)n);
    pthread_mutex_unlock(&u_mu);
    return ok;
}

static bool uart_has(enum PtzVerb v) { return ptz_proto_has(u_proto, v); }

static const char *uart_proto_name(void) { return ptz_proto_name(u_proto); }
// After a zoom-out from X5.0 to X2.0 the 85H50AI's board set focus at 32-35 % of where the
// after-zoom pass then put it, the same with the focus gear's slack pre-loaded either way; a zoom-in
// to X2.0 lands at ~92 %. Ending the zoom-out with a 150-200 ms zoom-in brought it to 90-91 %, the
// ratio unchanged within 0.1; 60 ms gave 56 %, 100 ms 75-89 %. The stock firmware has no such step:
// its picture stays at 0-7 % of best after a zoom-out to X1.0-X2.8 (OpenIPC/motors uart-bridge
// dvrip_twin.py, out-X* levels). Measured on the XM board only, so plain Pelco-D gets none.
static long uart_zoom_out_bounce_ms(void) {
    return strcmp(ptz_proto_name(u_proto), "pelco-xm") == 0 ? 200 : 0;
}

static bool uart_wake_blob(void) {
    size_t wlen = 0;
    const unsigned char *w = ptz_wake_blob(u_proto, &wlen);
    // No blob for this protocol is nothing to send, not a failure; only a short
    // or failed write is, and the caller can tell the two apart.
    pthread_mutex_lock(&u_mu);
    bool ok = u_fd >= 0 && (!w || write_raw(w, wlen));
    pthread_mutex_unlock(&u_mu);
    return ok;
}

static bool uart_wake(void) {
    unsigned char f[PTZ_FRAME_MAX];

    if (!uart_wake_blob()) {
        return false;
    }
    if (ptz_preset_frame(u_proto, 0x53, f) <= 0) {
        return true;   // the XiongMai variant has only the blob
    }

    // The Pelco-D lens tool's wakeup: two vendor presets, then an ICR exercise.
    // Slow on purpose -- the lens needs the time -- and run on the caller's
    // thread, which is why this verb is not on the pad.
    static const int steps_ms[] = {500, 500};
    const int presets[] = {0x53, 0x52};
    for (int i = 0; i < 2; i++) {
        usleep(steps_ms[i] * 1000);
        int n = ptz_preset_frame(u_proto, presets[i], f);
        pthread_mutex_lock(&u_mu);
        write_raw(f, (size_t)n);
        pthread_mutex_unlock(&u_mu);
    }
    usleep(3000 * 1000);
    uart_emit(PTZ_NIGHT, 0);
    usleep(3000 * 1000);
    uart_emit(PTZ_DAY, 0);
    return true;
}

static void uart_close(void) {
    pthread_mutex_lock(&u_mu);
    if (u_fd >= 0) {
        close(u_fd);
        u_fd = -1;
    }
    pthread_mutex_unlock(&u_mu);
}

static int uart_fd(void) {
    pthread_mutex_lock(&u_mu);
    int fd = u_fd;
    pthread_mutex_unlock(&u_mu);
    return fd;
}

const Actuator act_uart = {
    .name = "pelco",   // the family; the resolved protocol name is proto.c's
    .proto_name = uart_proto_name,
    .open = uart_open,
    .close = uart_close,
    .emit = uart_emit,
    .has = uart_has,
    .wake_blob = uart_wake_blob,
    .wake = uart_wake,
    .fd = uart_fd,
    .derives_mag = false,
    .backlash_ms = 0,      // af2's default, measured on the 85H50AI
    // The 85H50AI's MCU goes on moving focus after a zoom stop: usually done within 1-3 s,
    // and the stock firmware's picture has settled by then. On one of the two boards measured
    // it made a last move as late as 9.4 s after the stop (4 runs in 6 at X3.0), to the same
    // position on its own curve whatever focus the lens had meanwhile (OpenIPC/motors
    // xm-uart/PROTOCOL.md, "Zoom tracking inside the board"). Waiting 10 s for that rare move
    // made every pass finish ~16 s after the zoom, long after the stock camera; so the pass
    // starts once the usual settle is over and watches for the late move instead.
    .zoom_settle_ms = 3000,
    .zoom_late_ms = 10000,
    .zoom_out_bounce_ms = uart_zoom_out_bounce_ms,
};
