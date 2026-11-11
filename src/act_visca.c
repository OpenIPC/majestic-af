// act_visca — a lens MCU that takes Sony VISCA over a tty.
//
// Built for the Xiongmai N51813L 18x zoom block (Hi3518CV100): its lens MCU is on
// UART0 at 9600 8N1, camera address 1, and is also what configures the sensor (a
// separate boot script sends it the "init camera" frame before majestic starts).
// The verbs become the VISCA commands the stock firmware sends (visca_calc.c).
// Focus is the lens's own: a zoom hands it back to the MCU's autofocus, and only
// an operator's near/far switches it to manual. The engine books no after-zoom
// pass for it (own_focus); an explicit /autofocus still runs one.
//
// Magnification. The MCU streams a status frame (BB 82 .. FF) with the integer
// magnification, and answers CAM_ZoomPosInq with the zoom position; this backend
// polls the position on its own reader thread, maps it through the measured curve,
// and pushes the result through af_zoom_report() (derives_mag), so the engine's
// "X<ratio>" UART reader -- the XiongMai Pelco board's report -- never starts;
// the same thread therefore flushes a settled value to disk (af_zoom_flush).
//
// Locking: v_mu is a leaf lock around the descriptor and the focus-mode state.
// emit() is called with motion.c's mo_mu held and only takes v_mu for one bounded
// write. The reader thread takes v_mu only to send its inquiry, and calls
// af_zoom_report() with nothing held. The reader is joinable and joined in close().

#include "act_visca.h"
#include "actuator.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include <majestic/af_plugin_abi.h>   // config_get_* — the core's seams
#include <majestic/log.h>

void af_zoom_report(float mag);   // engine.c: the magnification sink
void af_zoom_flush(void);         // engine.c: write a settled magnification out

static pthread_mutex_t v_mu = PTHREAD_MUTEX_INITIALIZER;
static int v_fd = -1;
static char v_port[64];
static int v_baud;
// The lens's focus mode as this backend last set it, while v_focus_known. It is
// not known at open (a reload can find the MCU in either), nor after a write
// that may have put the mode frame on the wire without the move behind it; the
// next move then sends the mode it needs whatever the cache says.
static bool v_focus_manual;
static bool v_focus_known;

static pthread_t v_reader;
static bool v_reader_started;
static volatile int v_stop;

#define VISCA_POLL_MS 400   // zoom position inquiry period

static speed_t baud_const(int baud, int *opened) {
    switch (baud) {
    case 2400: *opened = 2400; return B2400;
    case 4800: *opened = 4800; return B4800;
    case 19200: *opened = 19200; return B19200;
    case 38400: *opened = 38400; return B38400;
    case 57600: *opened = 57600; return B57600;
    case 115200: *opened = 115200; return B115200;
    default: *opened = 9600; return B9600;   // VISCA's usual rate, and this MCU's
    }
}

// The non-blocking write, finished bounded. v_mu is held by the caller.
static bool write_raw(const unsigned char *buf, size_t len) {
    if (v_fd < 0 || !len) {
        return false;
    }
    size_t done = 0;
    for (int tries = 0; done < len && tries < 50; tries++) {
        ssize_t n = write(v_fd, buf + done, len - done);
        if (n > 0) {
            done += (size_t)n;
        } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            usleep(1000);
        } else {
            break;
        }
    }
    if (done != len) {
        log_e("ptz: short VISCA write (%u of %u): %s", (unsigned)done,
              (unsigned)len, strerror(errno));
        return false;
    }
    return true;
}

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void *reader_main(void *arg) {
    (void)arg;
    unsigned char frame[32];
    size_t flen = 0;
    float last = -1.0f;
    long next_poll = 0;

    while (!v_stop) {
        long t = now_ms();
        if (t >= next_poll) {
            unsigned char q[VISCA_FRAME_MAX];
            int n = visca_zoom_inquiry(q);
            pthread_mutex_lock(&v_mu);
            write_raw(q, (size_t)n);
            pthread_mutex_unlock(&v_mu);
            next_poll = t + VISCA_POLL_MS;
        }

        pthread_mutex_lock(&v_mu);
        int fd = v_fd;
        pthread_mutex_unlock(&v_mu);
        if (fd < 0) {
            break;
        }
        fd_set rs;
        FD_ZERO(&rs);
        FD_SET(fd, &rs);
        struct timeval tv = {0, 100 * 1000};
        if (select(fd + 1, &rs, NULL, NULL, &tv) <= 0) {
            af_zoom_flush();
            continue;
        }
        unsigned char buf[64];
        ssize_t r;
        while ((r = read(fd, buf, sizeof buf)) > 0) {
            for (ssize_t i = 0; i < r; i++) {
                if (flen < sizeof frame) {
                    frame[flen++] = buf[i];
                } else {
                    flen = 0;   // no frame is this long: resynchronise on the next FF
                }
                if (buf[i] != 0xff) {
                    continue;
                }
                int pos = -1, mag = -1;
                float m = -1.0f;
                switch (visca_parse(frame, flen, &pos, &mag)) {
                case 1: m = visca_pos_to_mag(pos); break;
                case 2: if (last < 0 && mag > 0) m = (float)mag; break;   // until a position arrives
                default: break;
                }
                flen = 0;
                if (m > 0 && (m - last >= 0.05f || last - m >= 0.05f)) {
                    last = m;
                    af_zoom_report(m);   // nothing held
                }
            }
        }
        af_zoom_flush();   // nothing held; a no-op until a value has settled
    }
    return NULL;
}

static bool visca_open(void) {
    const char *port = config_get_string("ptz", "port");
    int speed = config_get_int("ptz", "baud");

    pthread_mutex_lock(&v_mu);
    if (v_fd >= 0) {
        pthread_mutex_unlock(&v_mu);
        return true;
    }
    // O_NONBLOCK for the same two reasons as act_uart: no carrier wait at open,
    // and the reader's drain loop must end on EAGAIN.
    if (!port || !*port) {
        port = "/dev/ttyAMA0";
    }
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        pthread_mutex_unlock(&v_mu);
        return false;   // motion.c owns the rate-limited retry logging
    }
    // A line that cannot be set up is not open: VISCA at the wrong rate is noise,
    // and motion.c's retry is what gets another go at it.
    struct termios tio;
    int opened = 0;
    memset(&tio, 0, sizeof tio);
    bool ok = tcgetattr(fd, &tio) == 0;
    if (ok) {
        cfmakeraw(&tio);
        ok = cfsetspeed(&tio, baud_const(speed, &opened)) == 0;
    }
    if (ok) {
        tio.c_cflag |= (CLOCAL | CREAD);
        ok = tcsetattr(fd, TCSANOW, &tio) == 0;
    }
    if (!ok) {
        log_e("ptz: cannot set up %s for VISCA: %s", port, strerror(errno));
        close(fd);
        pthread_mutex_unlock(&v_mu);
        return false;
    }
    tcflush(fd, TCIFLUSH);
    v_fd = fd;
    snprintf(v_port, sizeof v_port, "%s", port);
    v_baud = opened;
    v_focus_known = false;
    pthread_mutex_unlock(&v_mu);

    // Without its reader this backend has no magnification at all, so a lens
    // that moves but never reports is not open either.
    v_stop = 0;
    if (pthread_create(&v_reader, NULL, reader_main, NULL) != 0) {
        log_e("ptz: VISCA magnification reader did not start");
        pthread_mutex_lock(&v_mu);
        close(v_fd);
        v_fd = -1;
        pthread_mutex_unlock(&v_mu);
        return false;
    }
    v_reader_started = true;
    return true;
}

static void visca_close(void) {
    unsigned char f[VISCA_FRAME_MAX];
    int n = visca_frame(PTZ_STOP, 0, false, NULL, f);

    v_stop = 1;
    if (v_reader_started) {
        pthread_join(v_reader, NULL);
        v_reader_started = false;
    }
    pthread_mutex_lock(&v_mu);
    if (v_fd >= 0) {
        write_raw(f, (size_t)n);
        close(v_fd);
        v_fd = -1;
    }
    pthread_mutex_unlock(&v_mu);
}

static bool visca_emit(enum PtzVerb v, int speed) {
    unsigned char f[VISCA_FRAME_MAX];
    (void)speed;   // motion.c's 0..63 is a pan/tilt speed; zoom/focus use the lens's
    pthread_mutex_lock(&v_mu);
    // Not known, the lens is taken to be in the mode this verb does NOT want, so
    // the frame carries the mode switch: auto before a zoom, manual before focus.
    bool cur = v_focus_known ? v_focus_manual : (v == PTZ_TELE || v == PTZ_WIDE);
    bool manual = cur;
    int n = visca_frame(v, VISCA_SPEED_DEFAULT, cur, &manual, f);
    bool ok = n > 0 && write_raw(f, (size_t)n);
    if (ok) {
        if (v != PTZ_STOP) {   // a stop leaves the mode where it was
            v_focus_manual = manual;
            v_focus_known = true;
        }
    } else if (n > 0) {
        v_focus_known = false;   // the mode frame may have gone out alone
    }
    pthread_mutex_unlock(&v_mu);
    return ok;
}

static bool visca_line(char *port, size_t n, int *baud, int *addr) {
    pthread_mutex_lock(&v_mu);
    bool open_ = v_fd >= 0;
    if (open_) {
        snprintf(port, n, "%s", v_port);
        *baud = v_baud;
        *addr = 1;   // VISCA camera address 1, the only one this MCU answers
    }
    pthread_mutex_unlock(&v_mu);
    return open_;
}

static bool visca_has_verb(enum PtzVerb v) { return visca_has(v); }

static const char *visca_proto_name(void) { return "visca"; }

static bool visca_wake(void) { return true; }   // the boot script woke it already

static int visca_fd(void) { return -1; }   // own reader: the engine's stays off

const Actuator act_visca = {
    .name = "visca",
    .proto_name = visca_proto_name,
    .open = visca_open,
    .close = visca_close,
    .emit = visca_emit,
    .has = visca_has_verb,
    .wake_blob = visca_wake,
    .wake = visca_wake,
    .fd = visca_fd,
    .line = visca_line,
    .derives_mag = true,
    .seed_mag = NULL,       // the MCU's zoom position is absolute
    .backlash_ms = 0,
    .zoom_settle_ms = 0,    // focus is the MCU's own; the engine's pass is off
    .zoom_late_ms = 0,
    .own_focus = true,      // the MCU refocuses after a zoom by itself
};
