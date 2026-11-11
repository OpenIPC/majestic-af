// act_visca — a lens MCU that takes Sony VISCA over a tty.
//
// Built for the Xiongmai N51813L 18x zoom block (Hi3518CV100): its lens MCU is on
// UART0 at 9600 8N1, camera address 1, and is also what configures the sensor (a
// separate boot script sends it the "init camera" frame before majestic starts).
// The verbs become the VISCA commands the stock firmware sends (visca_calc.c).
// Focus is the lens's own: a zoom hands it back to the MCU's autofocus, and only
// an operator's near/far switches it to manual -- run the engine with
// ptz.focusMode: manual so its own after-zoom search stays out of the way.
//
// Magnification. The MCU streams a status frame (BB 82 .. FF) with the integer
// magnification, and answers CAM_ZoomPosInq with the zoom position; this backend
// polls the position on its own reader thread, maps it through the measured curve,
// and pushes the result through af_zoom_report() (derives_mag), so the engine's
// "X<ratio>" UART reader -- the XiongMai Pelco board's report -- never starts.
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
#include <unistd.h>

#include <majestic/af_plugin_abi.h>   // config_get_* — the core's seams
#include <majestic/log.h>

void af_zoom_report(float mag);   // engine.c: the magnification sink

static pthread_mutex_t v_mu = PTHREAD_MUTEX_INITIALIZER;
static int v_fd = -1;
static bool v_focus_manual;   // the lens's focus mode as this backend last set it

static pthread_t v_reader;
static bool v_reader_started;
static volatile int v_stop;

#define VISCA_POLL_MS 400   // zoom position inquiry period

static speed_t baud_const(int baud) {
    switch (baud) {
    case 2400: return B2400;
    case 4800: return B4800;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B9600;   // VISCA's usual rate, and this MCU's
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
    int fd = open(port && *port ? port : "/dev/ttyAMA0",
                  O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        pthread_mutex_unlock(&v_mu);
        return false;   // motion.c owns the rate-limited retry logging
    }
    struct termios tio;
    memset(&tio, 0, sizeof tio);
    if (tcgetattr(fd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetspeed(&tio, baud_const(speed));
        tio.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &tio);
    }
    tcflush(fd, TCIFLUSH);
    v_fd = fd;
    // Where the MCU's focus mode stands is not known at load; say auto, and the
    // first zoom (which sends auto only from manual) is then a plain move.
    v_focus_manual = false;
    pthread_mutex_unlock(&v_mu);

    v_stop = 0;
    if (pthread_create(&v_reader, NULL, reader_main, NULL) == 0) {
        v_reader_started = true;
    } else {
        log_w("ptz: VISCA magnification reader did not start");
    }
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
    bool manual = v_focus_manual;
    int n = visca_frame(v, VISCA_SPEED_DEFAULT, v_focus_manual, &manual, f);
    bool ok = n > 0 && write_raw(f, (size_t)n);
    if (ok) {
        v_focus_manual = manual;
    }
    pthread_mutex_unlock(&v_mu);
    return ok;
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
    .derives_mag = true,
    .seed_mag = NULL,       // the MCU's zoom position is absolute
    .backlash_ms = 0,
    .zoom_settle_ms = 0,    // focus is the MCU's own; the engine's pass is off
    .zoom_late_ms = 0,
};
