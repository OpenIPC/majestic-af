// act_uart — the Pelco / XiongMai byte-frame actuator over a serial tty.
//
// This is the transport half that used to live in motion.c: it owns the one
// descriptor, its termios, the frame writer, and the vendor wake sequence, and
// exposes the fd so the engine's magnification reader can share it (the lens MCU
// reports its zoom ratio on the same RX line). The wire itself is proto.c. All of
// motion.c's arbitration — the watchdog, deadlines, manual-preempts-search — stays
// in motion.c and reaches the wire only through the Actuator vtable below, so the
// byte output is unchanged and proto_test still pins every frame.
//
// As the pan/tilt head, the port may instead be ptz.headPort: another tty, or
// tcp://host:port for a base whose RS-485 pair hangs off a gateway on the
// network (an SBC or a serial server copying the bytes onto the pair).

#include "actuator.h"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <majestic/af_plugin_abi.h>   // config_get_* — the core's seams
#include <majestic/log.h>

#include "proto.h"

// Where a GPIO is driven from. A define so the test can stand a directory in.
#ifndef PTZ_GPIO_ROOT
#define PTZ_GPIO_ROOT "/sys/class/gpio"
#endif

// A leaf lock: every byte this backend puts on the wire is serialised here. It is
// never held across a call back into motion.c, so motion.c's mo_mu -> u_mu is the
// only order that occurs. The magnification reader only READS the fd, on its own
// thread, which does not contend with a write.
static pthread_mutex_t u_mu = PTHREAD_MUTEX_INITIALIZER;
static int u_fd = -1;
static const PtzProto *u_proto;
// What the open port is running at, for the capability line (line()).
static char u_port[128];
static int u_baud;
// The device every frame goes to, read with the rest of the line at open.
static unsigned char u_addr = PTZ_ADDR_DEFAULT;
// The RS-485 transceiver's direction (ptz.rs485Pin), or -1 on a plain UART: the
// sysfs value file of a pin held high while a frame goes out, under u_mu.
static int u_dir_fd = -1;
static bool u_dir_warned;
// The pin this plugin exported itself, to give back at close; -1 for one a
// board script already had, which stays the script's.
static int u_dir_exported = -1;
// The port is a TCP connection to a gateway (ptz.headPort tcp://...), not a tty.
static bool u_net;
// After a failed connect, none is tried again before u_net_next (monotonic ms):
// an unreachable gateway would otherwise stall every frame, and every retry of
// the open, for the whole connect timeout. 64-bit: a 32-bit long of ms wraps
// negative after 24.8 days up.
static bool u_net_backoff;
static long long u_net_next;
static bool u_net_down;

#define NET_PREFIX "tcp://"
#define NET_CONNECT_MS 300
#define NET_RETRY_MS 2000
// How long the gateway may sit on sent bytes without acknowledging them, or
// stay silent to keepalive probes, before the connection counts as dead and
// the next frame redials: a gateway that lost power sends no FIN or RST.
#define NET_USER_TIMEOUT_MS 2000

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

// baud_const has a case for every rate ptz_baud (proto.c) passes through, so
// ptz_baud(baud) is the rate the line is actually set to.

static bool write_file(const char *path, const char *text) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        return false;
    }
    bool ok = write(fd, text, strlen(text)) == (ssize_t)strlen(text);
    close(fd);
    return ok;
}

// A half-duplex bus: the transceiver sends while this pin is high and listens
// otherwise, and the pl011 driver in the 4.9 kernels here cannot turn it round
// for us. Output and low in one write: "out" and then a level is two steps, and
// a pin that came up high between them would hold the bus against the head.
static int dir_open(int pin) {
    char path[96], num[16];
    snprintf(num, sizeof num, "%d", pin);
    // Fails with EBUSY on a pin a board script already exported. The direction
    // below is what says whether the pin can be used.
    u_dir_exported = write_file(PTZ_GPIO_ROOT "/export", num) ? pin : -1;
    snprintf(path, sizeof path, PTZ_GPIO_ROOT "/gpio%d/direction", pin);
    if (!write_file(path, "low")) {
        return -1;
    }
    snprintf(path, sizeof path, PTZ_GPIO_ROOT "/gpio%d/value", pin);
    return open(path, O_WRONLY);
}

// Back to the kernel, or a reload that picks another pin leaves this one held.
// It stays an output driven low: unexporting only ends sysfs's claim on it.
static void dir_release(void) {
    if (u_dir_exported >= 0) {
        char num[16];
        snprintf(num, sizeof num, "%d", u_dir_exported);
        write_file(PTZ_GPIO_ROOT "/unexport", num);
        u_dir_exported = -1;
    }
}

// True on a plain UART, which has no pin to move.
static bool dir_set(bool send) {
    if (u_dir_fd < 0 || write(u_dir_fd, send ? "1" : "0", 1) == 1) {
        return true;
    }
    log_e("ptz: RS-485 direction pin stuck %s: %s", send ? "low" : "high",
          strerror(errno));
    return false;
}

// write() only queued the frame, and going back to listen now would cut it off
// in the FIFO. tcdrain waits for the last stop bit, a few ms under u_mu, which
// only ever delays the next frame.
static bool drained(void) {
    if (u_dir_fd < 0) {
        return true;
    }
    while (tcdrain(u_fd) != 0) {
        if (errno != EINTR) {
            log_e("ptz: UART drain: %s", strerror(errno));
            return false;
        }
    }
    return true;
}

static bool is_net(const char *port) {
    return port && !strncmp(port, NET_PREFIX, strlen(NET_PREFIX));
}

static long long mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000L;
}

// A non-blocking connect, waited for: it is made under u_mu, and motion.c's
// lock above it, so it must not take the kernel's minutes on a dead route.
static bool connected_within(int fd, int ms) {
    struct pollfd p = {.fd = fd, .events = POLLOUT};
    int n;
    do {
        n = poll(&p, 1, ms);
    } while (n < 0 && errno == EINTR);
    if (n == 0) {
        errno = ETIMEDOUT;
        return false;
    }
    int err = 0;
    socklen_t len = sizeof err;
    if (n < 0 || getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0) {
        return false;
    }
    errno = err;
    return err == 0;
}

// Ask the kernel to notice a gateway that vanished without a FIN or RST
// (power, cable): unacknowledged frames, or unanswered probes, fail the socket
// after NET_USER_TIMEOUT_MS, and the next frame redials instead of queueing.
static void net_dead_peer_timeouts(int fd) {
    int one = 1, idle = 2, intvl = 1, cnt = 2;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof idle);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof intvl);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof cnt);
#ifdef TCP_USER_TIMEOUT
    unsigned int ut = NET_USER_TIMEOUT_MS;
    setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &ut, sizeof ut);
#endif
}

// A connected, non-blocking socket to tcp://host:port, or -1 with errno set.
// The host is a numeric IPv4 or bracketed IPv6 address, never a name: a lookup
// cannot be bounded, and this runs under u_mu beneath motion.c's lock, where a
// slow resolver would hold every PTZ command and the watchdog's stop.
static int net_connect(const char *spec) {
    long long t = mono_ms();
    if (u_net_backoff && t < u_net_next) {
        errno = EAGAIN;
        return -1;
    }
    char host[128];
    snprintf(host, sizeof host, "%s", spec + strlen(NET_PREFIX));
    char *colon = strrchr(host, ':');
    if (!colon || !colon[1]) {
        errno = EINVAL;
        return -1;
    }
    *colon = 0;
    char *h = host;
    if (*h == '[' && colon > h + 1 && colon[-1] == ']') {
        h++;
        colon[-1] = 0;
    }
    struct addrinfo hints = {.ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM,
                             .ai_flags = AI_NUMERICHOST | AI_NUMERICSERV};
    struct addrinfo *res = NULL;
    int fd = -1;
    int rc = getaddrinfo(h, colon + 1, &hints, &res);
    if (rc != 0) {
        errno = rc == EAI_SYSTEM ? errno : EINVAL;   // not host:port in numbers
    }
    // One connect wait for the whole dial, however many addresses there are.
    long long deadline = t + NET_CONNECT_MS;
    for (struct addrinfo *ai = rc == 0 ? res : NULL; ai && fd < 0; ai = ai->ai_next) {
        int left = (int)(deadline - mono_ms());
        if (left <= 0) {
            errno = ETIMEDOUT;
            break;
        }
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }
        // Blocking, the connect below would wait out the kernel's route timeout.
        int fl = fcntl(fd, F_GETFL);
        if (fl < 0 || fcntl(fd, F_SETFL, fl | O_NONBLOCK) != 0) {
            int e = errno;
            close(fd);
            fd = -1;
            errno = e;
            continue;
        }
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) != 0 &&
            !(errno == EINPROGRESS && connected_within(fd, left))) {
            int e = errno;
            close(fd);
            fd = -1;
            errno = e;
        }
    }
    if (res) {
        freeaddrinfo(res);
    }
    if (fd < 0) {
        int e = errno;
        u_net_backoff = true;
        u_net_next = mono_ms() + NET_RETRY_MS;
        if (!u_net_down) {
            log_w("ptz: head gateway %s: %s", spec, strerror(e));
            u_net_down = true;
        }
        errno = e;
        return -1;
    }
    // A frame is a few bytes that should leave now, not wait for more.
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    net_dead_peer_timeouts(fd);
    u_net_backoff = false;
    if (u_net_down) {
        log_i("ptz: head gateway %s connected", spec);
        u_net_down = false;
    }
    return fd;
}

// Whether the gateway is still there. One that closed shows as a readable
// socket that reads 0, and the first send() after that still succeeds: the
// frame would be lost and reported sent. Whatever the base answered is read
// and dropped (bounded, against a peer that floods): nobody reads a head.
static bool net_alive(void) {
    unsigned char sink[64];
    for (int i = 0; i < 16; i++) {
        ssize_t n = recv(u_fd, sink, sizeof sink, MSG_DONTWAIT);
        if (n == 0) {
            return false;
        }
        if (n < 0) {
            return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
        }
    }
    return true;
}

// A new connection onto the same descriptor number, so the fd fd() handed out
// stays this backend's. The dead one stays in place when no new one is had,
// and its sends keep failing until a later frame reconnects.
static bool net_reconnect(void) {
    int fd = net_connect(u_port);
    if (fd < 0) {
        return false;
    }
    bool ok = dup2(fd, u_fd) >= 0;
    close(fd);
    return ok;
}

static ssize_t put_bytes(const unsigned char *buf, size_t len) {
    // MSG_NOSIGNAL: a gateway gone away is an EPIPE here, not a SIGPIPE that
    // takes majestic down.
    return u_net ? send(u_fd, buf, len, MSG_NOSIGNAL) : write(u_fd, buf, len);
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
    // A frame sent with the transceiver listening reaches nobody, so it is not
    // sent. Failing here tells motion it did not go out, which is the truth.
    if (!dir_set(true)) {
        return false;
    }
    if (u_net && !net_alive() && !net_reconnect()) {
        return false;
    }
    size_t done = 0;
    bool redialled = false;
    for (int tries = 0; done < len && tries < 20; tries++) {
        ssize_t n = put_bytes(buf + done, len - done);
        if (n > 0) {
            done += (size_t)n;
        } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            usleep(1000);
        } else if (u_net && !redialled && done == 0) {
            // Reset under us (the gateway restarted) before any of the frame
            // left: the frame, once, on a new connection. Not after part of it
            // went: the gateway may have put that prefix on the pair already,
            // and the frame is reported unsent for motion.c to retry instead.
            redialled = true;
            if (!net_reconnect()) {
                break;
            }
        } else {
            break;
        }
    }
    bool ok = done == len;
    if (!ok) {
        log_e("ptz: short %s write (%u of %u): %s", u_net ? "gateway" : "UART",
              (unsigned)done, (unsigned)len, strerror(errno));
    }
    // The pin goes back down whatever happened: a transceiver left in send
    // holds the bus against the head.
    ok = drained() && ok;
    return dir_set(false) && ok;
}

static bool uart_open(void) {
    const char *port = config_get_string("ptz", "port");
    const char *name = config_get_string("ptz", "actuator");
    // The head's own port, when it has one: only as the head (below), and then
    // the lens keeps ptz.port to itself.
    const char *head_port = NULL;
    // Opened as the pan/tilt head beside a lens on another wire (ptz.head,
    // actuator.h): ptz.actuator names that other lens, and the protocol is the
    // head's. Only then -- an unknown or legacy actuator name is this tty's
    // lens, as actuator_select resolves it, and keeps its fallback protocol
    // whatever ptz.head says.
    if (actuator_names_other(name)) {
        head_port = config_get_string("ptz", "headPort");
        const char *head = config_get_string("ptz", "head");
        if (head && *head && !strcmp(head, ptz_proto_name(ptz_proto(head)))) {
            name = head;
        }
    }
    if (head_port && *head_port) {
        port = head_port;
    }
    if (!port || !*port) {
        port = "/dev/ttyAMA0";
    }
    int speed = config_get_int("ptz", "baud");
    // A head on a tty of its own may run at another rate than the lens: a
    // VISCA block at 9600 beside a Pelco-D base at 2400. Unset (0, or a
    // core's -1 / INT_MIN), it is ptz.baud.
    if (port == head_port) {
        int hb = config_get_int("ptz", "headBaud");
        if (hb > 0) {
            speed = hb;
        }
    }
    bool addr_ok;
    int addr = ptz_addr(config_get_int("ptz", "address"), &addr_ok);

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

    // A gateway: no line to set, and the RS-485 turnaround is its business.
    // Only for the head: a lens shares its fd with the magnification reader,
    // and net_alive() drains what arrives.
    if (port == head_port && is_net(port)) {
        int fd = net_connect(port);
        if (fd < 0) {
            pthread_mutex_unlock(&u_mu);
            return false;
        }
        if (!addr_ok) {
            log_w("ptz: address %d is not a Pelco address, using %d",
                  config_get_int("ptz", "address"), addr);
        }
        u_fd = fd;
        u_net = true;
        snprintf(u_port, sizeof u_port, "%s", port);
        u_baud = 0;   // nothing here was set to a rate
        u_addr = (unsigned char)addr;
        pthread_mutex_unlock(&u_mu);
        return true;
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
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        pthread_mutex_unlock(&u_mu);
        return false;   // motion.c owns the rate-limited retry logging
    }
    // A port whose line could not be set is not open at that rate, and is
    // not reported as open at all: the capability line would otherwise name
    // a speed nothing set, and the lens would be spoken to at whatever the
    // tty was left at. motion.c treats the false as any failed open: it
    // retries, and owns the rate-limited log, as for the open(2) above.
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        pthread_mutex_unlock(&u_mu);
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    if (cfsetspeed(&tio, baud_const(speed)) != 0 ||
        tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        pthread_mutex_unlock(&u_mu);
        return false;
    }
    if (ptz_baud(speed) != speed) {
        log_w("ptz: %d baud is not a rate this port can be set to, using %d",
              speed, ptz_baud(speed));
    }
    if (!addr_ok) {
        log_w("ptz: address %d is not a Pelco address, using %d",
              config_get_int("ptz", "address"), addr);
    }
    // Set on a camera that needs it, a pin that cannot be driven is a port that
    // cannot send: every frame would go nowhere while reporting success. Not
    // open, then, and said once, as motion.c retries an open for as long as the
    // camera is up.
    int dir_pin = config_get_int("ptz", "rs485Pin");
    if (dir_pin >= 0) {
        int dir = dir_open(dir_pin);
        if (dir < 0) {
            if (!u_dir_warned) {
                log_e("ptz: RS-485 direction pin %d: %s", dir_pin, strerror(errno));
                u_dir_warned = true;
            }
            dir_release();
            close(fd);
            pthread_mutex_unlock(&u_mu);
            return false;
        }
        u_dir_fd = dir;
    }
    tcflush(fd, TCIFLUSH);
    u_fd = fd;
    u_net = false;
    snprintf(u_port, sizeof u_port, "%s", port);
    u_baud = ptz_baud(speed);
    u_addr = (unsigned char)addr;
    pthread_mutex_unlock(&u_mu);
    return true;
}

static bool uart_emit(enum PtzVerb v, int speed) {
    unsigned char f[PTZ_FRAME_MAX];
    pthread_mutex_lock(&u_mu);
    int n = ptz_frame(u_proto, u_addr, v, speed, f);
    if (n <= 0) {
        pthread_mutex_unlock(&u_mu);
        return false;
    }
    char hex[3 * PTZ_FRAME_MAX + 1];
    for (int i = 0; i < n; i++) {
        snprintf(hex + 3 * i, 4, "%02x%s", f[i], i + 1 < n ? " " : "");
    }
    bool ok = write_raw(f, (size_t)n);
    // Every frame that went out, at verbose and in wire order (under u_mu):
    // the one way to see what reached the lens board without a tap on its
    // UART. Cheap -- a move is a handful of frames. One that did not go out
    // is said louder.
    if (ok) {
        log_v("ptz: %s speed %d -> %s", ptz_verb_name(v), speed, hex);
    } else {
        log_w("ptz: %s frame did not reach the lens: %s", ptz_verb_name(v), hex);
    }
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

static bool uart_preset(enum PtzPresetOp op, int n) {
    unsigned char f[PTZ_FRAME_MAX];
    pthread_mutex_lock(&u_mu);
    int len = ptz_preset_op_frame(u_proto, u_addr, op, n, f);
    bool ok = len > 0 && write_raw(f, (size_t)len);
    pthread_mutex_unlock(&u_mu);
    if (ok) {
        log_v("ptz: preset %s %d", op == PTZ_PRESET_SET ? "set" : op == PTZ_PRESET_GO ? "go" : "clear", n);
    }
    return ok;
}

static bool uart_has_presets(void) {
    pthread_mutex_lock(&u_mu);
    bool h = u_fd >= 0 && ptz_proto_has_presets(u_proto);
    pthread_mutex_unlock(&u_mu);
    return h;
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
    if (ptz_preset_frame(u_proto, PTZ_ADDR_DEFAULT, 0x53, f) <= 0) {
        return true;   // the XiongMai variant has only the blob
    }

    // The Pelco-D lens tool's wakeup: two vendor presets, then an ICR exercise.
    // Slow on purpose -- the lens needs the time -- and run on the caller's
    // thread, which is why this verb is not on the pad.
    //
    // Each preset is built under u_mu, with the address of the port as it is
    // at that moment: the sleeps leave room for a reload to close the port
    // and reopen it at another address, and a frame built before that would
    // go out on the new port to the old device.
    static const int steps_ms[] = {500, 500};
    const int presets[] = {0x53, 0x52};
    for (int i = 0; i < 2; i++) {
        usleep(steps_ms[i] * 1000);
        pthread_mutex_lock(&u_mu);
        int n = ptz_preset_frame(u_proto, u_addr, presets[i], f);
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
    // A reload may fix the address: its open tries at once.
    u_net = false;
    u_net_backoff = false;
    u_net_down = false;
    if (u_dir_fd >= 0) {
        close(u_dir_fd);
        u_dir_fd = -1;
    }
    dir_release();
    pthread_mutex_unlock(&u_mu);
}

static int uart_fd(void) {
    pthread_mutex_lock(&u_mu);
    int fd = u_fd;
    pthread_mutex_unlock(&u_mu);
    return fd;
}

static bool uart_line(char *port, size_t n, int *baud, int *addr) {
    pthread_mutex_lock(&u_mu);
    bool open_ = u_fd >= 0;
    if (open_) {
        snprintf(port, n, "%s", u_port);
        *baud = u_baud;
        *addr = u_addr;
    }
    pthread_mutex_unlock(&u_mu);
    return open_;
}

const Actuator act_uart = {
    .name = "pelco",   // the family; the resolved protocol name is proto.c's
    // Pan and tilt carry a speed byte (data 1 / data 2, 1..63), which is where
    // ptz_frame() puts the speed it is given. Zoom and focus frames have none,
    // and the XiongMai lens board ignores one anyway: the stock firmware sends
    // them the same at every DVRIP Step, and the zoom runs at the same rate.
    .honours_speed = true,
    .proto_name = uart_proto_name,
    .open = uart_open,
    .close = uart_close,
    .emit = uart_emit,
    .preset = uart_preset,
    .has_presets = uart_has_presets,
    .has = uart_has,
    .wake_blob = uart_wake_blob,
    .wake = uart_wake,
    .fd = uart_fd,
    .line = uart_line,
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
