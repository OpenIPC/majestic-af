// Host unit test for src/act_motorsd.c against an in-process mock motorsd
// Unix socket server: verifies capabilities, non-blocking emit() / move_steps()
// (including diagonal pan+tilt), frame_steps(), initial & event-driven
// zoom_magnification reporting via af_zoom_report(), and clean teardown.

#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <greatest.h>

#include <errno.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "actuator.h"
#include "motion.h"
#include <majestic/af_plugin_abi.h>
#include <majestic/log.h>

extern const Actuator act_motorsd;

static char cfg_port[128];
static int cfg_frame_w = 1060;
static int cfg_frame_h = 640;
static bool cfg_omit_initial_zoom = false;
static bool cfg_hold_focus_completion = false;
static bool cfg_fail_move_once = false;

const char *config_get_string(const char *path, const char *param) {
    if (!strcmp(path, "ptz") && !strcmp(param, "port")) {
        return cfg_port;
    }
    return NULL;
}

int config_get_int(const char *path, const char *param) {
    if (!strcmp(path, "ptz")) {
        if (!strcmp(param, "frameWidth")) return cfg_frame_w;
        if (!strcmp(param, "frameHeight")) return cfg_frame_h;
    }
    return INT_MIN;
}

bool config_get_boolean(const char *path, const char *param) {
    (void)path;
    (void)param;
    return false;
}

void log_log(enum LogType level, const char *file, int line, const char *func,
             const char *fmt, ...) {
    (void)level;
    (void)file;
    (void)line;
    (void)func;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static pthread_mutex_t test_mu = PTHREAD_MUTEX_INITIALIZER;
static float reported_mag = 0.0f;
static int reported_count = 0;

void af_zoom_report(float mag) {
    pthread_mutex_lock(&test_mu);
    reported_mag = mag;
    reported_count++;
    pthread_mutex_unlock(&test_mu);
}

static int listen_fd = -1;
static volatile bool server_stop = false;
static pthread_t server_thread;
static char seen_ops[8192];
static int sub_fd = -1;

static void record_op(const char *line) {
    pthread_mutex_lock(&test_mu);
    size_t cur = strlen(seen_ops);
    size_t add = strlen(line);
    if (cur + add + 2 < sizeof seen_ops) {
        memcpy(seen_ops + cur, line, add);
        seen_ops[cur + add] = '\n';
        seen_ops[cur + add + 1] = '\0';
    }
    pthread_mutex_unlock(&test_mu);
}

static const char *extract_id(const char *req, char *buf, size_t n) {
    const char *p = strstr(req, "\"id\":\"");
    if (!p) return "1";
    p += 6;
    const char *e = strchr(p, '"');
    if (!e || (size_t)(e - p) >= n) return "1";
    memcpy(buf, p, (size_t)(e - p));
    buf[e - p] = '\0';
    return buf;
}

static void *mock_server(void *arg) {
    (void)arg;
    int clients[8];
    int nclients = 0;
    while (!server_stop) {
        struct pollfd pfds[9];
        pfds[0].fd = listen_fd;
        pfds[0].events = POLLIN;
        for (int i = 0; i < nclients; i++) {
            pfds[1 + i].fd = clients[i];
            pfds[1 + i].events = POLLIN;
        }
        int pr = poll(pfds, 1 + nclients, 50);
        if (pr <= 0) continue;
        if ((pfds[0].revents & POLLIN) && nclients < 8) {
            int cfd = accept(listen_fd, NULL, NULL);
            if (cfd >= 0) clients[nclients++] = cfd;
        }
        for (int i = 0; i < nclients; i++) {
            if (!(pfds[1 + i].revents & POLLIN)) continue;
            char buf[2048];
            ssize_t r = read(clients[i], buf, sizeof buf - 1);
            if (r <= 0) {
                close(clients[i]);
                if (sub_fd == clients[i]) sub_fd = -1;
                clients[i] = clients[--nclients];
                i--;
                continue;
            }
            buf[r] = '\0';
            char *save = NULL;
            for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
                record_op(line);
                char idb[64];
                const char *id = extract_id(line, idb, sizeof idb);
                char resp[512];
                if (strstr(line, "\"op\":\"capabilities\"")) {
                    snprintf(resp, sizeof resp,
                             "{\"version\":1,\"id\":\"%s\",\"ok\":true,\"driver\":\"anjoy\","
                             "\"axes\":[\"pan\",\"tilt\",\"zoom\",\"focus\"],"
                             "\"raw\":false,\"available\":true}\n", id);
                } else if (strstr(line, "\"op\":\"status\"")) {
                    if (cfg_omit_initial_zoom) {
                        snprintf(resp, sizeof resp,
                                 "{\"version\":1,\"id\":\"%s\",\"ok\":true,\"driver\":\"anjoy\","
                                 "\"available\":true}\n", id);
                    } else {
                        snprintf(resp, sizeof resp,
                                 "{\"version\":1,\"id\":\"%s\",\"ok\":true,\"driver\":\"anjoy\","
                                 "\"available\":true,\"zoom_magnification\":3.5}\n", id);
                    }
                } else if (strstr(line, "\"op\":\"subscribe\"")) {
                    sub_fd = clients[i];
                    snprintf(resp, sizeof resp,
                             "{\"version\":1,\"id\":\"%s\",\"ok\":true,\"message\":\"subscribed\"}\n", id);
                } else if (cfg_fail_move_once && strstr(line, "\"op\":\"pantilt\"")) {
                    cfg_fail_move_once = false;
                    snprintf(resp, sizeof resp,
                             "{\"version\":1,\"id\":\"%s\",\"ok\":false,\"error\":\"simulated drop\"}\n", id);
                } else {
                    snprintf(resp, sizeof resp,
                             "{\"version\":1,\"id\":\"%s\",\"ok\":true,\"message\":\"ok\"}\n", id);
                }
                (void)write(clients[i], resp, strlen(resp));
                if (!cfg_hold_focus_completion &&
                    strstr(line, "\"op\":\"move\"") && strstr(line, "\"axis\":\"focus\"")) {
                    const char *evt = "{\"version\":1,\"event\":\"movement_ended\",\"axes\":8,\"completed_mono_ms\":100}\n";
                    (void)write(clients[i], evt, strlen(evt));
                }
            }
        }
    }
    for (int i = 0; i < nclients; i++) close(clients[i]);
    return NULL;
}

static bool wait_for_op(const char *needle) {
    for (int i = 0; i < 40; i++) {
        pthread_mutex_lock(&test_mu);
        bool got = strstr(seen_ops, needle) != NULL;
        pthread_mutex_unlock(&test_mu);
        if (got) return true;
        usleep(10000);
    }
    return false;
}

TEST opens_moves_presets_and_scales_zoom(void) {
    cfg_omit_initial_zoom = false;
    cfg_hold_focus_completion = false;
    cfg_fail_move_once = false;
    snprintf(cfg_port, sizeof cfg_port, "/tmp/test_motorsd_%d.sock", (int)getpid());
    unlink(cfg_port);

    listen_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    ASSERT(listen_fd >= 0);
    struct sockaddr_un addr = {.sun_family = AF_UNIX};
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", cfg_port);
    ASSERT_EQ(0, bind(listen_fd, (struct sockaddr *)&addr, sizeof addr));
    ASSERT_EQ(0, listen(listen_fd, 4));

    server_stop = false;
    seen_ops[0] = '\0';
    reported_mag = 0.0f;
    reported_count = 0;
    ASSERT_EQ(0, pthread_create(&server_thread, NULL, mock_server, NULL));

    ASSERT(act_motorsd.open());
    ASSERT_STR_EQ("motorsd:anjoy", act_motorsd.proto_name());
    ASSERT_EQ(-1, act_motorsd.fd());
    ASSERT(act_motorsd.derives_mag);
    ASSERT(act_motorsd.honours_speed);
    ASSERT(act_motorsd.position == NULL);
    ASSERT(act_motorsd.has_presets && act_motorsd.has_presets());
    ASSERT(act_motorsd.preset != NULL);
    ASSERT(act_motorsd.focus_ms != NULL);
    ASSERT(act_motorsd.has(PTZ_LEFT));
    ASSERT(act_motorsd.has(PTZ_UPRIGHT));
    ASSERT(act_motorsd.has(PTZ_TELE));
    ASSERT(act_motorsd.has(PTZ_NEAR));

    int fw = 0, fh = 0;
    ASSERT(act_motorsd.frame_steps(&fw, &fh));
    ASSERT_EQ(303, fw);   /* 1060 / 3.5 */
    ASSERT_EQ(183, fh);   /* 640 / 3.5 */
    ASSERT_EQ(10L, act_motorsd.step_ms(PTZ_LEFT, 0));

    pthread_mutex_lock(&test_mu);
    ASSERT(reported_count >= 1);
    ASSERT(reported_mag > 3.4f && reported_mag < 3.6f);
    pthread_mutex_unlock(&test_mu);

    ASSERT(act_motorsd.move_steps(PTZ_UPRIGHT, 250, 0));
    ASSERT(wait_for_op("\"duration_ms\":2500"));
    pthread_mutex_lock(&test_mu);
    ASSERT(strstr(seen_ops, "\"pantilt\"") != NULL);
    ASSERT(strstr(seen_ops, "\"upright\"") != NULL);
    pthread_mutex_unlock(&test_mu);

    ASSERT(act_motorsd.preset(PTZ_PRESET_SET, 1));
    ASSERT(wait_for_op("\"preset.set\""));
    ASSERT(act_motorsd.preset(PTZ_PRESET_GO, 2));
    ASSERT(wait_for_op("\"preset.call\""));
    ASSERT(act_motorsd.preset(PTZ_PRESET_CLEAR, 3));
    ASSERT(wait_for_op("\"preset.clear\""));

    if (sub_fd >= 0) {
        const char *evt = "{\"version\":1,\"event\":\"telemetry\",\"name\":\"zoom_magnification\",\"value\":7.2}\n";
        (void)write(sub_fd, evt, strlen(evt));
        for (int i = 0; i < 40; i++) {
            pthread_mutex_lock(&test_mu);
            bool got = reported_mag > 7.1f && reported_mag < 7.3f;
            pthread_mutex_unlock(&test_mu);
            if (got) break;
            usleep(10000);
        }
    }
    pthread_mutex_lock(&test_mu);
    ASSERT(reported_mag > 7.1f && reported_mag < 7.3f);
    pthread_mutex_unlock(&test_mu);

    ASSERT(act_motorsd.frame_steps(&fw, &fh));
    ASSERT_EQ(147, fw);   /* 1060 / 7.2 */
    ASSERT_EQ(89, fh);    /* 640 / 7.2 */

    ASSERT_EQ(50L, act_motorsd.focus_ms(1, 50));

    act_motorsd.close();
    server_stop = true;
    pthread_join(server_thread, NULL);
    close(listen_fd);
    unlink(cfg_port);
    PASS();
}

TEST refuses_unscaled_frame_steps_until_zoom_reported(void) {
    cfg_omit_initial_zoom = true;
    cfg_hold_focus_completion = false;
    cfg_fail_move_once = false;
    snprintf(cfg_port, sizeof cfg_port, "/tmp/test_motorsd_nozoom_%d.sock", (int)getpid());
    unlink(cfg_port);

    listen_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    ASSERT(listen_fd >= 0);
    struct sockaddr_un addr = {.sun_family = AF_UNIX};
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", cfg_port);
    ASSERT_EQ(0, bind(listen_fd, (struct sockaddr *)&addr, sizeof addr));
    ASSERT_EQ(0, listen(listen_fd, 4));

    server_stop = false;
    seen_ops[0] = '\0';
    ASSERT_EQ(0, pthread_create(&server_thread, NULL, mock_server, NULL));

    ASSERT(act_motorsd.open());
    int fw = 0, fh = 0;
    ASSERT(!act_motorsd.frame_steps(&fw, &fh));
    ASSERT(act_motorsd.seed_mag && act_motorsd.seed_mag(2.0f));
    ASSERT(act_motorsd.frame_steps(&fw, &fh));
    ASSERT_EQ(530, fw);   /* 1060 / 2.0 */
    ASSERT_EQ(320, fh);   /* 640 / 2.0 */

    act_motorsd.close();
    server_stop = true;
    pthread_join(server_thread, NULL);
    close(listen_fd);
    unlink(cfg_port);
    PASS();
}

struct FocusWorkerArgs {
    long result;
};

static void *focus_pulse_thread(void *arg) {
    struct FocusWorkerArgs *a = arg;
    a->result = act_motorsd.focus_ms(1, 500);
    return NULL;
}

TEST queue_preserves_stop_and_preempts_focus_and_avoids_move_retry(void) {
    cfg_omit_initial_zoom = false;
    cfg_hold_focus_completion = false;
    cfg_fail_move_once = true;
    snprintf(cfg_port, sizeof cfg_port, "/tmp/test_motorsd_q_%d.sock", (int)getpid());
    unlink(cfg_port);

    listen_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    ASSERT(listen_fd >= 0);
    struct sockaddr_un addr = {.sun_family = AF_UNIX};
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", cfg_port);
    ASSERT_EQ(0, bind(listen_fd, (struct sockaddr *)&addr, sizeof addr));
    ASSERT_EQ(0, listen(listen_fd, 4));

    server_stop = false;
    seen_ops[0] = '\0';
    ASSERT_EQ(0, pthread_create(&server_thread, NULL, mock_server, NULL));
    ASSERT(act_motorsd.open());

    /* A failed non-idempotent move must not be reissued a second time. */
    ASSERT(act_motorsd.move_steps(PTZ_UPRIGHT, 12, 0));
    ASSERT(wait_for_op("\"duration_ms\":120"));
    usleep(50000);
    pthread_mutex_lock(&test_mu);
    const char *first_pt = strstr(seen_ops, "\"pantilt\"");
    ASSERT(first_pt != NULL);
    ASSERT(strstr(first_pt + 1, "\"pantilt\"") == NULL);
    seen_ops[0] = '\0';
    pthread_mutex_unlock(&test_mu);

    /* Enqueue move -> stop -> move back-to-back: FIFO ordering keeps the stop. */
    ASSERT(act_motorsd.move_steps(PTZ_LEFT, 10, 0));
    ASSERT(act_motorsd.emit(PTZ_STOP, 0));
    ASSERT(act_motorsd.move_steps(PTZ_RIGHT, 20, 0));
    ASSERT(wait_for_op("\"right\""));
    pthread_mutex_lock(&test_mu);
    const char *p_left = strstr(seen_ops, "\"left\"");
    const char *p_stop = p_left ? strstr(p_left, "\"stop\"") : NULL;
    const char *p_right = p_stop ? strstr(p_stop, "\"right\"") : NULL;
    ASSERT(p_left != NULL && p_stop != NULL && p_right != NULL);
    seen_ops[0] = '\0';
    pthread_mutex_unlock(&test_mu);

    /* Manual stop aborts an in-flight focus pulse promptly. */
    cfg_hold_focus_completion = true;
    struct FocusWorkerArgs fargs = {.result = 999};
    pthread_t ft;
    ASSERT_EQ(0, pthread_create(&ft, NULL, focus_pulse_thread, &fargs));
    ASSERT(wait_for_op("\"axis\":\"focus\""));
    ASSERT(act_motorsd.emit(PTZ_STOP, 0));
    pthread_join(ft, NULL);
    ASSERT_EQ(-1L, fargs.result);

    act_motorsd.close();
    server_stop = true;
    pthread_join(server_thread, NULL);
    close(listen_fd);
    unlink(cfg_port);
    PASS();
}

SUITE(act_motorsd_suite) {
    RUN_TEST(opens_moves_presets_and_scales_zoom);
    RUN_TEST(refuses_unscaled_frame_steps_until_zoom_reported);
    RUN_TEST(queue_preserves_stop_and_preempts_focus_and_avoids_move_retry);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(act_motorsd_suite);
    GREATEST_MAIN_END();
}
