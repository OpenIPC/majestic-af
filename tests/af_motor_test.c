#include "af_motor.h"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <libmotors.h>
#include <majestic/log.h>

struct motors_client { int unused; };

static struct motors_client test_client;
static const char *opened_path;
static enum motors_client_role acquired_role;
static enum motors_client_axis acquired_axis;
static unsigned acquired_lease;
static enum motors_client_axis moved_axis;
static enum motors_direction moved_direction;
static unsigned moved_duration;
static enum motors_client_axis stopped_axis;
static int stop_count;
static int release_count;
static int close_count;
static int subscribe_count;
static int capabilities_count;
static bool revoked;
static uint32_t capability_axes = MOTORS_AXIS_MASK(MOTORS_FOCUS) |
                                  MOTORS_AXIS_MASK(MOTORS_ZOOM);
static bool driver_available = true;

void log_log(enum LogType level, const char *file, int line, const char *func,
             const char *format, ...) {
    (void)level; (void)file; (void)line; (void)func; (void)format;
}

int motors_open(struct motors_client **client, const char *socket_path,
                char *error, size_t error_size) {
    (void)error; (void)error_size;
    opened_path = socket_path;
    *client = &test_client;
    return 0;
}

void motors_close(struct motors_client *client) {
    assert(client == &test_client);
    ++close_count;
}

int motors_get_capabilities(struct motors_client *client,
                            struct motors_capabilities *capabilities,
                            char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    *capabilities = (struct motors_capabilities){
        .axes = capability_axes,
        .raw = true,
        .available = driver_available,
    };
    snprintf(capabilities->driver, sizeof(capabilities->driver), "mock");
    ++capabilities_count;
    return 0;
}

int motors_acquire(struct motors_client *client, enum motors_client_role role,
                   enum motors_client_axis axis, unsigned lease_ms,
                   char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    acquired_role = role;
    acquired_axis = axis;
    acquired_lease = lease_ms;
    return 0;
}

int motors_move(struct motors_client *client, enum motors_client_axis axis,
                enum motors_direction direction, unsigned duration_ms,
                char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    moved_axis = axis;
    moved_direction = direction;
    moved_duration = duration_ms;
    return 0;
}

int motors_stop(struct motors_client *client, enum motors_client_axis axis,
                char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    stopped_axis = axis;
    ++stop_count;
    return 0;
}

int motors_release(struct motors_client *client, char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    ++release_count;
    return 0;
}

int motors_subscribe(struct motors_client *client, char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    ++subscribe_count;
    return 0;
}

int motors_poll(struct motors_client *client, int timeout_ms,
                char *error, size_t error_size) {
    (void)error; (void)error_size;
    assert(client == &test_client);
    assert(timeout_ms == 0);
    return 0;
}

bool motors_lease_revoked(const struct motors_client *client,
                          enum motors_client_axis axis) {
    assert(client == &test_client);
    assert(axis == (acquired_axis == MOTORS_ZOOM ? MOTORS_ZOOM : MOTORS_FOCUS));
    return revoked;
}

bool motors_zoom_magnification(const struct motors_client *client,
                               float *magnification,
                               uint64_t *observed_mono_ms) {
    (void)client;
    (void)magnification;
    (void)observed_mono_ms;
    return false;
}

int motors_wait_movement(struct motors_client *client,
                         enum motors_client_axis axis, int timeout_ms,
                         uint64_t *completed_mono_ms,
                         char *error, size_t error_size) {
    (void)completed_mono_ms; (void)error; (void)error_size;
    assert(client == &test_client);
    assert(axis == MOTORS_FOCUS);
    assert(timeout_ms == 2070);
    return 0;
}

int main(void) {
    AfMotor motor;
    assert(af_motor_open(&motor, AF_MOTOR_AXIS_FOCUS, NULL, NULL) ==
           AF_MOTOR_OPEN_OK);
    assert(!strcmp(opened_path, "/run/motorsd.sock"));
    assert(acquired_role == MOTORS_AF);
    assert(acquired_axis == MOTORS_FOCUS);
    assert(acquired_lease == 120000);
    assert(capabilities_count == 1);
    assert(subscribe_count == 1);
    assert(!af_motor_poll(&motor));
    revoked = true;
    assert(af_motor_poll(&motor));
    revoked = false;

    af_motor_focus(&motor, AF_MOTOR_FOCUS_NEAR);
    assert(moved_axis == MOTORS_FOCUS && moved_direction == MOTORS_NEAR &&
           moved_duration == 0);
    af_motor_focus(&motor, AF_MOTOR_FOCUS_FAR);
    assert(moved_axis == MOTORS_FOCUS && moved_direction == MOTORS_FAR);
    assert(af_motor_focus_timed(&motor, AF_MOTOR_FOCUS_NEAR, 70));
    assert(moved_axis == MOTORS_FOCUS && moved_direction == MOTORS_NEAR &&
           moved_duration == 70);
    af_motor_zoom(&motor, 1);
    assert(moved_axis == MOTORS_ZOOM && moved_direction == MOTORS_TELE);
    af_motor_zoom(&motor, -1);
    assert(moved_axis == MOTORS_ZOOM && moved_direction == MOTORS_WIDE);
    af_motor_stop(&motor);
    assert(stopped_axis == MOTORS_FOCUS);
    assert(stop_count == 1);
    revoked = true;
    af_motor_stop(&motor);
    assert(stop_count == 1);
    revoked = false;
    af_motor_close(&motor);
    assert(release_count == 1 && close_count == 1 && motor.client == NULL);

    assert(af_motor_open(&motor, AF_MOTOR_AXIS_ZOOM, NULL, NULL) ==
           AF_MOTOR_OPEN_OK);
    assert(acquired_axis == MOTORS_ZOOM);
    assert(subscribe_count == 2);
    af_motor_stop(&motor);
    assert(stopped_axis == MOTORS_ZOOM);
    assert(stop_count == 2);
    af_motor_close(&motor);

    int subscriptions_before = subscribe_count;
    int closes_before = close_count;
    capability_axes = MOTORS_AXIS_MASK(MOTORS_FOCUS);
    assert(af_motor_open(&motor, AF_MOTOR_AXIS_ZOOM, NULL, NULL) ==
           AF_MOTOR_OPEN_ERROR);
    assert(subscribe_count == subscriptions_before);
    assert(close_count == closes_before + 1);
    assert(motor.client == NULL);

    driver_available = false;
    assert(af_motor_open(&motor, AF_MOTOR_AXIS_FOCUS, NULL, NULL) ==
           AF_MOTOR_OPEN_ERROR);
    assert(subscribe_count == subscriptions_before);
    assert(close_count == closes_before + 2);
    assert(motor.client == NULL);
    puts("motorsd adapter tests passed");
    return 0;
}
