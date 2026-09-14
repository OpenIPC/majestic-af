// Motor adapter for majestic-af. The AF engine requests logical operations.
// motorsd owns coordination, and its selected driver owns the hardware.

#include "af_motor.h"

#include <stdio.h>
#include <string.h>

#include <majestic/log.h>

#include <libmotors.h>

#define AF_MOTOR_LEASE_MS 120000U

static void report_error(const char *operation, const char *error) {
    log_e("autofocus motor: %s failed: %s", operation,
          error && *error ? error : "unknown error");
}

static enum motors_client_axis client_axis(const AfMotor *motor) {
    return motor->axis == AF_MOTOR_AXIS_ZOOM ? MOTORS_ZOOM : MOTORS_FOCUS;
}

void af_motor_init(AfMotor *motor) {
    motor->client = NULL;
    motor->axis = AF_MOTOR_AXIS_FOCUS;
}

enum AfMotorOpenResult af_motor_open(AfMotor *motor,
                                     enum AfMotorAxis axis,
                                     AfMotorCancelledFn cancelled, void *ctx) {
    char error[160] = "";
    af_motor_init(motor);
    if (cancelled && cancelled(ctx)) return AF_MOTOR_OPEN_BUSY;
    if (motors_open(&motor->client, "/run/motorsd.sock", error, sizeof(error)) != 0) {
        report_error("connect", error);
        return AF_MOTOR_OPEN_ERROR;
    }
    motor->axis = axis;
    enum motors_client_axis requested_axis = client_axis(motor);
    struct motors_capabilities capabilities = {0};
    if (motors_get_capabilities(motor->client, &capabilities,
                                error, sizeof(error)) != 0 ||
        !capabilities.available ||
        !(capabilities.axes & MOTORS_AXIS_MASK(requested_axis))) {
        if (!error[0]) {
            const char *reason = capabilities.available
                                     ? "requested axis is not available"
                                     : "driver is not available";
            snprintf(error, sizeof(error), "%s", reason);
        }
        report_error("capabilities", error);
        motors_close(motor->client);
        motor->client = NULL;
        return AF_MOTOR_OPEN_ERROR;
    }
    if (motors_subscribe(motor->client, error, sizeof(error)) != 0) {
        report_error("subscribe", error);
        motors_close(motor->client);
        motor->client = NULL;
        return AF_MOTOR_OPEN_ERROR;
    }
    if (motors_acquire(motor->client, MOTORS_AF, requested_axis,
                       AF_MOTOR_LEASE_MS, error, sizeof(error)) != 0) {
        bool busy = !strncmp(error, "busy:", 5);
        if (!busy) report_error("acquire", error);
        motors_close(motor->client);
        motor->client = NULL;
        return busy ? AF_MOTOR_OPEN_BUSY : AF_MOTOR_OPEN_ERROR;
    }
    return AF_MOTOR_OPEN_OK;
}

void af_motor_focus(AfMotor *motor, enum AfMotorFocusDirection direction) {
    char error[160] = "";
    enum motors_direction command = direction == AF_MOTOR_FOCUS_NEAR
                                        ? MOTORS_NEAR : MOTORS_FAR;
    if (motors_move(motor->client, MOTORS_FOCUS, command, 0,
                    error, sizeof(error)) != 0)
        report_error("focus", error);
}

bool af_motor_focus_timed(AfMotor *motor,
                          enum AfMotorFocusDirection direction,
                          unsigned duration_ms) {
    char error[160] = "";
    enum motors_direction command = direction == AF_MOTOR_FOCUS_NEAR
                                        ? MOTORS_NEAR : MOTORS_FAR;
    if (!duration_ms ||
        motors_move(motor->client, MOTORS_FOCUS, command, duration_ms,
                    error, sizeof(error)) != 0 ||
        motors_wait_movement(motor->client, MOTORS_FOCUS,
                             (int)duration_ms + 2000, NULL,
                             error, sizeof(error)) != 0) {
        report_error("timed focus", error);
        return false;
    }
    return true;
}

void af_motor_zoom(AfMotor *motor, int direction) {
    char error[160] = "";
    enum motors_direction command = direction > 0 ? MOTORS_TELE : MOTORS_WIDE;
    if (motors_move(motor->client, MOTORS_ZOOM, command, 0,
                    error, sizeof(error)) != 0)
        report_error("zoom", error);
}

void af_motor_stop(AfMotor *motor) {
    char error[160] = "";
    // Preemption already stopped the old movement before motorsd gave the
    // shared controller to the new client. A late cleanup request from this
    // revoked AF session could otherwise stop the new manual movement.
    if (motor->client &&
        !motors_lease_revoked(motor->client, client_axis(motor)) &&
        motors_stop(motor->client, client_axis(motor), error, sizeof(error)) != 0)
        report_error("stop", error);
}

void af_motor_close(AfMotor *motor) {
    if (!motor->client) return;
    char error[160] = "";
    if (motors_release(motor->client, error, sizeof(error)) != 0)
        report_error("release", error);
    motors_close(motor->client);
    af_motor_init(motor);
}

bool af_motor_poll(AfMotor *motor) {
    if (!motor->client) return true;
    char error[160] = "";
    if (motors_poll(motor->client, 0, error, sizeof(error)) < 0) {
        report_error("event", error);
        return true;
    }
    return motors_lease_revoked(motor->client, client_axis(motor));
}

void af_motor_read_magnification(volatile int *stop,
                                 AfMotorMagnificationFn publish, void *ctx) {
    char error[160] = "";
    struct motors_client *client = NULL;
    if (motors_open(&client, "/run/motorsd.sock", error, sizeof(error)) != 0) {
        report_error("telemetry connect", error);
        return;
    }
    if (motors_subscribe(client, error, sizeof(error)) != 0) {
        report_error("telemetry subscribe", error);
        motors_close(client);
        return;
    }

    while (!*stop) {
        int result = motors_poll(client, 250, error, sizeof(error));
        if (result < 0) {
            report_error("telemetry", error);
            break;
        }
        float magnification = 0.0f;
        if (result > 0 &&
            motors_zoom_magnification(client, &magnification, NULL))
            publish(ctx, magnification);
    }
    motors_close(client);
}
