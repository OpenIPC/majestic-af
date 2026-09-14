#ifndef MAJESTIC_AF_MOTOR_H
#define MAJESTIC_AF_MOTOR_H

#include <stdbool.h>

struct motors_client;

enum AfMotorFocusDirection {
    AF_MOTOR_FOCUS_NEAR = -1,
    AF_MOTOR_FOCUS_FAR = 1,
};

enum AfMotorAxis {
    AF_MOTOR_AXIS_FOCUS,
    AF_MOTOR_AXIS_ZOOM,
};

typedef bool (*AfMotorCancelledFn)(void *ctx);
typedef void (*AfMotorMagnificationFn)(void *ctx, float value);

typedef struct {
    struct motors_client *client;
    enum AfMotorAxis axis;
} AfMotor;

enum AfMotorOpenResult {
    AF_MOTOR_OPEN_OK,
    AF_MOTOR_OPEN_BUSY,
    AF_MOTOR_OPEN_ERROR,
};

void af_motor_init(AfMotor *motor);

/* Claim the selected device and open it for motor commands. */
enum AfMotorOpenResult af_motor_open(AfMotor *motor,
                                     enum AfMotorAxis axis,
                                     AfMotorCancelledFn cancelled, void *ctx);

void af_motor_focus(AfMotor *motor, enum AfMotorFocusDirection direction);
bool af_motor_focus_timed(AfMotor *motor,
                          enum AfMotorFocusDirection direction,
                          unsigned duration_ms);
void af_motor_zoom(AfMotor *motor, int direction);
/* Request one logical STOP. The backend applies its device delivery rules. */
void af_motor_stop(AfMotor *motor);

/* Close the device and release its ownership lock. */
void af_motor_close(AfMotor *motor);

/* Process pending service events. True means this AF lease ended. */
bool af_motor_poll(AfMotor *motor);

/* Read optional magnification reports until stop becomes nonzero. */
void af_motor_read_magnification(volatile int *stop,
                                 AfMotorMagnificationFn publish, void *ctx);

#endif
