// VISCA backend — the pure, hardware-free helpers, exposed so the unit test
// (tests/visca_test.c) can pin every byte and the magnification curve without a
// camera. The tty and the reader thread stay private to src/act_visca.c.
//
// The lens this was built against is the Xiongmai N51813L 18x zoom block
// (Hi3518CV100): its lens MCU sits on UART0 at 9600 8N1 and takes plain Sony
// VISCA at camera address 1. The frames below are what the stock firmware sends
// (captured from Sofia's own writes), and the magnification table was measured on
// that block with CAM_Zoom Direct and CAM_ZoomPosInq.

#ifndef MAJESTIC_AF_ACT_VISCA_H
#define MAJESTIC_AF_ACT_VISCA_H

#include <stdbool.h>
#include <stddef.h>

#include "proto.h"

// The longest frame this backend sends: a verb can be two VISCA commands
// (focus mode, then the focus move), 6 bytes each.
#define VISCA_FRAME_MAX 16

// The zoom/focus speed nibble, 0 (slowest) .. 7. 5 is what the stock firmware
// sends for its default DVRIP step.
#define VISCA_SPEED_DEFAULT 5

// Bytes for one verb. `focus_manual` is the lens's focus mode as the backend last
// set it: a focus move first switches the lens to manual (CAM_Focus Manual), and a
// zoom first hands focus back to the lens (CAM_Focus Auto), as the stock firmware
// does. `*now_manual` is set to the mode after these bytes. STOP halts both the
// zoom and the focus drive. Returns the length, 0 if the verb is not carried.
int visca_frame(enum PtzVerb v, int speed, bool focus_manual, bool *now_manual,
                unsigned char out[VISCA_FRAME_MAX]);

bool visca_has(enum PtzVerb v);

// CAM_ZoomPosInq, 81 09 04 47 FF.
int visca_zoom_inquiry(unsigned char out[VISCA_FRAME_MAX]);

// Parse one FF-terminated frame (the FF included). A zoom position reply
// (90 50 0p 0q 0r 0s FF) sets *pos and returns 1. The lens MCU's own status frame
// (BB 82 .. FF, 13 bytes) sets *mag to the integer magnification it carries in
// bytes 6..7 and returns 2. Anything else returns 0.
int visca_parse(const unsigned char *f, size_t n, int *pos, int *mag);

// Magnification at a CAM_Zoom position (0x0000 wide .. 0x4000 tele), interpolated
// in the measured table.
float visca_pos_to_mag(int pos);

// The MCU's exposure in its status frame (BB 82 .. FF, 13 bytes): be16 bytes
// 2..3 is its sensor gain, 0 in bright light and rising as the light falls
// (0x13e at one 5 W bulb's 20 %), be16 bytes 4..5 its shutter, which caps at
// 0x3de before the gain moves. Measured on the N51813L with the zoom held
// still. False for any other frame.
bool visca_status(const unsigned char *f, size_t n, int *gain, int *shutter);

// The MCU's gain word as an ISO (100 = no gain), taking the word as tenths of a
// dB -- the scale is an assumption, not a measurement; what is measured is that
// the word only rises as the light falls, and the board's ISO tables are fitted
// to the ISOs this gives.
unsigned visca_gain_to_iso(int gain);

#endif
