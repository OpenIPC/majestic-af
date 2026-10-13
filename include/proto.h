// The motor wire, as data rather than as a table of hand-typed bytes.
//
// Two protocols reach the lens controllers OpenIPC meets: the XiongMai
// near-Pelco variant (0xC5 sync, 0x5C terminator, 8 bytes) and standard
// Pelco-D (0xFF sync, 7 bytes), both with a mod-256 checksum. The command BITS
// are the same in both but for focus, whose two bits the XiongMai lens board
// reads the other way round — so one builder serves them, and every frame this
// plugin can put on the wire is derived from the same rules instead of being
// written out by hand. Frames built here are unit-tested byte-for-byte: the
// Pelco-D ones against what the shipped btzoom script sent, the XiongMai ones
// against what the stock firmware sends its lens board.

#ifndef MAJESTIC_AF_PROTO_H
#define MAJESTIC_AF_PROTO_H

#include <stdbool.h>
#include <stddef.h>

// Every verb the wire can carry. The first nine are the viewing controls the
// WebUI pad draws; day/night are lens maintenance (an ICR that sits on the
// serial bus rather than on a GPIO) and are reachable only by name. The four
// diagonals move pan and tilt at once (the WebUI's thumbstick): one Pelco frame
// with both direction bits, or a gpiostep head stepping its two axes in turn.
enum PtzVerb {
    PTZ_STOP = 0,
    PTZ_NEAR,
    PTZ_FAR,
    PTZ_TELE,
    PTZ_WIDE,
    PTZ_UP,
    PTZ_DOWN,
    PTZ_LEFT,
    PTZ_RIGHT,
    PTZ_DAY,
    PTZ_NIGHT,
    PTZ_UPLEFT,
    PTZ_UPRIGHT,
    PTZ_DOWNLEFT,
    PTZ_DOWNRIGHT,
    PTZ_VERB_COUNT
};

// Longest frame this builder emits (XiongMai: sync + 5 + checksum + terminator).
#define PTZ_FRAME_MAX 8

typedef struct PtzProto PtzProto;

// Look a protocol up by its ptz.actuator name. NULL, empty or
// unknown yields pelco-xm (the default the key has always carried); an unknown
// name is logged by the caller, not here — this stays free of the HAL.
const PtzProto *ptz_proto(const char *name);
const char *ptz_proto_name(const PtzProto *p);

// Does this protocol carry this verb at all? The XiongMai variant has no
// day/night: its MCU only *reports* them, there is no send command.
bool ptz_proto_has(const PtzProto *p, enum PtzVerb v);

// The device address every frame carried before it could be set, and what
// the ptz.address key defaults to: what a Pelco-D head strapped to factory
// settings answers at. The XiongMai lens board ignores the byte (85H50AI,
// 2026-10-13: it zoomed the same at 1 and at 2).
#define PTZ_ADDR_DEFAULT 1

// The address to put on the wire for the configured one: itself when it is a
// Pelco address (0..255), PTZ_ADDR_DEFAULT when it is not -- unset reads as
// INT_MIN -- with *ok false then, so the caller can say so. ok may be NULL.
int ptz_addr(int configured, bool *ok);

// Build one frame for the device at `addr` (Pelco-D's receiver address; the
// XiongMai variant has the same byte in the same place). `speed` is 0..63 and
// lands in the pan or tilt slot for the four direction verbs, ignored by the
// rest; 0 reproduces exactly the bytes the btzoom scripts sent. Returns the
// frame length, or 0 if the protocol does not carry the verb.
int ptz_frame(const PtzProto *p, unsigned char addr, enum PtzVerb v, int speed,
              unsigned char out[PTZ_FRAME_MAX]);

// Call a stored preset (Pelco-D command 0x07) on the device at `addr`. Used
// only by the lens-wakeup sequence, which calls two vendor presets. Returns the
// frame length, or 0.
int ptz_preset_frame(const PtzProto *p, unsigned char addr, int preset,
                     unsigned char out[PTZ_FRAME_MAX]);

// The one-shot wakeup blob each protocol's tool sent once at startup, or NULL.
// Not a Pelco frame — a vendor byte string, passed through as-is, whatever the
// address: it was captured from the vendor's tool, and which of its bytes, if
// any, is an address is not known.
const unsigned char *ptz_wake_blob(const PtzProto *p, size_t *len);

// Verb <-> name, for the ABI's text interface. ptz_verb_parse returns false on
// anything not in the list above, which is what keeps an unvalidated request
// token from reaching the wire.
const char *ptz_verb_name(enum PtzVerb v);
bool ptz_verb_parse(const char *s, enum PtzVerb *out);

// The core's `ptz` command, "<verb>[:<ms>[:<speed>]]": the verb (validated as
// above), how long to run it (0 when absent; 0..100000 ms), and how fast as a
// percentage of the axis's top rate (0 when absent, meaning the top rate;
// 1..100 when given). False for anything else -- a number that is not plain,
// one out of range, a trailing field -- which the core turns into a 400.
typedef struct PtzCommand {
    enum PtzVerb verb;
    long ms;
    int speed;
} PtzCommand;
bool ptz_command_parse(const char *s, PtzCommand *out);

// The core's `ptzstep` command, "<verb>:<steps>[:<speed>]": a pan/tilt verb, a
// number of steps (1..PTZ_STEPS_MAX) and an optional speed as for `ptz`
// (1..100, 0 when absent). False for anything else, and for verbs that are not
// pan or tilt -- there is nothing to count on a zoom or a focus here.
#define PTZ_STEPS_MAX 1000
typedef struct PtzStepCommand {
    enum PtzVerb verb;
    int steps;
    int speed;
} PtzStepCommand;
bool ptz_step_command_parse(const char *s, PtzStepCommand *out);

// Classifiers the motion layer reasons with: a focus move invalidates the
// engine's dead reckoning and cancels a booked pass, a zoom move books one.
bool ptz_verb_is_focus(enum PtzVerb v);
bool ptz_verb_is_zoom(enum PtzVerb v);
bool ptz_verb_is_pantilt(enum PtzVerb v);

// Which way a pan/tilt verb turns each axis: *pan -1 left, +1 right, *tilt +1
// up, -1 down, 0 where it leaves that axis alone. False for every other verb.
bool ptz_verb_dirs(enum PtzVerb v, int *pan, int *tilt);

#endif
