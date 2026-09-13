// The motor wire, as data rather than as a table of hand-typed bytes.
//
// Two protocols reach the lens controllers OpenIPC meets: the XiongMai
// near-Pelco variant (0xC5 sync, 0x5C terminator, 8 bytes, checksum mod 100)
// and standard Pelco-D (0xFF sync, 7 bytes, checksum mod 256). The command
// BITS are the same in both — they differ only in the wrapper — so one builder
// serves them, and every frame this plugin can put on the wire is derived from
// the same two rules instead of being written out by hand.
//
// That matters: the previous five-frame-per-protocol constant tables are where
// a wrong checksum hid for two years, visible on exactly one frame. `far` is
// the only verb whose byte sum (129) exceeds 100, so it is the only place where
// the two checksum rules disagree -- every other frame is identical under both,
// which is why nothing ever caught it. Frames built here are unit-tested
// byte-for-byte against what the shipped scripts sent.

#ifndef MAJESTIC_AF_PROTO_H
#define MAJESTIC_AF_PROTO_H

#include <stdbool.h>
#include <stddef.h>

// Every verb the wire can carry. The first nine are the viewing controls the
// WebUI pad draws; day/night are lens maintenance (an ICR that sits on the
// serial bus rather than on a GPIO) and are reachable only by name.
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
    PTZ_VERB_COUNT
};

// Longest frame this builder emits (XiongMai: sync + 5 + checksum + terminator).
#define PTZ_FRAME_MAX 8

typedef struct PtzProto PtzProto;

// Look a protocol up by its isp.autofocus.actuator name. NULL, empty or
// unknown yields pelco-xm (the default the key has always carried); an unknown
// name is logged by the caller, not here — this stays free of the HAL.
const PtzProto *ptz_proto(const char *name);
const char *ptz_proto_name(const PtzProto *p);

// Does this protocol carry this verb at all? The XiongMai variant has no
// day/night: its MCU only *reports* them, there is no send command.
bool ptz_proto_has(const PtzProto *p, enum PtzVerb v);

// Build one frame. `speed` is 0..63 and lands in the pan or tilt slot for the
// four direction verbs, ignored by the rest; 0 reproduces exactly the bytes the
// btzoom scripts sent. Returns the frame length, or 0 if the protocol does not
// carry the verb.
int ptz_frame(const PtzProto *p, enum PtzVerb v, int speed,
              unsigned char out[PTZ_FRAME_MAX]);

// Call a stored preset (Pelco-D command 0x07). Used only by the lens-wakeup
// sequence, which calls two vendor presets. Returns the frame length, or 0.
int ptz_preset_frame(const PtzProto *p, int preset,
                     unsigned char out[PTZ_FRAME_MAX]);

// The one-shot wakeup blob each protocol's tool sent once at startup, or NULL.
// Not a Pelco frame — a vendor byte string, passed through as-is.
const unsigned char *ptz_wake_blob(const PtzProto *p, size_t *len);

// Verb <-> name, for the ABI's text interface. ptz_verb_parse returns false on
// anything not in the list above, which is what keeps an unvalidated request
// token from reaching the wire.
const char *ptz_verb_name(enum PtzVerb v);
bool ptz_verb_parse(const char *s, enum PtzVerb *out);

// Classifiers the motion layer reasons with: a focus move invalidates the
// engine's dead reckoning and cancels a booked pass, a zoom move books one.
bool ptz_verb_is_focus(enum PtzVerb v);
bool ptz_verb_is_zoom(enum PtzVerb v);

#endif
