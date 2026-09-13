// The motor's single owner.
//
// One process, one file descriptor, one mutex: every byte that reaches the
// lens controller goes through motion_*(). Before this existed the wire had two
// writers — this plugin's autofocus pass and the WebUI's btzoom/btzoom-xm shell
// scripts — arbitrated by a mkdir lock, /tmp/btzoom.lock. A lock is not
// arbitration when a movement is three steps (drive, wait, stop) and the other
// writer can arrive in the middle of it: the pass's trailing stop halts the
// operator's move, and dropping that stop instead leaves the motor running.
// Measured on a lab 85H50AI (2026-09-13), the two writers cost the operator
// both halves of it — presses were dropped outright while a pass held the lock
// ("PTZ port is busy", invisible in the WebUI), and presses that did land were
// undone seconds later by the pass the previous zoom had booked.
//
// So manual motion always wins, in-process: it writes its frame immediately and
// an autofocus pass writing through motion_engine_drive() is silently ignored
// for as long as a human is driving. The pass is cancelled in the same breath
// by its caller, and the operator never waits for it.
//
// Motion is press/release rather than a fixed pulse. motion_move() starts the
// motor and arms a deadline; calling it again with the same verb re-sends the
// frame and re-arms. A watchdog thread stops the motor when the deadline
// passes, so a release that never arrives — a closed tab, a dropped link —
// cannot leave a lens driving into its end stop.

#ifndef MAJESTIC_AF_MOTION_H
#define MAJESTIC_AF_MOTION_H

#include <stdbool.h>
#include <stddef.h>

#include "proto.h"

// Open the port named by isp.autofocus.{port,speed} and start the watchdog.
// Idempotent; false if the port cannot be opened (the plugin then has no motor
// and every verb answers "unavailable").
bool motion_start(void);

// Stop the motor, join the watchdog, close the port. Idempotent.
void motion_stop(void);

// The shared descriptor, for the magnification reader — one open, one termios,
// no second configuration of the same tty behind the writer's back. -1 when
// the port is not open.
int motion_fd(void);

// Start or continue a manual move, auto-stopping `ms` from now. A repeat of the
// verb already running re-sends the frame and re-arms the deadline: Pelco
// decoders expect a repeating command, and a re-send also survives a lost
// frame. Returns false if the port is shut or the protocol has no such verb.
// Never blocks.
bool motion_move(enum PtzVerb v, int ms);

// Stop now. Always reaches the wire.
bool motion_halt(void);

// The vendor lens-wakeup sequence, where the protocol has one: the wake blob,
// then the two presets and the ICR exercise the Pelco-D lens tool performed.
// Runs on the caller's thread and takes a few seconds. Returns false if the
// protocol carries no such sequence.
bool motion_wake(void);

// af2's actuator hook: -1 near, 0 stop, +1 far. Writes nothing while a manual
// move owns the wire — the operator's hand outranks the search.
void motion_engine_drive(int dir);

// True while a manual move is running.
bool motion_manual_active(void);

// Milliseconds since manual motion last ended, and 0 while a move is running.
// "The wire has been quiet for N ms" is what "settled" means now that there is
// no lock file to watch for it.
long motion_idle_ms(void);

// The default auto-stop window, isp.autofocus.pulse (ms), clamped to a sane
// range. Also the length of one tap.
int motion_default_ms(void);

// "actuator=pelco-xm port=/dev/ttyAMA0 speed=115200 verbs=..." into `buf`.
const char *motion_describe(char *buf, size_t n);

// --- provided by engine.c, so this layer need not know what a pass is -------

// A manual focus move happened: the dead-reckoned focus position is no longer
// meaningful, and any pass booked by an earlier zoom must not run.
void af_note_manual_focus(void);

// Cancel a running pass without asking for another.
void af_preempt(void);

// Kick a pass (see af.h). Called by the watchdog once a manual zoom settles.
int af_trigger(bool settle);

// The manual-verb entry the plugin ABI lands on: preempt whatever the engine is
// doing, then move. `ms` of 0 means the configured default window.
bool af_ptz_move(enum PtzVerb v, int ms);

#endif
