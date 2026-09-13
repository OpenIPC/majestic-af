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

// Teardown, in two halves, because the order matters. The watchdog can start an
// autofocus pass (af_book_tick), so it has to be joined BEFORE the engine joins
// its worker — otherwise it can spawn one into a shutdown that has already
// decided there was nothing to join, and that thread outlives the dlclose.
// The port closes last, after both the worker and the reader are joined.
void motion_stop_watchdog(void);
void motion_close(void);

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

// af2's actuator hook: -1 near, 0 stop, +1 far. Returns FALSE when it wrote
// nothing because a manual move owns the wire. The caller must not carry on as
// if the lens had moved: af2 integrates dead-reckoned position and samples the
// focus statistic from its own commands, so a silently dropped move makes the
// rest of that pass fiction — and the lens would start obeying it again the
// moment the operator let go.
bool motion_engine_drive(int dir);

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
// meaningful, any pass booked by an earlier zoom must not run, and a pass
// already running is cancelled.
void af_note_manual_focus(void);

// Cancel a running pass without asking for another, and drop any restart an
// earlier trigger had queued.
void af_preempt(void);

// The same, unconditionally — for a move the actuator hook could not deliver.
void af_preempt_always(void);

// The manual-focus generation. A booking taken at one value is refused if it
// has moved by the time it is armed.
unsigned af_focus_gen(void);

// The after-zoom follow-up focus. The engine owns the booking so that a manual
// focus can revoke it in the same critical section that would start it;
// motion.c only says when a zoom stopped moving and ticks the clock.
// af_book_tick returns true when it started the booked pass.
void af_book_after_zoom(long at_ms, unsigned gen);
bool af_book_tick(long now);

// The manual-verb entry the plugin ABI lands on: preempt whatever the engine is
// doing, then move. `ms` of 0 means the configured default window.
bool af_ptz_move(enum PtzVerb v, int ms);

#endif
