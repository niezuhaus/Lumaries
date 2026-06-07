# Software Architecture

This document describes how the firmware in [`lumaries.ino`](lumaries.ino) is structured and
why. It is aimed at someone modifying or extending the installation — for end-user operation
see the [README](README.md).

---

## 1. Overview

The firmware is a **single-translation-unit Arduino sketch** (one `.ino`, no separate headers)
running on an ATmega2560. It drives two stepper motors and a UV laser to draw on a
phosphorescent canvas, and an RS485/DMX light fixture for ambient lighting.

Three design constraints shape everything:

- **8-bit AVR, 8 KB RAM, 256 KB flash.** Constant data (fonts, gamma table) lives in `PROGMEM`;
  the only large RAM consumer is the recording buffer, which is statically bounded.
- **Cooperative single loop.** There is no RTOS and no interrupts beyond the Arduino core. All
  timing is `millis()`/`micros()`-based and the program is a classic `setup()`/`loop()` superloop.
- **Long-running drawing must stay responsive.** A single circle or spiral can take many seconds
  to draw. Rather than fully event-driven motion, the code uses *blocking draw primitives that
  pump the serial parser internally* and bail out via a global interrupt flag (see §6).

The whole program is organized into labelled sections within the one file, roughly in
dependency order: tunable parameters → state declarations → font data → laser → LED → buttons →
jog → calibration → mode switching → motion primitives → draw modes → font rendering →
recording → manual mode → DMX/day mode → playback → serial interface → auto-cycle → setup/loop.

---

## 2. Execution model: two-tier state machine

The system has two layers of state.

### Tier 1 — `SystemState` (lifecycle)

```
CALIBRATE_LEFT → CALIBRATE_RIGHT → CALIBRATE_TOP → CALIBRATE_BOTTOM → READY
```

`loop()` branches on this first: anything other than `READY` runs `handleCalibrationState()`
(jog + button-to-save-point + serial). Once `READY`, the laser parks at canvas centre and a
**first-touch gate** (`waitingForFirstTouch`) holds the show until the visitor moves the
joystick — then it drops into `MODE_MANUAL`.

### Tier 2 — `DrawMode` (the show, only meaningful when `READY`)

```
MODE_CIRCLE  MODE_MANUAL  MODE_PLAYBACK  MODE_SPIRAL
MODE_CORNERS MODE_RAIN    MODE_LETTERS   MODE_PLACE
```

`setDrawMode()` is the single entry point for switching: it stops the motors, kills the laser,
finalizes any in-progress recording, sets per-mode motor speeds, and resets `modeStartMs` /
`inInterMode`. The active mode's handler is dispatched once per `loop()` iteration from a
`switch` in `loop()`.

Two further *transient* state machines run on top of the draw modes when active:
**day mode** (`DayPhase`, §11) and its nested **cloud animation** (`CloudPhase`, §11). When day
mode is active it takes over the loop body entirely until it finishes or is cancelled.

---

## 3. Coordinate systems

| Space | Units | Where used |
|---|---|---|
| **Canvas-normalized** | `float` 0.0–1.0 in each axis | all drawing logic, font rendering, recordings |
| **Serial-normalized** | `int` 0–1000 (thousandths) | serial `M`/`D` commands, recording storage (`uint16_t`) |
| **Steps** | `long` motor steps | AccelStepper, calibration bounds |

Calibration captures the four corner positions in steps (`X_left/X_right/Y_top/Y_bot`),
normalizes them so `left < right` / `top < bottom`, and derives `canvas_width_steps` /
`canvas_height_steps`. Conversion from canvas-normalized to steps is centralized in two helpers:

```c
long normToStepsX(float n) { return X_left + (long)(constrain(n,0,1) * canvas_width_steps); }
long normToStepsY(float n) { return Y_top  + (long)(constrain(n,0,1) * canvas_height_steps); }
```

Everything above the motion layer thinks in 0.0–1.0; only these two functions (and the inverse
`map()` calls used when recording) know about steps. Note the two `AccelStepper` instances are
deliberately cross-wired (`xStepper` ← `Y_STEP/Y_DIR`, `yStepper` ← `X_STEP/X_DIR`) to match the
physical gantry orientation — a hardware mapping detail isolated to the instance declarations.

---

## 4. Motion layer

Built on the **AccelStepper** library, used in two distinct regimes:

- **Velocity / jog mode** — `setSpeed()` + `runSpeed()` (constant speed, no acceleration ramp).
  Used for live joystick jogging during calibration (`jogMotors()`) and `MODE_MANUAL`
  (`handleManualMode()`), with software clamping at the canvas edges.
- **Position mode** — `moveTo()` + `run()` with acceleration. Used for all generative drawing,
  wrapped in blocking primitives:

  | Primitive | Behaviour |
  |---|---|
  | `moveToCanvas(x,y)` | laser off, move to point, wait |
  | `drawToCanvas(x,y,p)` | laser on, move, wait, laser off |
  | `drawLine(...)` | move to start, draw to end |
  | `drawSegTo(x,y)` | move with laser **left as-is** — for chaining multi-segment strokes |
  | `waitForMotors()` | spin `run()` until `distanceToGo()==0`, pumping serial each iteration |

Three speed regimes are selected per mode in `setDrawMode()`: `JOG_MAX_SPEED` (calibration/
manual/place), `DRAW_MAX_SPEED` (detailed generative modes), `PLAYBACK_SPEED`, plus
`RAIN_DRAW_SPEED` and the per-ring speed ramp in spiral mode.

---

## 5. The cooperative loop & blocking calls

`loop()` runs one "unit of work" per iteration:

- Non-blocking modes (`MANUAL`, `PLACE`) do a small amount of work and return immediately, so
  the loop spins fast and stays responsive.
- Blocking modes (`CIRCLE`, `SPIRAL`, `RAIN`, `CORNERS`, `LETTERS`, `PLAYBACK`) call the motion
  primitives above, which *block* inside `waitForMotors()` for the duration of a move.

To keep serial responsive even during a long blocking move, **`waitForMotors()` calls
`handleSerialInput()` on every step-tick.** This is the key to the whole concurrency model:
the serial parser runs "inside" blocking draws.

---

## 6. Interruption model

Because a draw can block for seconds, there must be a way to abort it. That is the global
`serialInterrupt` flag:

1. A serial command (or button event, or day-mode trigger) sets `serialInterrupt = true`.
2. `waitForMotors()` sees the flag, stops the motors, and returns early.
3. The draw routine checks the flag and bails. The repetitive generative modes use a
   `*_CHECK()` macro (`RAIN_CHECK`, `SPIRAL_CHECK`, `CIRCLE_CHECK`) that turns the laser off
   and `return`s if `serialInterrupt` is set **or** the mode changed out from under it.
4. Back in `loop()`, the pending serial command (`pendingSerial` / `pendingWord`) is executed,
   then `serialInterrupt` is cleared at the bottom of the iteration.

So `serialInterrupt` is a one-shot "drop what you're doing and re-read intent" signal. It is the
single mechanism behind serial-driven mode switches, the `M`/`D`/`H` movement commands,
long-press spot mode, and day-mode entry.

---

## 7. Laser subsystem

`writeLaser(val)` is the only function that touches the PWM pin. It applies a **gamma 2.2
lookup table** (`GAMMA_TABLE` in PROGMEM) so brightness is perceptually linear, and mirrors a
clamped copy onto the recording-indicator LED while `READY`. `setLaser(power)` is the
higher-level call: it tracks `laserEnabled` and writes either `currentLaserPower` or 0.
`currentLaserPower` is the global brightness setpoint (settable over serial with `L`).

---

## 8. Input subsystem

- **Joystick** → two analog potis read with a centre dead-zone (`POTI_MIN`/`POTI_MAX`).
  `joystickMoved()` is the idle/activity detector used all over (recording start/stop, day-mode
  cancel, first-touch gate). `potiToSpeed()` maps raw reading → signed speed.
- **Joystick button** → `checkButtonEvent()` is a small debounced detector returning
  `BTN_SHORT` (on release) or `BTN_LONG` (fires *while held* past `LONG_PRESS_MS`). Short press
  saves calibration points; long press enters "spot mode" (motors halt, Y axis sets brightness).
- **Prev/Next mode buttons** are wired and read but currently **deactivated** in
  `checkModeButtons()`; mode stepping happens over serial (`N`/`P`).

---

## 9. Draw modes

All generative modes share the same shape: one `loop()` dispatch = one quantum of drawing,
repeated until interrupted.

| Mode | Kind | Per-iteration work | Auto-transition |
|---|---|---|---|
| `CIRCLE` | generative | one full distorted revolution; phases advance each call so it slowly morphs | none (button/serial) |
| `SPIRAL` | generative | draws a full rectangular spiral to the edge | **self-advances** via `advanceAutoMode()` on reaching the edge |
| `RAIN` | generative | one falling streak + chance of a DMX lightning flash | none |
| `CORNERS` | generative | one random 90° corner | none |
| `LETTERS` | generative | one random glyph at random position/scale | none |
| `MANUAL` | interactive | jog + record + idle-breathing pulse | idle → recording auto-saves |
| `PLACE` | interactive | dim positioning dot; serial text draws at cursor | none |
| `PLAYBACK` | replay | replays one recording (see §10) | inter-mode countdown → `postInterMode` |

Generative geometry is parameter-driven (the `CIRCLE_*`, `SPIRAL_*`, `RAIN_*`, `CORNER_*`,
`LETTER_*` constants), keeping the algorithms tunable without code changes.

---

## 10. Recording & playback subsystem

This is the interactive heart of the piece: visitor gestures are captured and woven back into
the autonomous show over time.

**Storage** (statically allocated, the main RAM budget):

```c
struct RecPoint { uint16_t x; uint16_t y; uint8_t laser; };   // 5 bytes
RecPoint recordings[MAX_RECORDINGS][MAX_REC_POINTS];          // 20 × 30 × 5 = 3 KB
uint8_t  recLengths[], recAges[], recTimesPlayed[];
```

**Capture** — in `MODE_MANUAL`, the first joystick movement after entering the mode auto-starts
a recording (`autoRecordEnabled` edge-trigger). Positions are sampled every `REC_SAMPLE_MS`
into the active slot (with the laser on/off bit). Recording auto-stops and saves on joystick
idle (`REC_IDLE_STOP_MS`) or when the point buffer fills.

**Replay with aging** — `handlePlaybackMode()` picks a recording via `pickPlaybackIdx()`
(weighted random favouring **newer** recordings, avoiding immediate repeats), then replays its
points. Recordings have an **age** that grows each time they are told (after a grace period of
`PLAYBACK_FREE_TELLINGS` clean playbacks); age maps to a noise amplitude that `deformCoord()`
adds to every coordinate. The result: a gesture starts crisp and gradually decays into
abstraction the more often it is shown — a deliberate "fading memory" aesthetic governed by the
`PLAYBACK_*` constants.

---

## 11. Auto-cycle, day mode & DMX

**Auto-cycle / inter-mode** — `advanceAutoMode()` steps through the `AUTO_MODES` rotation
(which excludes `MANUAL`/`PLACE`). When recordings exist, it inserts a short **inter-mode
playback** burst (1…`INTERMODE_PLAYBACK_MAX` recordings) before the next mode, tracked by
`inInterMode` / `interModeLeft` / `postInterMode`. Today this is invoked when spiral reaches the
edge and after inter-mode playback completes; `modeStartMs`/`AUTO_CYCLE_MS` back the status
read-out of time-in-mode.

**Day mode** — an idle ambience triggered by `checkDayModeTrigger()` after `DAY_IDLE_MS` with no
recent recording activity (and never during `MANUAL`). It is a **non-blocking phase machine**
(`DayPhase`: `FADE_IN → CLOUDS → FADE_OUT → NONE`) run from `loop()`, so the joystick can cancel
it at any time. It crossfades the laser down while bringing the DMX fixture up, runs a cloud
simulation, then crossfades back.

**Cloud simulation** — a nested state machine (`CloudPhase`: `SUNNY → FADING_OUT → COVERED →
FADING_IN`) driven by randomly chosen `CLOUD_PRESETS`, emitting a DMX level ~33 Hz.

**DMX transport** — `dmxSend()` bit-bangs a DMX512 frame on `Serial1`: it drops the TX line to
generate the BREAK + Mark-After-Break, then transmits the start code and channel data at
250 kbaud `8N2` via RS485 (`DMX_PIN` holds the transceiver in transmit). It writes channels up
to `DMX_ADDRESS`, only the addressed one carrying the value. Lightning flashes in `RAIN` mode
reuse the same path.

---

## 12. Font rendering

Text uses the **KiCad Newstroke** single-stroke font, stored as compact PROGMEM strings.
Format: each character pair encodes a value `v = char - 'R'`; the first pair is the
(left, right) bearing, subsequent pairs are stroke waypoints, and `" R"` (space) is a pen-up
marker. `drawGlyphNS()` decodes a glyph straight from PROGMEM into canvas-space line segments,
scaling by a caller-supplied factor.

- `NS_TABLE[]` — A–Z; `NS_DIGITS[]` — 0–9; plus period/hyphen/colon glyphs.
- German umlauts (Ä Ö Ü ß) are **not** in Newstroke, so they fall back to the older hand-crafted
  `int8_t` 8×12 format (`UMLAUT_TABLE`, decoded by `drawGlyphOld()`).
- `drawSerialWord()` lays out a centred word for `LETTERS` mode; `drawWordAt()` places text at an
  arbitrary cursor for `PLACE` mode.

---

## 13. Serial command interface

`handleSerialInput()` accumulates newline-terminated lines into a fixed buffer and hands them to
`parseSerialCommand()`, a single-letter dispatcher (`M D H L S N P C K`, plus context-sensitive
text/number handling in `LETTERS`/`PLACE`). See the [README](README.md#serial-command-reference)
for the command table. Movement/draw commands don't act immediately — they set `pendingSerial`
+ `serialInterrupt` and are executed at the bottom of `loop()` so they cleanly preempt whatever
was drawing. `reportStatus()` (`S`) prints a multi-line snapshot of mode, day-mode countdown,
recordings, laser, canvas and position.

---

## 14. Memory budget

- **Flash (PROGMEM):** font tables, gamma LUT, cloud presets, all the `F()`-wrapped strings.
- **RAM:** dominated by `recordings[]` (~3 KB at the default 20×30 bounds). Tuning
  `MAX_RECORDINGS` / `MAX_REC_POINTS` is the main lever on RAM use. Everything else is small
  fixed globals — there is no dynamic allocation anywhere.

---

## 15. Control-flow summary

```
setup()
  └─ init pins, steppers, laser; print banner; state = CALIBRATE_LEFT

loop()  ── if not READY ───────────────────────────────────────────────
  └─ handleCalibrationState(): jog + save-point + serial

loop()  ── if READY ───────────────────────────────────────────────────
  ├─ handleSerialInput()                  (also pumped inside blocking draws)
  ├─ button events → spot mode / long-press
  ├─ first-touch gate → MODE_MANUAL
  ├─ if spot mode:      laser brightness from joystick
  ├─ else if day mode:  handleDayMode()   (cancellable phase machine)
  ├─ else if !interrupt:
  │     checkDayModeTrigger(), else dispatch current DrawMode handler
  ├─ execute pending serial move / word
  └─ clear serialInterrupt; updateLedBlink()
```
