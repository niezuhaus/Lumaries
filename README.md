# Lumaries — Laser Canvas

Firmware for a two-axis UV-laser drawing installation. An Arduino Mega 2560 + RAMPS 1.4
drives two stepper motors that steer a laser across a phosphorescent canvas, leaving
glowing trails. The installation runs autonomously — cycling through generative drawing modes —
and also lets a visitor take over with a joystick to draw freehand, with their gestures
recorded and replayed back into the mix over time.

The main sketch is [`lumaries.ino`](lumaries.ino).

---

## Hardware

| Part | Detail |
|---|---|
| Controller | Arduino Mega 2560 |
| Motor shield | RAMPS 1.4 (X + Y stepper drivers) |
| Laser | UV laser on a PWM pin (`LASER_PIN`, pin 45) |
| Input | 2-axis joystick → analog potis (`A12`/`A11`) + click button (pin 42) |
| Buttons | Prev / Next mode (pins 50 / 51) |
| Indicator | Recording LED, PWM (pin 2) |
| DMX | RS485 out on `Serial1` TX (pin 18), direction-enable on pin 4 — drives an ambient light fixture (base channel `DMX_ADDRESS`) |

Pin assignments live at the top of [`lumaries.ino`](lumaries.ino). The canvas is a
rectangular area whose corners are found at startup by calibration.

---

## Build & upload

1. Install the **Arduino IDE** (or `arduino-cli`).
2. Install the **AccelStepper** library (the only external dependency).
3. Select board **Arduino Mega 2560**.
4. Open [`lumaries.ino`](lumaries.ino), compile, and upload.
5. Open the Serial Monitor at **9600 baud** to see status output and send commands.

---

## Operation

### Calibration

On boot the laser turns on and the system walks through a four-point calibration to learn
the canvas bounds:

```
CALIBRATE_LEFT → CALIBRATE_RIGHT → CALIBRATE_TOP → CALIBRATE_BOTTOM → READY
```

Jog the laser with the joystick; **hold the joystick button** (or press Space over serial)
to save each limit. Send `K` to skip calibration and reuse the last known canvas size
(`SAVED_CANVAS_W` / `SAVED_CANVAS_H`), or `C` to restart calibration at any time.

After calibration the laser parks at canvas centre and waits for the first joystick touch
before the autonomous show begins.

### Controls

- **Joystick** — jog the laser. In `MANUAL` mode this draws freehand and records the gesture.
- **Joystick button (short)** — saves a calibration point (during calibration).
- **Joystick button (long-press)** — "spot" mode: motors stop, laser stays on with brightness set by the Y axis. Release to end.
- **Prev / Next buttons** — step backwards/forwards through draw modes.

### Draw modes

When `READY`, the installation cycles automatically through these modes
(`AUTO_CYCLE_MS`, ~2 min each), with short bursts of recorded-shape playback between cycles:

| Mode | Description |
|---|---|
| `CIRCLE` | Sine-wave-distorted circle, slowly morphing each revolution |
| `MANUAL` | Joystick freehand drawing with the laser on; idle gestures are recorded |
| `PLAYBACK` | Replays stored recordings, adding noise as they "age" |
| `SPIRAL` | Rectangular spiral expanding from canvas centre |
| `CORNERS` | Random 90° corner shapes |
| `RAIN` | Rainstorm: falling streaks with occasional lightning flashes |
| `LETTERS` | Random characters (A–Z plus German Ä Ö Ü ß) in a single-stroke font |
| `PLACE` | Dim positioning dot + serial-driven text placement |

### Recording & playback

In `MANUAL` mode, a visitor's joystick drawing is sampled into a recording slot
(`REC_SAMPLE_MS` cadence, up to `MAX_RECORDINGS` shapes of `MAX_REC_POINTS` points).
After the joystick goes idle the recording auto-saves and the piece returns to the show.
`PLAYBACK` mode and the inter-mode bursts replay these shapes, biased toward recent ones
and progressively distorting older recordings (see the `PLAYBACK_*` parameters).

### Day mode (ambient light)

If the installation sits idle long enough (`DAY_IDLE_MS`), it runs a "day" sequence:
the laser crossfades down and a DMX light fixture takes over with a simulated sun-and-clouds
cycle, then fades back. Tunable via the `DAY_*` and `CLOUD_*` parameters.

---

## Serial command reference

Commands are newline-terminated, sent at 9600 baud. Coordinates are integers in
**thousandths of the canvas** (0–1000 → 0.0–1.0).

| Command | Action |
|---|---|
| `M x y` | Move (laser off) to canvas position `x/1000, y/1000` |
| `D x y` | Draw (laser on) to canvas position `x/1000, y/1000` |
| `H` | Home (return to a reference position) |
| `L p` | Set laser power `p` (0–255) |
| `S` | Print status report |
| `N` | Next draw mode |
| `P` | Previous draw mode |
| `C` | Restart calibration |
| `K` | Quick-calibrate using the saved canvas size |
| *text* | In `LETTERS` mode: queue a word to draw |
| `"text"` | In `PLACE` mode: draw the quoted text at the current position |
| *number* | In `PLACE` mode: set the font size (÷1000 = scale) |

---

## Tuning

Nearly all behaviour is exposed as named constants in the **TUNABLE PARAMETERS** block near
the top of [`lumaries.ino`](lumaries.ino) — motor speeds and acceleration, recording/playback
behaviour, per-mode geometry (circle/rain/spiral/corners/letters/place), and the day-mode
light cycle. Adjust there and re-upload.

