# Neurorehabilitation Projektpraktikum

Seated trunk/leg rehabilitation exercises with a MuJoCo avatar, real-time
movement recognition, and a sequence game for motivation. Movements can come
from three sources: the MuJoCo avatar itself (keyboard-driven), a keyboard
simulator, or body-worn ESP32 IMU sensors.

## Components

| File | Role |
|---|---|
| `run_professional_rehab.py` | MuJoCo viewer: animates the avatar, publishes its joint angles in real time (`mujoco_motion.json`), can also mirror recognized movements (live mode) |
| `live_action.py` | Recognition GUI: reads a motion source, classifies the current movement, runs the sequence game, publishes results (`live_state.json`) |
| `classify.py` | Single-label action classifier (NEUTRAL, LEAN, LIFT_L, LIFT_R, TURN_LEFT, TURN_RIGHT) with gating and debounce |
| `mapping.py` | Turns raw IMU samples into normalized lean/lift/turn features |
| `game.py` | Sequence game: perform the required actions in order to score points |
| `professional_rehab_avatar.xml` | MuJoCo model of the seated avatar (sides named in the avatar's own anatomical frame) |

## Setup (step by step)

1. Clone the repository and switch to the branch you need:

   ```bash
   git clone https://github.com/morenzoe/Neurorehabilitation_Projekpraktikum.git
   cd Neurorehabilitation_Projekpraktikum
   git checkout Combination
   ```

2. Create a virtual environment and install the dependencies
   (Python 3.10 or newer):

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate        # Windows: .venv\Scripts\activate
   pip install mujoco pyqtgraph PyQt6 numpy pyserial
   ```

## Running the project (avatar + recognition, no sensors needed)

1. **Terminal 1 — start the MuJoCo avatar viewer:**

   ```bash
   # macOS (must use mjpython, installed with the mujoco package):
   ./.venv/bin/mjpython run_professional_rehab.py

   # Linux / Windows:
   python run_professional_rehab.py
   ```

   A window opens with a seated avatar.

2. **Terminal 2 — start the movement recognition window:**

   ```bash
   python live_action.py --mode mujoco
   ```

   It waits for the viewer, then calibrates for 2 seconds (keep the avatar
   neutral) and starts recognizing.

3. **Drive the avatar.** Click the MuJoCo window and press keys — each key
   queues ONE repetition (neutral → max → neutral, about 2 s); press several
   keys to perform a string of different actions in order:

   | Key | Action (avatar's own left/right) |
   |---|---|
   | `1` | Trunk rotation LEFT |
   | `2` | Trunk rotation RIGHT |
   | `3` | Trunk bending (lean forward) |
   | `4` | LEFT leg raise |
   | `5` | RIGHT leg raise |
   | `0` | Reset to neutral / clear the queue |

   The recognition window names each movement in real time and scores the
   sequence game: LEAN → TURN LEFT → LIFT RIGHT → TURN RIGHT, 10 points per
   correct step, +50 bonus for completing the sequence. A wrong move resets
   the sequence.

   Note: the camera faces the avatar, so the screen is a mirror — the
   avatar's left appears on your right. All labels use the avatar's own
   anatomical left/right, which is also what body-worn sensors report.

## Other modes of live_action.py

- **Keyboard simulator (no avatar, no sensors):**

  ```bash
  python live_action.py --mode simulate
  ```

  Keys: `Q` neutral, `W` lean, `E` left lift, `R` right lift, `T` turn
  left, `Y` turn right.

- **Live IMU sensors (ESP32):**

  ```bash
  python live_action.py --mode live
  ```

  Set `SERIAL_PORT` at the top of `live_action.py` to your device first
  (macOS: find it with `ls /dev/cu.*`). Hold still during the 2 s
  calibration. In this mode you can press `L` in the MuJoCo viewer so the
  avatar mirrors the recognized movements. (Don't use `L` together with
  `--mode mujoco` — the avatar would be watching itself.)

## How it works (data flow)

```
run_professional_rehab.py --(mujoco_motion.json: joint angles, 30 ms)--> live_action.py
        avatar                                                     classify + game
        ^                                                                 |
        +-------(live_state.json: label, features, score; live mode)------+
```

All three motion sources are converted to the same normalized features
(`lean`, `lift_L`, `lift_R`, `turn` in [0, 1]) and pass through the same
classifier, so recognition validated against the MuJoCo avatar carries over
unchanged when the real sensors are connected.

## Hardware (for --mode live)

- Adafruit Feather ESP32 V2 — board package v2.0.17
  (board URL: https://dl.espressif.com/dl/package_esp32_index.json)
- ICM20948_WE library v1.1.11
- Arduino sketches in `Arduino/`
