# Neurorehabilitation Projektpraktikum

Seated trunk/leg rehabilitation exercises with a MuJoCo avatar, real-time
movement recognition, and a choreographed exercise routine (warmup + dance)
scored as a game. Movements can come from three sources: the MuJoCo avatar
itself (keyboard-driven), a keyboard simulator, or body-worn ESP32 IMU
sensors.

## Components

| File | Role |
|---|---|
| `run_professional_rehab.py` | MuJoCo viewer: animates the avatar, publishes its joint angles in real time (`mujoco_motion.json`), can also mirror recognized movements (live mode, key `L`) |
| `live_action.py` | Recognition GUI: reads a motion source, classifies the current movement, runs the routine game, publishes results (`live_state.json`) |
| `classify.py` | Single-label action classifier (NEUTRAL, LEAN, LIFT_L, LIFT_R, TURN_LEFT, TURN_RIGHT) with gating and debounce |
| `mapping.py` | Turns raw IMU samples into normalized lean/lift/turn features (lean from the lumbar sensor, turn from the cervical sensor, lifts from the thighs) |
| `game.py` | Routine game: the warmup + dance choreography, step by step |
| `professional_rehab_avatar.xml` | MuJoCo model of the seated avatar (sides named in the avatar's own anatomical frame) |
| `imu_collect.py` | Dataset recorder: logs raw IMU streams to CSV with live plots |

---

## 1. Setup

### macOS

1. Open Terminal and clone the repository:

   ```bash
   git clone https://github.com/morenzoe/Neurorehabilitation_Projekpraktikum.git
   cd Neurorehabilitation_Projekpraktikum
   git checkout Combination
   ```

2. Create a virtual environment and install the dependencies
   (Python 3.10 or newer; check with `python3 --version`):

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install mujoco pyqtgraph PyQt6 numpy pyserial
   ```

### Windows

1. Install Python 3.10+ from https://www.python.org/downloads/ .
   During installation tick **"Add python.exe to PATH"**.

2. Open *PowerShell* (or *Command Prompt*) and clone the repository:

   ```powershell
   git clone https://github.com/morenzoe/Neurorehabilitation_Projekpraktikum.git
   cd Neurorehabilitation_Projekpraktikum
   git checkout Combination
   ```

   (If `git` is missing, install it from https://git-scm.com/download/win .)

3. Create a virtual environment and install the dependencies:

   ```powershell
   py -m venv .venv
   .venv\Scripts\activate
   pip install mujoco pyqtgraph PyQt6 numpy pyserial
   ```

   If PowerShell refuses to run the activate script, run once:
   `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`

---

## 2. Run without sensors (avatar demo)

Two programs run side by side and talk through small JSON files.

1. **Terminal 1 — the MuJoCo avatar viewer:**

   ```bash
   # macOS — MUST use mjpython (installed with the mujoco package):
   ./.venv/bin/mjpython run_professional_rehab.py
   ```

   ```powershell
   # Windows — plain python works:
   python run_professional_rehab.py
   ```

   A window opens with a seated avatar.

2. **Terminal 2 — the movement recognition window**
   (activate the venv in this terminal too):

   ```bash
   python live_action.py --mode mujoco
   ```

   It waits for the viewer, calibrates for 2 seconds (keep the avatar
   neutral), then starts recognizing and running the routine.

3. **Drive the avatar.** Click the MuJoCo window; each key queues ONE
   repetition (neutral → max → neutral, ~2 s). Press several keys to
   perform a string of moves in order:

   | Key | Move (avatar's own left/right) |
   |---|---|
   | `1` | Trunk rotation LEFT |
   | `2` | Trunk rotation RIGHT |
   | `3` | Trunk bending (lean) |
   | `4` | LEFT leg raise |
   | `5` | RIGHT leg raise |
   | `0` | Reset / clear queue |

   Note: the camera faces the avatar, so the screen is a mirror — the
   avatar's left appears on your right. All labels are the avatar's own
   anatomical left/right, the same frame the body-worn sensors report.

---

## 3. Run with sensors (live mode)

### Hardware

- Adafruit Feather ESP32 V2 flashed with
  `Arduino/IMU_getAccGyr_twoAddr_oneMult_fourIMU` (board package v2.0.17,
  ICM20948_WE library v1.1.11, 115200 baud).
- Four ICM-20948 IMUs: **IMU0 lumbar** (lower back), **IMU1 cervical**
  (neck), **IMU3 left thigh**, **IMU4 right thigh**. The thigh sensors
  connect through the I2C multiplexer.

### Find the serial port and configure it

- **macOS**: plug the board in and run `ls /dev/cu.*` — the board appears
  as something like `/dev/cu.usbserial-59690891081`.
- **Windows**: open *Device Manager → Ports (COM & LPT)* — the board
  appears as e.g. `Silicon Labs CP210x` on a port like `COM5`. If no port
  appears, install the CP210x VCP driver from Silicon Labs.

Open `live_action.py` and set the port at the top:

```python
SERIAL_PORT = "/dev/cu.usbserial-59690891081"   # macOS example
SERIAL_PORT = "COM5"                            # Windows example
```

### Start a session

1. Start the viewer (Terminal 1, as above) if you want the avatar.
2. Start recognition on the sensors (Terminal 2):

   ```bash
   python live_action.py --mode live
   ```

3. Strap the sensors on, sit still in neutral, and click
   **Recalibrate (hold still)** — calibration takes 2 seconds and captures
   your personal neutral pose. Recalibrate any time a sensor is moved.
4. Press `L` in the MuJoCo viewer if you want the avatar to mirror you.

---

## 4. The routine (game)

The recognition window guides you through the choreography and shows the
current step, its progress, and the next expected move. A wrong or
out-of-order move flashes a warning but does **not** reset your progress.

**Warmup**
1. Baseline — hold neutral for 10 s (timer restarts if you move)
2. Foot Rotate — 20 s instructed (not sensor-checked)
3. Reach-Over — 20 s instructed

**Dance** (every move is checked, in order)
1. Tip Toes — lift_L → lift_R, alternating ×3
2. Lift off R — lift_R ×3
3. Lift off L — lift_L ×3
4. Babysteps — lift_L ×3
5. + 6. Trunk Flex — lean ×2, twice
7. Trunk Rotate — turn LEFT then turn RIGHT
8.–11. the same block again (Tip Toes, Flex ×2 twice, Rotate)
12. Babysteps — lift_R ×3

Scoring: +10 per correct move, +20 per completed step, +100 for finishing
the routine — 760 points per round, then it loops. Edit `ROUTINE` at the
top of `game.py` to change the choreography.

### Simulator mode (no avatar, no sensors)

```bash
python live_action.py --mode simulate
```

Keys: `Q` neutral, `W` lean, `E` left lift, `R` right lift, `T` turn left,
`Y` turn right.

---

## 5. How it works (data flow)

```
ESP32 (4 IMUs, serial)  --or--  MuJoCo avatar (mujoco_motion.json)
            |                            |
            +---------> live_action.py <-+
                        mapping.py -> classify.py -> game.py
                              |
                        live_state.json
                              |
            MuJoCo viewer live mode (press L: avatar mirrors you)
```

All motion sources are converted to the same normalized features
(`lean`, `lift_L`, `lift_R`, signed `turn` that tracks the held angle) and
pass through the same classifier, so recognition validated against the
avatar carries over unchanged to the real sensors.

---

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| macOS: viewer crashes at start with a launch error | Use `mjpython`, not `python`, for `run_professional_rehab.py` |
| "Port … not found" at startup | Board not plugged / still enumerating — check the port name and retry; on Windows install the CP210x driver |
| Port keeps disappearing and reappearing | Unstable USB connection: reseat the cable or use another USB port |
| A lift bar stuck at 0 while the leg moves | That thigh IMU isn't delivering data (NaN or zeros) — check the multiplexer wiring; the app keeps running on the healthy sensors |
| Features large while sitting still | Sensors moved after calibration — click **Recalibrate (hold still)** |
| Turn left/right look mirrored on screen | The screen is a mirror view; labels are in YOUR body frame, not the observer's |
| "Video file not found" in the video panel | Optional: place `instruction_video.mp4` next to `live_action.py` |
