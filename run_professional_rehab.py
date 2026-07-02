import os
import time
import json
import math
import mujoco
import mujoco.viewer

# Shared state file written by live_action.py's classifier thread.
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "live_state.json")

LEAN_MAX_DEG = 32
LIFT_MAX_DEG = 35
TURN_MAX_DEG = 45

model = mujoco.MjModel.from_xml_path("professional_rehab_avatar.xml")
data = mujoco.MjData(model)

torso_rotation_id = model.joint("torso_rotation").qposadr[0]
torso_bend_id = model.joint("torso_bend").qposadr[0]
head_id = model.joint("head_rotation").qposadr[0]
left_hip_id = model.joint("left_hip_lift").qposadr[0]
right_hip_id = model.joint("right_hip_lift").qposadr[0]
left_shoulder_id = model.joint("left_shoulder").qposadr[0]
right_shoulder_id = model.joint("right_shoulder").qposadr[0]

current_exercise = 0
exercise_start_time = time.time()
live_mode = False


def set_angle(joint_id, degrees):
    data.qpos[joint_id] = math.radians(degrees)


def read_live_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def key_callback(keycode):
    global current_exercise, exercise_start_time, live_mode

    key = chr(keycode).lower() if 0 <= keycode < 256 else ""

    if key == "l":
        live_mode = not live_mode
        print("Live sensor mode:", "ON (from live_action.py)" if live_mode else "OFF (keyboard demo)")
        return

    if key == "0":
        current_exercise = 0
        exercise_start_time = time.time()
        print("Exercise 0: Reset neutral")

    elif key == "1":
        current_exercise = 1
        exercise_start_time = time.time()
        print("Exercise 1: Trunk rotation LEFT")

    elif key == "q":
        current_exercise = 6
        exercise_start_time = time.time()
        print("Exercise Q: Trunk rotation RIGHT")

    elif key == "2":
        current_exercise = 2
        exercise_start_time = time.time()
        print("Exercise 2: Trunk bending")

    elif key == "3":
        current_exercise = 3
        exercise_start_time = time.time()
        print("Exercise 3: Left leg raise")

    elif key == "4":
        current_exercise = 4
        exercise_start_time = time.time()
        print("Exercise 4: Right leg raise")

    elif key == "5":
        current_exercise = 5
        exercise_start_time = time.time()
        print("Exercise 5: All movements")


print("Keyboard controls:")
print("1 = Trunk rotation LEFT")
print("Q = Trunk rotation RIGHT")
print("2 = Trunk bending, larger ROM")
print("3 = Left leg raise")
print("4 = Right leg raise")
print("0 = Reset neutral")
print("L = Toggle LIVE sensor mode (mirrors live_action.py's IMU classifier)")
print("Close the MuJoCo window to stop.")

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    while viewer.is_running():
        t = time.time() - exercise_start_time

        torso_rotation_angle = 0
        trunk_bend_angle = 0
        head_angle = 0
        left_knee_lift = 0
        right_knee_lift = 0

        if live_mode:
            live_state = read_live_state()
            if live_state is not None:
                label = live_state.get("label", "NEUTRAL")
                feats = live_state.get("features", {})

                if label == "LEAN":
                    trunk_bend_angle = LEAN_MAX_DEG * feats.get("lean", 0.0)

                elif label == "LIFT_L":
                    left_knee_lift = -LIFT_MAX_DEG * feats.get("lift_L", 0.0)

                elif label == "LIFT_R":
                    right_knee_lift = -LIFT_MAX_DEG * feats.get("lift_R", 0.0)

                elif label == "TURN_LEFT":
                    torso_rotation_angle = -TURN_MAX_DEG * abs(feats.get("turn", 0.0))
                    head_angle = torso_rotation_angle

                elif label == "TURN_RIGHT":
                    torso_rotation_angle = TURN_MAX_DEG * abs(feats.get("turn", 0.0))
                    head_angle = torso_rotation_angle

        else:
            # Smooth one-side movement: neutral -> max -> neutral
            one_rep = max(0, math.sin(t * 1.2))

            if current_exercise == 1:
                # Left only
                torso_rotation_angle = -45 * one_rep
                head_angle = torso_rotation_angle

            elif current_exercise == 6:
                # Right only
                torso_rotation_angle = 45 * one_rep
                head_angle = torso_rotation_angle

            elif current_exercise == 2:
                # Larger trunk bending range of motion
                trunk_bend_angle = 32 * one_rep

            elif current_exercise == 3:
                left_knee_lift = -35 * one_rep

            elif current_exercise == 4:
                right_knee_lift = -35 * one_rep

        set_angle(torso_rotation_id, torso_rotation_angle)
        set_angle(torso_bend_id, trunk_bend_angle)
        set_angle(head_id, head_angle)
        set_angle(left_hip_id, left_knee_lift)
        set_angle(right_hip_id, right_knee_lift)

        set_angle(left_shoulder_id, 0)
        set_angle(right_shoulder_id, 0)

        mujoco.mj_forward(model, data)
        viewer.sync()

        time.sleep(0.01)