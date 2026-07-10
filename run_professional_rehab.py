import os
import time
import json
import math
import mujoco
import mujoco.viewer

# Shared state file written by live_action.py's classifier thread.
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "live_state.json")

# Real-time joint angles published for live_action.py's "mujoco" mode, so it
# can monitor and recognize whatever motion the avatar is currently doing.
MOTION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "mujoco_motion.json")
MOTION_WRITE_INTERVAL = 0.03  # seconds, throttles disk writes

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

# All directions are in the AVATAR'S OWN body frame (its anatomical
# left/right), matching what body-worn sensors will report. The default
# camera faces the avatar, so the view is a mirror: the avatar's left
# appears on the observer's right.
# Positive torso_rotation (+z hinge) turns the avatar to ITS left.

KEY_TO_ACTION = {
    "1": "TURN_LEFT",
    "2": "TURN_RIGHT",
    "3": "LEAN",
    "4": "LIFT_L",
    "5": "LIFT_R",
}

ACTION_TEXT = {
    "TURN_LEFT": "Trunk rotation LEFT",
    "TURN_RIGHT": "Trunk rotation RIGHT",
    "LEAN": "Trunk bending",
    "LIFT_L": "LEFT leg raise",
    "LIFT_R": "RIGHT leg raise",
}

# Each keypress queues ONE discrete repetition (neutral -> max -> neutral),
# like a tester performing a required action once. Queued keys play back in
# order, separated by a neutral gap so the classifier sees a clean edge
# between consecutive moves.
REP_SECONDS = 2.0
REP_GAP_SECONDS = 0.4

current_action = None        # (action, start_time) of the rep in progress
action_queue = []
last_rep_end = 0.0
live_mode = False

# Console feedback for the routine game running inside live_action.py.
GAME_EVENT_TEXT = {
    "correct": "Correct move!",
    "wrong": "Wrong move.",
    "step_complete": "Step complete!",
    "routine_complete": "Routine complete! Bonus awarded.",
}
_last_game_event_ts = 0.0


def report_game_event(game, event, event_ts):
    global _last_game_event_ts
    if event is None or event_ts <= _last_game_event_ts:
        return
    _last_game_event_ts = event_ts
    print("[GAME] %-9s Score: %-4d Rounds: %-3d Step: %s"
          % (GAME_EVENT_TEXT.get(event, event), game["score"],
             game["rounds_completed"], game["expected"]))


def set_angle(joint_id, degrees):
    data.qpos[joint_id] = math.radians(degrees)


def read_live_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


_last_motion_write = 0.0


def write_motion_state():
    global _last_motion_write
    now = time.time()
    if now - _last_motion_write < MOTION_WRITE_INTERVAL:
        return
    _last_motion_write = now
    payload = {
        "angles": {
            "torso_rotation": math.degrees(data.qpos[torso_rotation_id]),
            "torso_bend": math.degrees(data.qpos[torso_bend_id]),
            "left_hip": math.degrees(data.qpos[left_hip_id]),
            "right_hip": math.degrees(data.qpos[right_hip_id]),
        },
        "ts": now,
    }
    tmp = MOTION_FILE + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.replace(tmp, MOTION_FILE)
    except OSError:
        pass


def key_callback(keycode):
    global live_mode, current_action, last_rep_end

    key = chr(keycode).lower() if 0 <= keycode < 256 else ""

    if key == "l":
        live_mode = not live_mode
        print("Live sensor mode:", "ON (from live_action.py)" if live_mode else "OFF (keyboard demo)")
        return

    if live_mode:
        return

    if key == "0":
        action_queue.clear()
        current_action = None
        last_rep_end = 0.0
        print("Reset: queue cleared, back to neutral")

    elif key in KEY_TO_ACTION:
        action = KEY_TO_ACTION[key]
        action_queue.append(action)
        print("Queued: %s  (%d pending)" % (ACTION_TEXT[action], len(action_queue)))


print("Keyboard controls (avatar's OWN left/right; mirror view on screen):")
print("1 = Trunk rotation LEFT")
print("2 = Trunk rotation RIGHT")
print("3 = Trunk bending")
print("4 = LEFT leg raise")
print("5 = RIGHT leg raise")
print("0 = Reset neutral / clear queue")
print("Each key queues ONE repetition; press several keys to perform")
print("a string of different actions in order, like a tester would.")
print("L = Toggle LIVE sensor mode (mirrors live_action.py's IMU classifier)")
print("Close the MuJoCo window to stop.")

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    while viewer.is_running():
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
                game = live_state.get("game")
                if game is not None:
                    report_game_event(game, live_state.get("game_event"),
                                       live_state.get("game_event_ts", 0.0))

                if label == "LEAN":
                    trunk_bend_angle = LEAN_MAX_DEG * feats.get("lean", 0.0)

                elif label == "LIFT_L":
                    left_knee_lift = -LIFT_MAX_DEG * feats.get("lift_L", 0.0)

                elif label == "LIFT_R":
                    right_knee_lift = -LIFT_MAX_DEG * feats.get("lift_R", 0.0)

                elif label == "TURN_LEFT":
                    torso_rotation_angle = TURN_MAX_DEG * abs(feats.get("turn", 0.0))
                    head_angle = torso_rotation_angle

                elif label == "TURN_RIGHT":
                    torso_rotation_angle = -TURN_MAX_DEG * abs(feats.get("turn", 0.0))
                    head_angle = torso_rotation_angle

        else:
            now = time.time()
            if current_action is None and action_queue and \
                    now - last_rep_end >= REP_GAP_SECONDS:
                current_action = (action_queue.pop(0), now)
                print("Performing:", ACTION_TEXT[current_action[0]])

            if current_action is not None:
                action, t0 = current_action
                t = now - t0
                if t >= REP_SECONDS:
                    current_action = None
                    last_rep_end = now
                else:
                    # One discrete repetition: neutral -> max -> neutral
                    amp = math.sin(math.pi * t / REP_SECONDS)

                    if action == "TURN_LEFT":
                        torso_rotation_angle = TURN_MAX_DEG * amp
                        head_angle = torso_rotation_angle

                    elif action == "TURN_RIGHT":
                        torso_rotation_angle = -TURN_MAX_DEG * amp
                        head_angle = torso_rotation_angle

                    elif action == "LEAN":
                        trunk_bend_angle = LEAN_MAX_DEG * amp

                    elif action == "LIFT_L":
                        left_knee_lift = -LIFT_MAX_DEG * amp

                    elif action == "LIFT_R":
                        right_knee_lift = -LIFT_MAX_DEG * amp

        set_angle(torso_rotation_id, torso_rotation_angle)
        set_angle(torso_bend_id, trunk_bend_angle)
        set_angle(head_id, head_angle)
        set_angle(left_hip_id, left_knee_lift)
        set_angle(right_hip_id, right_knee_lift)

        set_angle(left_shoulder_id, 0)
        set_angle(right_shoulder_id, 0)

        mujoco.mj_forward(model, data)
        write_motion_state()
        viewer.sync()

        time.sleep(0.01)