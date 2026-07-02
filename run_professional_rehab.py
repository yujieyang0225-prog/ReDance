import time
import math
import mujoco
import mujoco.viewer

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


def set_angle(joint_id, degrees):
    data.qpos[joint_id] = math.radians(degrees)


def key_callback(keycode):
    global current_exercise, exercise_start_time

    key = chr(keycode).lower() if 0 <= keycode < 256 else ""

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