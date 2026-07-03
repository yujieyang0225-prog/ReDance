import time
import math
import serial
import mujoco
import mujoco.viewer

# ---------- SETTINGS ----------
SERIAL_PORT = "COM5"   # change this to your ESP32 port
BAUD_RATE = 115200

from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
XML_PATH = SCRIPT_DIR / "professional_rehab_avatar.xml"

print("Loading XML from:", XML_PATH)

model = mujoco.MjModel.from_xml_path(str(XML_PATH))
data = mujoco.MjData(model)

torso_rotation_id = model.joint("torso_rotation").qposadr[0]
torso_bend_id = model.joint("torso_bend").qposadr[0]
head_id = model.joint("head_rotation").qposadr[0]

left_hip_id = model.joint("left_hip_lift").qposadr[0]
right_hip_id = model.joint("right_hip_lift").qposadr[0]

left_shoulder_id = model.joint("left_shoulder").qposadr[0]
right_shoulder_id = model.joint("right_shoulder").qposadr[0]

# ---------- SERIAL ----------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
time.sleep(2)

# Calibration offsets
offsets = None


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def set_angle(joint_id, degrees):
    data.qpos[joint_id] = math.radians(degrees)


def read_imu_csv():
    """
    Expected Arduino CSV:
    time,pitch0,roll0,pitch1,roll1,pitch2,roll2,pitch3,roll3,pitch4,roll4

    IMU0 = lumbar
    IMU1 = clavicle right
    IMU2 = clavicle left
    IMU3 = thigh left
    IMU4 = thigh right
    """

    line = ser.readline().decode(errors="ignore").strip()

    if not line:
        return None

    try:
        values = [float(x) for x in line.split(",")]

        if len(values) != 11:
            return None

        return {
            "time_ms": values[0],

            "lumbar_pitch": values[1],
            "lumbar_roll": values[2],

            "clavicleR_pitch": values[3],
            "clavicleR_roll": values[4],

            "clavicleL_pitch": values[5],
            "clavicleL_roll": values[6],

            "thighL_pitch": values[7],
            "thighL_roll": values[8],

            "thighR_pitch": values[9],
            "thighR_roll": values[10],
        }

    except ValueError:
        return None


def apply_calibration(imu):
    global offsets

    if offsets is None:
        return imu

    calibrated = {}

    for key, value in imu.items():
        if key == "time_ms":
            calibrated[key] = value
        else:
            calibrated[key] = value - offsets[key]

    return calibrated


def key_callback(keycode):
    global offsets

    key = chr(keycode).lower() if 0 <= keycode < 256 else ""

    if key == "c":
        imu = read_imu_csv()
        if imu is not None:
            offsets = imu.copy()
            print("Calibration saved. Current posture is now neutral.")

    elif key == "r":
        offsets = None
        print("Calibration reset.")


print("MuJoCo IMU Rehab Viewer")
print("-----------------------")
print("C = calibrate current posture as neutral")
print("R = reset calibration")
print("Close MuJoCo window to stop.")

last_valid_imu = None

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    while viewer.is_running():

        imu = read_imu_csv()

        if imu is not None:
            last_valid_imu = apply_calibration(imu)

        if last_valid_imu is not None:
            imu = last_valid_imu

            # -------- MAPPING IMU DATA TO MUJOCO JOINTS --------
            # You may need to change signs depending on sensor orientation.

            torso_bend_angle = imu["lumbar_pitch"]
            torso_rotation_angle = imu["lumbar_roll"]

            left_hip_angle = -imu["thighL_pitch"]
            right_hip_angle = -imu["thighR_pitch"]

            # Optional simple shoulder visualization
            left_shoulder_angle = imu["clavicleL_pitch"] * 0.5
            right_shoulder_angle = imu["clavicleR_pitch"] * 0.5

            # Clamp angles to match XML joint limits
            torso_bend_angle = clamp(torso_bend_angle, -25, 25)
            torso_rotation_angle = clamp(torso_rotation_angle, -45, 45)

            left_hip_angle = clamp(left_hip_angle, -65, 25)
            right_hip_angle = clamp(right_hip_angle, -65, 25)

            left_shoulder_angle = clamp(left_shoulder_angle, -20, 20)
            right_shoulder_angle = clamp(right_shoulder_angle, -20, 20)

            # -------- UPDATE MUJOCO --------
            set_angle(torso_bend_id, torso_bend_angle)
            set_angle(torso_rotation_id, torso_rotation_angle)
            set_angle(head_id, torso_rotation_angle)

            set_angle(left_hip_id, left_hip_angle)
            set_angle(right_hip_id, right_hip_angle)

            set_angle(left_shoulder_id, left_shoulder_angle)
            set_angle(right_shoulder_id, right_shoulder_angle)

        mujoco.mj_forward(model, data)
        viewer.sync()

        time.sleep(0.01)
