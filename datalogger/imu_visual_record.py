import sys
import os
import csv
import time
import threading
import datetime
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not found in THIS Python interpreter.")
    print("Install it into the same Python you run this script with:")
    print("    pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/cu.usbserial-59690891081"   # find yours with:  ls /dev/cu.*
BAUD = 115200

ENABLE_GUI = True            # False = log only (records immediately), no plotting

OUTPUT_DIR = "logs"
SESSION_ID = "session"
ESP_VERSION = 2
SENSOR_PLACEMENT = "IMU0 lumbar, IMU1 clavicleR, IMU2 clavicleL, IMU3 thighL, IMU4 thighR"

WINDOW_SAMPLES = 1500        # rolling buffer length (samples kept in memory)
WINDOW_SEC = 15              # sliding view width: show the last N seconds
GUI_UPDATE_MS = 33           # plot refresh (about 30 FPS)
FLUSH_EVERY = 50             # flush CSV to disk every N rows
STATUS_EVERY_S = 5.0         # print kept/dropped + rate every N seconds

# ---------------------------------------------------------------------------
# Fixed schema: 5 IMUs x (accel xyz + gyro xyz) = 30 channels
# ---------------------------------------------------------------------------
IMU_LABELS = [
    "IMU0 lumbar",
    "IMU1 clavicleR",
    "IMU2 clavicleL",
    "IMU3 thighL",
    "IMU4 thighR",
]

CHANNELS = []
for i in range(5):
    s = str(i)
    CHANNELS += ["ax" + s, "ay" + s, "az" + s, "gx" + s, "gy" + s, "gz" + s]

CSV_COLUMNS = ["t_esp_ms", "t_host_s"] + CHANNELS + ["emg", "fsr"]

AXIS_COLORS = {"x": (220, 70, 70), "y": (70, 180, 90), "z": (80, 130, 230)}


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def parse_line(line):
    # Returns (t_esp_ms_or_None, values_dict, ok).
    # ok is False if any key:value token fails to parse as a float.
    t_esp = None
    values = {}
    ok = True
    for token in line.split(","):
        token = token.strip()
        if token == "":
            continue
        if ":" in token:
            key, _, raw = token.partition(":")
            key = key.strip()
            try:
                values[key] = float(raw)
            except ValueError:
                ok = False
        else:
            if t_esp is None:
                try:
                    t_esp = int(float(token))
                except ValueError:
                    pass
    return t_esp, values, ok


def make_csv_path():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(OUTPUT_DIR, str(SESSION_ID) + "_" + stamp + ".csv")


class SerialReader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._lock = threading.Lock()       # protects the plot buffers
        self._rec_lock = threading.Lock()   # protects the CSV file/writer

        self.ser = None

        # recording state (None/closed until start_recording is called)
        self.fh = None
        self.writer = None
        self.csv_path = None
        self.recording = False
        self._rows_since_flush = 0
        self.rec_rows = 0                   # rows written in the current session

        self.t_buf = deque(maxlen=WINDOW_SAMPLES)
        self.ch_buf = {k: deque(maxlen=WINDOW_SAMPLES) for k in CHANNELS}

        self.kept = 0                       # valid samples received (buffered)
        self.dropped = 0

    def open(self):
        # Open the serial port only. The CSV file is opened later, on demand,
        # by start_recording so each recording session gets its own file.
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)               # let the ESP32 finish resetting
        self.ser.reset_input_buffer()

    def start_recording(self):
        # Open a fresh timestamped CSV and begin writing incoming samples.
        # Returns the path that was opened (or the existing one if already on).
        with self._rec_lock:
            if self.recording:
                return self.csv_path
            self.csv_path = make_csv_path()
            self.fh = open(self.csv_path, "w", newline="")
            now = datetime.datetime.now().isoformat(timespec="seconds")
            self.fh.write("# session_id: " + str(SESSION_ID) + "\n")
            self.fh.write("# datetime: " + now + "\n")
            self.fh.write("# esp_version: " + str(ESP_VERSION) + "\n")
            self.fh.write("# sensor_placement: " + SENSOR_PLACEMENT + "\n")
            self.fh.write("# format: raw accel (g) + gyro (deg/s), 5 IMUs\n")
            self.fh.write("# baud: " + str(self.baud) + "\n")
            self.writer = csv.writer(self.fh)
            self.writer.writerow(CSV_COLUMNS)
            self._rows_since_flush = 0
            self.rec_rows = 0
            self.recording = True
            return self.csv_path

    def stop_recording(self):
        # Flush and close the current CSV. Returns the path saved (or None).
        with self._rec_lock:
            if not self.recording:
                return None
            self.recording = False
            path = self.csv_path
            if self.fh is not None:
                self.fh.flush()
                self.fh.close()
                self.fh = None
                self.writer = None
            return path

    def run(self):
        start = time.perf_counter()
        last_status = start
        kept_since_status = 0

        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                print("Serial read error:", exc)
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if line == "":
                continue

            t_esp, values, ok = parse_line(line)
            if not ok or not all(k in values for k in CHANNELS):
                self.dropped += 1
                continue

            t_host = time.perf_counter() - start

            # Always feed the live plot buffers, recording or not.
            t_plot = (t_esp / 1000.0) if t_esp is not None else t_host
            with self._lock:
                self.t_buf.append(t_plot)
                for k in CHANNELS:
                    self.ch_buf[k].append(values[k])

            # Write to CSV only while a recording session is active.
            with self._rec_lock:
                if self.recording and self.writer is not None:
                    row = [t_esp if t_esp is not None else "", round(t_host, 4)]
                    row += [values[k] for k in CHANNELS]
                    row += ["", ""]           # emg, fsr reserved
                    self.writer.writerow(row)
                    self.rec_rows += 1
                    self._rows_since_flush += 1
                    if self._rows_since_flush >= FLUSH_EVERY:
                        self.fh.flush()
                        self._rows_since_flush = 0

            self.kept += 1
            kept_since_status += 1
            nowp = time.perf_counter()
            if nowp - last_status >= STATUS_EVERY_S:
                hz = kept_since_status / (nowp - last_status)
                print("kept {}  dropped {}  ({:.1f} samples/s)".format(
                    self.kept, self.dropped, hz))
                kept_since_status = 0
                last_status = nowp

    def snapshot(self):
        with self._lock:
            t = list(self.t_buf)
            data = {k: list(self.ch_buf[k]) for k in CHANNELS}
        return t, data

    def stop(self):
        self._stop.set()
        self.join(timeout=3.0)
        saved = self.stop_recording()        # flush/close any open recording
        if saved is not None:
            print("recording saved ->", saved)
        if self.ser is not None:
            self.ser.close()
        print("done. received", self.kept, "samples, dropped", self.dropped)


def run_with_gui(reader):
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets

    app = pg.mkQApp("IMU live")
    pg.setConfigOptions(antialias=True)
    pg.setConfigOption("background", "#0e0f12")
    pg.setConfigOption("foreground", "#b8b8b8")

    # Container: a small toolbar (Start/Stop + status) above the plot area.
    container = QtWidgets.QWidget()
    container.setWindowTitle("IMU live")
    container.resize(1100, 1200)
    vbox = QtWidgets.QVBoxLayout(container)
    vbox.setContentsMargins(8, 8, 8, 8)
    vbox.setSpacing(8)

    bar = QtWidgets.QHBoxLayout()
    btn_start = QtWidgets.QPushButton("Start recording")
    btn_stop = QtWidgets.QPushButton("Stop recording")
    btn_stop.setEnabled(False)
    status = QtWidgets.QLabel("Idle - not recording")
    bar.addWidget(btn_start)
    bar.addWidget(btn_stop)
    bar.addWidget(status)
    bar.addStretch(1)
    vbox.addLayout(bar)

    win = pg.GraphicsLayoutWidget()
    vbox.addWidget(win)
    win.ci.setSpacing(8)

    # One color key for all plots (x/y/z mapping is identical everywhere),
    # placed at the top so no legend floats over the data.
    cx = "#%02x%02x%02x" % AXIS_COLORS["x"]
    cy = "#%02x%02x%02x" % AXIS_COLORS["y"]
    cz = "#%02x%02x%02x" % AXIS_COLORS["z"]
    key = pg.LabelItem(justify="center")
    key.setText(
        "<span style='color:" + cx + "'>&#9632; x</span>"
        + "&nbsp;&nbsp;&nbsp;&nbsp;<span style='color:" + cy + "'>&#9632; y</span>"
        + "&nbsp;&nbsp;&nbsp;&nbsp;<span style='color:" + cz + "'>&#9632; z</span>",
        size="11pt")
    win.addItem(key, row=0, col=0, colspan=2)

    curves = {}
    first_plot = None
    for i in range(5):
        s = str(i)
        # left column: accelerometer (g)
        ap = win.addPlot(row=i + 1, col=0)
        ap.setTitle(IMU_LABELS[i] + "  accel (g)",
                    size="10pt", color="#dddddd")
        ap.showGrid(x=True, y=True, alpha=0.18)
        ap.setDownsampling(mode="peak")
        ap.setClipToView(True)

        # right column: gyroscope (deg/s)
        gp = win.addPlot(row=i + 1, col=1)
        gp.setTitle(IMU_LABELS[i] + "  gyro (deg/s)",
                    size="10pt", color="#dddddd")
        gp.showGrid(x=True, y=True, alpha=0.18)
        gp.setDownsampling(mode="peak")
        gp.setClipToView(True)

        # share one time axis across all plots: zoom/pan one, all follow
        if first_plot is None:
            first_plot = ap
        else:
            ap.setXLink(first_plot)
        gp.setXLink(first_plot)

        if i == 4:
            ap.setLabel("bottom", "t", units="s")
            gp.setLabel("bottom", "t", units="s")

        for axis in ("x", "y", "z"):
            pen = pg.mkPen(AXIS_COLORS[axis], width=1.3)
            curves["a" + axis + s] = ap.plot(pen=pen)
            curves["g" + axis + s] = gp.plot(pen=pen)

    def on_start():
        try:
            path = reader.start_recording()
        except Exception as exc:
            status.setText("Could not start recording: " + str(exc))
            return
        btn_start.setEnabled(False)
        btn_stop.setEnabled(True)
        status.setText("Recording -> " + os.path.basename(path))

    def on_stop():
        path = reader.stop_recording()
        btn_start.setEnabled(True)
        btn_stop.setEnabled(False)
        if path is not None:
            status.setText("Saved -> " + os.path.basename(path)
                           + "   (" + str(reader.rec_rows) + " rows)")
        else:
            status.setText("Idle - not recording")

    btn_start.clicked.connect(on_start)
    btn_stop.clicked.connect(on_stop)

    def update():
        t, data = reader.snapshot()
        if len(t) == 0:
            return
        for ch in CHANNELS:
            curves[ch].setData(t, data[ch])
        # slide the view to the last WINDOW_SEC seconds; x-axes are linked,
        # so setting it once on the master plot moves all ten.
        t_latest = t[-1]
        first_plot.setXRange(t_latest - WINDOW_SEC, t_latest, padding=0)
        if reader.recording:
            status.setText("Recording -> " + os.path.basename(reader.csv_path)
                           + "   (" + str(reader.rec_rows) + " rows)")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(GUI_UPDATE_MS)

    app.aboutToQuit.connect(reader.stop)
    container.show()
    pg.exec()


def run_headless(reader):
    print("Logging to", reader.csv_path, "(Ctrl-C to stop)")
    try:
        while reader.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


def main():
    if SERIAL_PORT not in list_serial_ports():
        print("Port", SERIAL_PORT, "not found. Available ports:")
        for p in list_serial_ports():
            print("   ", p)
        print("On macOS:  ls /dev/cu.*   then set SERIAL_PORT at the top.")
        sys.exit(1)

    reader = SerialReader(SERIAL_PORT, BAUD)
    try:
        reader.open()
    except Exception as exc:
        print("Could not open serial:", exc)
        sys.exit(1)

    reader.start()

    if ENABLE_GUI:
        print("GUI ready. Use the Start / Stop recording buttons.")
        run_with_gui(reader)
    else:
        # No buttons in headless mode, so record immediately.
        try:
            reader.start_recording()
        except Exception as exc:
            print("Could not open CSV:", exc)
            reader.stop()
            sys.exit(1)
        run_headless(reader)


if __name__ == "__main__":
    main()
