import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

try:
    import cv2
    import numpy as np
except ImportError as exc:
    print(f"Missing Python library: {exc}")
    print("Install with: py -m pip install opencv-python numpy pyserial")
    raise SystemExit(1)

try:
    import serial
except ImportError:
    serial = None

try:
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError:
    tk = None
    messagebox = None
    ttk = None


# =========================
# User configuration
# =========================

CAMERA_URL = "http://192.168.1.22:4747/video"

WIDTH_CM = 21.0
HEIGHT_CM = 14.85
PX_PER_CM = 30

H_MIN = 19
H_MAX = 54
S_MIN = 0
S_MAX = 248
V_MIN = 226
V_MAX = 255

CENTER_X = 13.5
CENTER_Y = 8.2

BASE_CENTER = 82
K_BASE = 4.0
BASE_MIN = 0
BASE_MAX = 180

SEND_SERIAL = True
SERIAL_PORT = "COM5"
BAUDRATE = 9600


# =========================
# Manual calibration defaults
# =========================

HOME_D11 = 82
HOME_D10 = 0
HOME_D9 = 0
GRIP_OPEN = 135
GRIP_CLOSE = 90

PICK_UP_D10 = 90
PICK_UP_D9 = 90

PICK_DOWN_D10 = 90
PICK_DOWN_D9 = 90

DROP_D11 = 82
DROP_D10 = 90
DROP_D9 = 90


# =========================
# Detection tuning
# =========================

TARGET_SIDE_CM = 2.0
TARGET_AREA_CM2 = TARGET_SIDE_CM * TARGET_SIDE_CM

SIDE_MIN_CM = 1.0
SIDE_MAX_CM = 3.4
AREA_MIN_CM2 = 1.2
AREA_MAX_CM2 = 9.0
MAX_ASPECT_RATIO = 1.65
MIN_RECT_EXTENT = 0.35
MIN_CONTOUR_AREA_PX = 80

MORPH_KERNEL_SIZE = 5
CAMERA_REOPEN_DELAY_SEC = 1.0


# =========================
# Display constants
# =========================

TOP_WIDTH_PX = int(round(WIDTH_CM * PX_PER_CM))
TOP_HEIGHT_PX = int(round(HEIGHT_CM * PX_PER_CM))

WINDOW_CAMERA = "Camera"
WINDOW_TOP = "Top View"
WINDOW_MASK = "Binary Mask"

CALIBRATION_LABELS = (
    "top-left",
    "top-right",
    "bottom-right",
    "bottom-left",
)


@dataclass
class Detection:
    contour: np.ndarray
    rect: Tuple[Tuple[float, float], Tuple[float, float], float]
    center_px: Tuple[float, float]
    x_cm: float
    y_cm: float
    width_cm: float
    height_cm: float
    area_cm2: float
    score: float


class CalibrationState:
    def __init__(self) -> None:
        self.points: list[Tuple[int, int]] = []
        self.homography: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.points.clear()
        self.homography = None
        print("Calibration reset. Click 4 corners: top-left, top-right, bottom-right, bottom-left.")

    def add_point(self, x: int, y: int) -> None:
        if len(self.points) >= 4:
            return

        self.points.append((x, y))
        label = CALIBRATION_LABELS[len(self.points) - 1]
        print(f"Calib {len(self.points)}/4 {label}: ({x}, {y})")

        if len(self.points) == 4:
            self.compute_homography()

    def compute_homography(self) -> None:
        src = np.float32(self.points)
        dst = np.float32(
            [
                [0, 0],
                [TOP_WIDTH_PX - 1, 0],
                [TOP_WIDTH_PX - 1, TOP_HEIGHT_PX - 1],
                [0, TOP_HEIGHT_PX - 1],
            ]
        )
        self.homography = cv2.getPerspectiveTransform(src, dst)
        print("Calibration done. Top View is ready.")

    def is_ready(self) -> bool:
        return self.homography is not None


class RobotSerial:
    def __init__(self) -> None:
        self.link = None

    def is_connected(self) -> bool:
        return self.link is not None and self.link.is_open

    def open(self, log: Callable[[str], None]) -> None:
        if not SEND_SERIAL:
            log("SEND_SERIAL = False. Serial commands are printed only.")
            return

        if serial is None:
            log("Missing pyserial. Install with: py -m pip install pyserial")
            return

        if self.is_connected():
            log(f"Serial already connected: {SERIAL_PORT}")
            return

        try:
            self.link = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.02)
            time.sleep(2.0)
            self.link.reset_input_buffer()
            log(f"Serial connected: {SERIAL_PORT} @ {BAUDRATE}")
        except Exception as exc:
            self.link = None
            log(f"Cannot open serial {SERIAL_PORT}: {exc}")

    def close(self) -> None:
        if self.link is not None and self.link.is_open:
            self.link.close()

    def disconnect(self, log: Callable[[str], None]) -> None:
        self.close()
        log("Serial disconnected.")

    def read_status(self, log: Callable[[str], None]) -> None:
        if not self.is_connected():
            return

        try:
            while self.link.in_waiting > 0:
                line = self.link.readline().decode("ascii", errors="replace").strip()
                if line:
                    log(f"Arduino: {line}")
        except Exception as exc:
            log(f"Serial read error: {exc}")
            self.close()

    def send_command(self, command: str, log: Callable[[str], None]) -> bool:
        if not SEND_SERIAL:
            log(f"Serial off. Command not sent: {command}")
            return False

        if not self.is_connected():
            log(f"Serial not connected. Command not sent: {command}")
            return False

        try:
            self.link.write((command + "\n").encode("ascii"))
            self.link.flush()
            log(f"Sent: {command}")
            return True
        except Exception as exc:
            log(f"Serial write error: {exc}")
            self.close()
            return False


class ControlPanel:
    def __init__(
        self,
        serial_link: RobotSerial,
        reset_calib: Callable[[], None],
    ) -> None:
        if tk is None or ttk is None or messagebox is None:
            print("Tkinter is required for the servo control GUI.")
            raise SystemExit(1)

        self.serial_link = serial_link
        self.reset_calib = reset_calib
        self.should_quit = False
        self.latest_detection: Optional[Detection] = None
        self.latest_base_angle: Optional[float] = None

        self.root = tk.Tk()
        self.root.title("Robot Arm Control")
        self.root.protocol("WM_DELETE_WINDOW", self.request_quit)

        self.serial_status = tk.StringVar(value="Serial: disconnected")
        self.camera_status = tk.StringVar(value="Camera: waiting")
        self.detect_status = tk.StringVar(value="Object: none")
        self.base_status = tk.StringVar(value="Base angle: --")

        self.angle_vars = {
            "D11": tk.IntVar(value=HOME_D11),
            "D10": tk.IntVar(value=HOME_D10),
            "D9": tk.IntVar(value=HOME_D9),
            "D6": tk.IntVar(value=GRIP_OPEN),
        }

        self._build()
        self.log("Program started. Servos stay disabled until you press ENABLE SERVOS.")

    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        status_frame = ttk.LabelFrame(main, text="Status", padding=8)
        status_frame.grid(row=0, column=0, sticky="ew")
        status_frame.columnconfigure(0, weight=1)
        ttk.Label(status_frame, textvariable=self.camera_status).grid(row=0, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.serial_status).grid(row=1, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.detect_status).grid(row=2, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.base_status).grid(row=3, column=0, sticky="w")

        serial_frame = ttk.LabelFrame(main, text="Serial / Safety", padding=8)
        serial_frame.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(serial_frame, text="Connect Serial", command=self.connect_serial).grid(row=0, column=0, padx=3, pady=3)
        ttk.Button(serial_frame, text="Disconnect", command=self.disconnect_serial).grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(serial_frame, text="ENABLE SERVOS", command=self.enable_servos).grid(row=0, column=2, padx=3, pady=3)
        ttk.Button(serial_frame, text="DETACH", command=self.detach_servos).grid(row=0, column=3, padx=3, pady=3)

        manual_frame = ttk.LabelFrame(main, text="Manual Servo Angles", padding=8)
        manual_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        manual_frame.columnconfigure(1, weight=1)

        self._add_servo_row(manual_frame, 0, "D11", "Base")
        self._add_servo_row(manual_frame, 1, "D10", "Lift")
        self._add_servo_row(manual_frame, 2, "D9", "Extend")
        self._add_servo_row(manual_frame, 3, "D6", "Gripper")

        manual_buttons = ttk.Frame(manual_frame)
        manual_buttons.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        ttk.Button(manual_buttons, text="Send All", command=self.send_all).grid(row=0, column=0, padx=3)
        ttk.Button(manual_buttons, text="D11 from Camera", command=self.fill_d11_from_camera).grid(row=0, column=1, padx=3)
        ttk.Button(manual_buttons, text="Grip Open", command=self.grip_open).grid(row=0, column=2, padx=3)
        ttk.Button(manual_buttons, text="Grip Close", command=self.grip_close).grid(row=0, column=3, padx=3)
        ttk.Button(manual_buttons, text="HOME", command=self.home_robot).grid(row=0, column=4, padx=3)

        pick_frame = ttk.LabelFrame(main, text="Camera Pick", padding=8)
        pick_frame.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(pick_frame, text="PICK Detected Object", command=self.confirm_pick).grid(row=0, column=0, padx=3, pady=3)
        ttk.Button(pick_frame, text="Reset Calib", command=self.reset_calib).grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(pick_frame, text="Quit", command=self.request_quit).grid(row=0, column=2, padx=3, pady=3)

        log_frame = ttk.LabelFrame(main, text="Log", padding=8)
        log_frame.grid(row=4, column=0, sticky="nsew", pady=(8, 0))
        main.rowconfigure(4, weight=1)
        self.log_text = tk.Text(log_frame, height=9, width=72, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

    def _add_servo_row(self, parent: ttk.LabelFrame, row: int, servo_id: str, label: str) -> None:
        var = self.angle_vars[servo_id]
        ttk.Label(parent, text=f"{servo_id} {label}", width=12).grid(row=row, column=0, sticky="w", pady=3)
        scale = tk.Scale(parent, from_=0, to=180, orient="horizontal", resolution=1, variable=var, length=330)
        scale.grid(row=row, column=1, sticky="ew", padx=5, pady=3)
        spin = ttk.Spinbox(parent, from_=0, to=180, textvariable=var, width=5)
        spin.grid(row=row, column=2, padx=5, pady=3)
        ttk.Button(parent, text="Send", command=lambda s=servo_id: self.send_single(s)).grid(row=row, column=3, padx=3, pady=3)

    def log(self, message: str) -> None:
        print(message)
        try:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", message + "\n")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        except tk.TclError:
            pass

    def _angle(self, servo_id: str) -> int:
        return int(clamp(self.angle_vars[servo_id].get(), 0, 180))

    def connect_serial(self) -> None:
        self.serial_link.open(self.log)
        self.update_serial_status()

    def disconnect_serial(self) -> None:
        self.serial_link.disconnect(self.log)
        self.update_serial_status()

    def update_serial_status(self) -> None:
        if self.serial_link.is_connected():
            self.serial_status.set(f"Serial: connected {SERIAL_PORT} @ {BAUDRATE}")
        else:
            self.serial_status.set("Serial: disconnected")

    def enable_servos(self) -> None:
        d11 = self._angle("D11")
        d10 = self._angle("D10")
        d9 = self._angle("D9")
        d6 = self._angle("D6")
        ok = messagebox.askyesno(
            "Enable servos",
            "This will attach servos and move them to the slider angles:\n"
            f"D11={d11}, D10={d10}, D9={d9}, D6={d6}\n\nContinue?",
        )
        if not ok:
            self.log("Enable servos cancelled.")
            return
        self.serial_link.send_command(f"ENABLE,{d11},{d10},{d9},{d6}", self.log)

    def detach_servos(self) -> None:
        self.serial_link.send_command("DETACH", self.log)

    def send_single(self, servo_id: str) -> None:
        angle = self._angle(servo_id)
        self.serial_link.send_command(f"SET,{servo_id},{angle}", self.log)

    def send_all(self) -> None:
        d11 = self._angle("D11")
        d10 = self._angle("D10")
        d9 = self._angle("D9")
        d6 = self._angle("D6")
        self.serial_link.send_command(f"SETALL,{d11},{d10},{d9},{d6}", self.log)

    def fill_d11_from_camera(self) -> None:
        if self.latest_base_angle is None:
            self.log("No camera base angle yet.")
            return
        angle = int(round(clamp(self.latest_base_angle, BASE_MIN, BASE_MAX)))
        self.angle_vars["D11"].set(angle)
        self.log(f"D11 slider set from camera: {angle}")

    def grip_open(self) -> None:
        self.angle_vars["D6"].set(GRIP_OPEN)
        self.send_single("D6")

    def grip_close(self) -> None:
        self.angle_vars["D6"].set(GRIP_CLOSE)
        self.send_single("D6")

    def home_robot(self) -> None:
        ok = messagebox.askyesno("Move HOME", "Move robot to HOME angles now?")
        if not ok:
            self.log("HOME cancelled.")
            return
        self.serial_link.send_command("HOME", self.log)

    def confirm_pick(self) -> None:
        if self.latest_detection is None or self.latest_base_angle is None:
            self.log("No valid yellow square. PICK not sent.")
            return

        angle = int(round(clamp(self.latest_base_angle, BASE_MIN, BASE_MAX)))
        detection = self.latest_detection
        ok = messagebox.askyesno(
            "Confirm PICK",
            "Send PICK command to Arduino?\n\n"
            f"X={detection.x_cm:.2f} cm, Y={detection.y_cm:.2f} cm\n"
            f"Base angle D11={angle}",
        )
        if not ok:
            self.log("PICK cancelled.")
            return

        self.serial_link.send_command(f"PICK,{angle}", self.log)

    def update_live_status(
        self,
        camera_ok: bool,
        calibrated: bool,
        detection: Optional[Detection],
        base_angle: Optional[float],
    ) -> None:
        self.latest_detection = detection
        self.latest_base_angle = base_angle

        if camera_ok:
            self.camera_status.set("Camera: connected")
        else:
            self.camera_status.set("Camera: waiting / reconnecting")

        if not calibrated:
            self.detect_status.set("Object: waiting for 4-point calibration")
            self.base_status.set("Base angle: --")
        elif detection is None or base_angle is None:
            self.detect_status.set("Object: none")
            self.base_status.set("Base angle: --")
        else:
            self.detect_status.set(f"Object: X={detection.x_cm:.2f} cm, Y={detection.y_cm:.2f} cm")
            self.base_status.set(f"Base angle: {base_angle:.0f} deg")

        self.update_serial_status()

    def poll(self) -> None:
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.should_quit = True

    def request_quit(self) -> None:
        self.should_quit = True

    def close(self) -> None:
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def open_camera() -> cv2.VideoCapture:
    print(f"Opening camera: {CAMERA_URL}")
    cap = cv2.VideoCapture(CAMERA_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def on_camera_mouse(event: int, x: int, y: int, flags: int, param: CalibrationState) -> None:
    del flags
    if event == cv2.EVENT_LBUTTONDOWN:
        param.add_point(x, y)


def make_blank_top_view(message: str) -> np.ndarray:
    image = np.zeros((TOP_HEIGHT_PX, TOP_WIDTH_PX, 3), dtype=np.uint8)
    cv2.putText(image, message, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
    return image


def make_blank_mask(message: str) -> np.ndarray:
    image = np.zeros((TOP_HEIGHT_PX, TOP_WIDTH_PX), dtype=np.uint8)
    cv2.putText(image, message, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 180, 2)
    return image


def draw_calibration(frame: np.ndarray, calib: CalibrationState) -> np.ndarray:
    output = frame.copy()

    for index, point in enumerate(calib.points):
        cv2.circle(output, point, 6, (0, 255, 255), -1)
        cv2.putText(
            output,
            str(index + 1),
            (point[0] + 8, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

    if len(calib.points) > 1:
        for start, end in zip(calib.points, calib.points[1:]):
            cv2.line(output, start, end, (0, 255, 255), 2)

    if len(calib.points) == 4:
        cv2.line(output, calib.points[3], calib.points[0], (0, 255, 255), 2)

    if not calib.is_ready():
        next_index = len(calib.points)
        label = CALIBRATION_LABELS[next_index] if next_index < 4 else "done"
        text = f"Click corner {next_index + 1}/4: {label}"
        cv2.putText(output, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    else:
        text = "Calibrated. Use GUI or p=confirm pick, r=reset, q=quit"
        cv2.putText(output, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

    return output


def detect_yellow_square(top_view: np.ndarray) -> Tuple[Optional[Detection], np.ndarray]:
    hsv = cv2.cvtColor(top_view, cv2.COLOR_BGR2HSV)
    lower = np.array([H_MIN, S_MIN, V_MIN], dtype=np.uint8)
    upper = np.array([H_MAX, S_MAX, V_MAX], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_detection: Optional[Detection] = None
    best_score = float("inf")

    for contour in contours:
        area_px = cv2.contourArea(contour)
        if area_px < MIN_CONTOUR_AREA_PX:
            continue

        rect = cv2.minAreaRect(contour)
        (cx, cy), (w_px, h_px), _ = rect
        if w_px <= 0 or h_px <= 0:
            continue

        width_cm = w_px / PX_PER_CM
        height_cm = h_px / PX_PER_CM
        long_side = max(width_cm, height_cm)
        short_side = min(width_cm, height_cm)
        if short_side <= 0:
            continue

        aspect_ratio = long_side / short_side
        rect_area_px = w_px * h_px
        rect_extent = area_px / rect_area_px if rect_area_px > 0 else 0
        area_cm2 = area_px / (PX_PER_CM * PX_PER_CM)

        if not (SIDE_MIN_CM <= short_side <= SIDE_MAX_CM and SIDE_MIN_CM <= long_side <= SIDE_MAX_CM):
            continue
        if not (AREA_MIN_CM2 <= area_cm2 <= AREA_MAX_CM2):
            continue
        if aspect_ratio > MAX_ASPECT_RATIO:
            continue
        if rect_extent < MIN_RECT_EXTENT:
            continue

        side_avg = (width_cm + height_cm) * 0.5
        score = abs(side_avg - TARGET_SIDE_CM) + abs(area_cm2 - TARGET_AREA_CM2) * 0.35 + abs(aspect_ratio - 1.0)

        if score < best_score:
            best_score = score
            best_detection = Detection(
                contour=contour,
                rect=rect,
                center_px=(cx, cy),
                x_cm=cx / PX_PER_CM,
                y_cm=cy / PX_PER_CM,
                width_cm=width_cm,
                height_cm=height_cm,
                area_cm2=area_cm2,
                score=score,
            )

    return best_detection, mask


def compute_base_angle(x_cm: float) -> float:
    dx = x_cm - CENTER_X
    return clamp(BASE_CENTER + K_BASE * dx, BASE_MIN, BASE_MAX)


def draw_detection(top_view: np.ndarray, detection: Optional[Detection], base_angle: Optional[float]) -> np.ndarray:
    output = top_view.copy()

    center_px = (int(round(CENTER_X * PX_PER_CM)), int(round(CENTER_Y * PX_PER_CM)))
    cv2.drawMarker(output, center_px, (255, 0, 255), cv2.MARKER_CROSS, 22, 2)
    cv2.putText(output, "CENTER", (center_px[0] + 8, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

    if detection is None or base_angle is None:
        cv2.putText(output, "No valid yellow square", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return output

    box = cv2.boxPoints(detection.rect)
    box = np.intp(box)
    cx, cy = detection.center_px
    center = (int(round(cx)), int(round(cy)))
    dx = detection.x_cm - CENTER_X

    cv2.drawContours(output, [box], 0, (0, 255, 0), 2)
    cv2.circle(output, center, 5, (0, 0, 255), -1)
    cv2.line(output, center_px, center, (255, 255, 0), 2)

    lines = [
        f"X={detection.x_cm:.2f} cm  Y={detection.y_cm:.2f} cm",
        f"dx={dx:.2f} cm  base={base_angle:.0f} deg",
        f"size={detection.width_cm:.2f}x{detection.height_cm:.2f} cm  area={detection.area_cm2:.2f} cm2",
        "Press p or GUI PICK, then confirm",
    ]

    y = 32
    for line in lines:
        cv2.putText(output, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        y += 28

    return output


def main() -> None:
    calib = CalibrationState()
    serial_link = RobotSerial()
    gui = ControlPanel(serial_link, calib.reset)

    cv2.namedWindow(WINDOW_CAMERA)
    cv2.namedWindow(WINDOW_TOP)
    cv2.namedWindow(WINDOW_MASK)
    cv2.setMouseCallback(WINDOW_CAMERA, on_camera_mouse, calib)

    cap = open_camera()
    if not cap.isOpened():
        gui.log("Camera not connected yet. The program will keep reconnecting.")

    last_reopen_time = 0.0
    latest_detection: Optional[Detection] = None
    latest_base_angle: Optional[float] = None
    camera_ok = False

    gui.log("Controls: click 4 corners | GUI buttons | p=confirm PICK | r=reset calib | q=quit")
    gui.log("Corner order: top-left, top-right, bottom-right, bottom-left")

    try:
        while not gui.should_quit:
            serial_link.read_status(gui.log)

            ok, frame = cap.read() if cap.isOpened() else (False, None)
            camera_ok = ok and frame is not None

            if not camera_ok:
                now = time.monotonic()
                if now - last_reopen_time >= CAMERA_REOPEN_DELAY_SEC:
                    last_reopen_time = now
                    cap.release()
                    cap = open_camera()

                camera_view = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(camera_view, "Camera not available", (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                top_view_display = make_blank_top_view("Waiting for camera")
                mask_display = make_blank_mask("Waiting for camera")
                latest_detection = None
                latest_base_angle = None
            else:
                camera_view = draw_calibration(frame, calib)

                if calib.is_ready():
                    top_view = cv2.warpPerspective(frame, calib.homography, (TOP_WIDTH_PX, TOP_HEIGHT_PX))
                    latest_detection, mask_display = detect_yellow_square(top_view)
                    latest_base_angle = compute_base_angle(latest_detection.x_cm) if latest_detection is not None else None
                    top_view_display = draw_detection(top_view, latest_detection, latest_base_angle)
                else:
                    top_view_display = make_blank_top_view("Click 4 corners on Camera window")
                    mask_display = make_blank_mask("No calibration")
                    latest_detection = None
                    latest_base_angle = None

            gui.update_live_status(camera_ok, calib.is_ready(), latest_detection, latest_base_angle)
            gui.poll()

            cv2.imshow(WINDOW_CAMERA, camera_view)
            cv2.imshow(WINDOW_TOP, top_view_display)
            cv2.imshow(WINDOW_MASK, mask_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                gui.request_quit()
            elif key == ord("r"):
                calib.reset()
            elif key == ord("p"):
                gui.confirm_pick()
    finally:
        serial_link.close()
        cap.release()
        cv2.destroyAllWindows()
        gui.close()


if __name__ == "__main__":
    main()
