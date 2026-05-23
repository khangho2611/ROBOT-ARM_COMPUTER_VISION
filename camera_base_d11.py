import time
from dataclasses import dataclass
from typing import Optional, Tuple

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
    from tkinter import messagebox
except ImportError:
    tk = None
    messagebox = None


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

REFERENCE_DISTANCE_CM = 20.0

BASE_CENTER = 82
K_BASE = 4.0
BASE_MIN = 0
BASE_MAX = 180

SEND_SERIAL = True
SERIAL_PORT = "COM1"
BAUDRATE = 9600


# =========================
# Manual calibration angles
# =========================
# Remembered safe defaults from manual servo GUI:
# D11 Base = 82, D10 Lift = 0, D9 Extend = 0, D6 Gripper/Open = 135.
# Fill the remaining pick/drop angles after manual calibration.

HOME_D11 = 82
HOME_D10 = 0
HOME_D9 = 0
GRIP_OPEN = 135
GRIP_CLOSE = 90

PICK_UP_D10 = 0
PICK_UP_D9 = 0

PICK_DOWN_D10 = 0
PICK_DOWN_D9 = 0

DROP_D11 = 82
DROP_D10 = 0
DROP_D9 = 0


# =========================
# Safety confirmations
# =========================

REQUIRE_CAMERA_STABLE_CONFIRM = True
REQUIRE_CALIB_CONFIRM = True
REQUIRE_PICK_CONFIRM = True
REQUIRE_SERIAL_CONFIRM = True
REQUIRE_ENABLE_CONFIRM = True

CAMERA_STABLE_FRAMES = 35
CAMERA_REOPEN_DELAY_SEC = 1.0


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


class ConfirmDialog:
    def __init__(self) -> None:
        self.root = None
        if tk is not None and messagebox is not None:
            self.root = tk.Tk()
            self.root.withdraw()

    def ask(self, title: str, text: str) -> bool:
        if self.root is None or messagebox is None:
            print()
            print(f"{title}: {text}")
            answer = input("Confirm? [y/N]: ").strip().lower()
            return answer in ("y", "yes")

        try:
            self.root.update()
            return bool(messagebox.askyesno(title, text, parent=self.root))
        except tk.TclError:
            return False

    def close(self) -> None:
        if self.root is not None:
            try:
                self.root.destroy()
            except tk.TclError:
                pass


class CalibrationState:
    def __init__(self) -> None:
        self.points: list[Tuple[int, int]] = []
        self.homography: Optional[np.ndarray] = None
        self.confirmed = False
        self.needs_confirmation = False
        self.allow_clicks = False

    def reset(self) -> None:
        self.points.clear()
        self.homography = None
        self.confirmed = False
        self.needs_confirmation = False
        print("Calibration reset. Click 4 corners: top-left, top-right, bottom-right, bottom-left.")

    def add_point(self, x: int, y: int) -> None:
        if not self.allow_clicks:
            print("Camera is not confirmed yet. Wait for stable camera confirmation first.")
            return

        if self.confirmed:
            print("Calibration already confirmed. Press r to reset if you want to calibrate again.")
            return

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
        self.needs_confirmation = True
        self.confirmed = False
        print("Homography created. Check Top View and confirm calibration.")

    def has_homography(self) -> bool:
        return self.homography is not None

    def is_ready(self) -> bool:
        return self.homography is not None and self.confirmed


class RobotSerial:
    def __init__(self) -> None:
        self.link = None
        self.servos_enabled = False

    def is_connected(self) -> bool:
        return self.link is not None and self.link.is_open

    def open(self) -> bool:
        if not SEND_SERIAL:
            print("SEND_SERIAL = False. Commands will be printed only.")
            return True

        if serial is None:
            print("Missing pyserial. Install with: py -m pip install pyserial")
            return False

        if self.is_connected():
            return True

        try:
            self.link = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
            time.sleep(2.0)
            self.link.reset_input_buffer()
            print(f"Serial connected: {SERIAL_PORT} @ {BAUDRATE}")
            return True
        except Exception as exc:
            self.link = None
            print(f"Cannot open serial {SERIAL_PORT}: {exc}")
            return False

    def close(self) -> None:
        if self.link is not None and self.link.is_open:
            self.link.close()

    def read_status(self) -> None:
        if not self.is_connected():
            return

        try:
            while self.link.in_waiting > 0:
                line = self.link.readline().decode("ascii", errors="replace").strip()
                if line:
                    print(f"Arduino: {line}")
                    if "SERVOS ENABLED" in line:
                        self.servos_enabled = True
                    elif "SERVOS DETACHED" in line:
                        self.servos_enabled = False
        except Exception as exc:
            print(f"Serial read error: {exc}")
            self.close()
            self.servos_enabled = False

    def send_command(self, command: str) -> bool:
        if not SEND_SERIAL:
            print(f"Serial off. Command not sent: {command}")
            return True

        if not self.is_connected():
            print(f"Serial not connected. Command not sent: {command}")
            return False

        try:
            self.link.write((command + "\n").encode("ascii"))
            self.link.flush()
            print(f"Sent: {command}")
            return True
        except Exception as exc:
            print(f"Serial write error: {exc}")
            self.close()
            self.servos_enabled = False
            return False

    def enable_servos(self) -> bool:
        command = f"ENABLE,{HOME_D11},{HOME_D10},{HOME_D9},{GRIP_OPEN}"
        ok = self.send_command(command)
        if ok:
            self.servos_enabled = True
        return ok


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


def put_lines(image: np.ndarray, lines: list[str], x: int, y: int, color: Tuple[int, int, int]) -> None:
    for index, line in enumerate(lines):
        cv2.putText(image, line, (x, y + index * 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def draw_calibration(frame: np.ndarray, calib: CalibrationState, camera_confirmed: bool, stable_count: int) -> np.ndarray:
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

    if not camera_confirmed:
        put_lines(
            output,
            [
                f"Waiting stable camera: {min(stable_count, CAMERA_STABLE_FRAMES)}/{CAMERA_STABLE_FRAMES}",
                "No robot command can be sent yet.",
            ],
            20,
            35,
            (0, 255, 255),
        )
    elif not calib.has_homography():
        next_index = len(calib.points)
        label = CALIBRATION_LABELS[next_index] if next_index < 4 else "done"
        put_lines(
            output,
            [
                f"Click corner {next_index + 1}/4: {label}",
                "Order: top-left, top-right, bottom-right, bottom-left",
            ],
            20,
            35,
            (0, 255, 255),
        )
    elif calib.needs_confirmation:
        put_lines(output, ["Check Top View, then confirm calibration popup."], 20, 35, (0, 255, 255))
    else:
        put_lines(output, ["Ready. p=confirm PICK, r=reset calib, q=quit"], 20, 35, (0, 255, 0))

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


def draw_detection(
    top_view: np.ndarray,
    detection: Optional[Detection],
    base_angle: Optional[float],
    system_enabled: bool,
) -> np.ndarray:
    output = top_view.copy()

    center_px = (int(round(CENTER_X * PX_PER_CM)), int(round(CENTER_Y * PX_PER_CM)))
    cv2.drawMarker(output, center_px, (255, 0, 255), cv2.MARKER_CROSS, 22, 2)
    cv2.putText(output, "CENTER", (center_px[0] + 8, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

    if detection is None or base_angle is None:
        put_lines(output, ["No valid yellow square", "No PICK can be sent."], 20, 35, (0, 0, 255))
        return output

    box = cv2.boxPoints(detection.rect)
    box = np.intp(box)
    cx, cy = detection.center_px
    center = (int(round(cx)), int(round(cy)))
    dx = detection.x_cm - CENTER_X

    cv2.drawContours(output, [box], 0, (0, 255, 0), 2)
    cv2.circle(output, center, 5, (0, 0, 255), -1)
    cv2.line(output, center_px, center, (255, 255, 0), 2)

    serial_state = "enabled" if system_enabled else "not enabled"
    put_lines(
        output,
        [
            f"X={detection.x_cm:.2f} cm  Y={detection.y_cm:.2f} cm",
            f"dx={dx:.2f} cm  base={base_angle:.0f} deg",
            f"size={detection.width_cm:.2f}x{detection.height_cm:.2f} cm  area={detection.area_cm2:.2f} cm2",
            f"Robot: {serial_state}. Press p, then confirm.",
        ],
        20,
        32,
        (0, 255, 0),
    )

    return output


def confirm_camera_stable(dialog: ConfirmDialog) -> bool:
    if not REQUIRE_CAMERA_STABLE_CONFIRM:
        return True

    return dialog.ask(
        "Confirm camera",
        "Camera frames are stable.\n\nContinue to 4-point calibration?",
    )


def confirm_calibration(dialog: ConfirmDialog) -> bool:
    if not REQUIRE_CALIB_CONFIRM:
        return True

    return dialog.ask(
        "Confirm calibration",
        "Check the Top View window.\n\nUse this 4-point calibration?",
    )


def ensure_robot_enabled(serial_link: RobotSerial, dialog: ConfirmDialog) -> bool:
    if not SEND_SERIAL:
        return True

    if not serial_link.is_connected():
        if REQUIRE_SERIAL_CONFIRM:
            ok = dialog.ask(
                "Connect serial",
                f"Connect Arduino serial now?\n\nPort: {SERIAL_PORT}\nBaudrate: {BAUDRATE}",
            )
            if not ok:
                print("Serial connect cancelled.")
                return False

        if not serial_link.open():
            return False

    if not serial_link.servos_enabled:
        if REQUIRE_ENABLE_CONFIRM:
            ok = dialog.ask(
                "Enable servos",
                "Attach servos at these HOME/open angles?\n\n"
                f"D11={HOME_D11}\nD10={HOME_D10}\nD9={HOME_D9}\nD6={GRIP_OPEN}\n\n"
                "Robot may move to these angles only after you confirm.",
            )
            if not ok:
                print("Servo enable cancelled.")
                return False

        if not serial_link.enable_servos():
            return False

    return True


def confirm_and_send_pick(
    detection: Optional[Detection],
    base_angle: Optional[float],
    serial_link: RobotSerial,
    dialog: ConfirmDialog,
) -> None:
    if detection is None or base_angle is None:
        print("No valid yellow square. PICK not sent.")
        return

    angle_int = int(round(clamp(base_angle, BASE_MIN, BASE_MAX)))

    if REQUIRE_PICK_CONFIRM:
        ok = dialog.ask(
            "Confirm PICK",
            "Send PICK command to robot?\n\n"
            f"X={detection.x_cm:.2f} cm\n"
            f"Y={detection.y_cm:.2f} cm\n"
            f"D11 base_angle={angle_int}\n\n"
            f"Command: PICK,{angle_int}",
        )
        if not ok:
            print("PICK cancelled.")
            return

    if not ensure_robot_enabled(serial_link, dialog):
        print("Robot is not enabled. PICK not sent.")
        return

    serial_link.send_command(f"PICK,{angle_int}")


def main() -> None:
    dialog = ConfirmDialog()
    calib = CalibrationState()
    serial_link = RobotSerial()

    cv2.namedWindow(WINDOW_CAMERA)
    cv2.namedWindow(WINDOW_TOP)
    cv2.namedWindow(WINDOW_MASK)
    cv2.setMouseCallback(WINDOW_CAMERA, on_camera_mouse, calib)

    cap = open_camera()
    if not cap.isOpened():
        print("Camera not connected yet. The program will keep reconnecting.")

    camera_confirmed = False
    stable_count = 0
    last_frame_shape: Optional[Tuple[int, int, int]] = None
    last_reopen_time = 0.0

    latest_detection: Optional[Detection] = None
    latest_base_angle: Optional[float] = None

    print("Controls: r=reset calibration | p=confirm PICK | q=quit")
    print("Robot will not move until you confirm serial, enable servos, and confirm PICK.")

    try:
        while True:
            serial_link.read_status()

            ok, frame = cap.read() if cap.isOpened() else (False, None)
            camera_ok = ok and frame is not None

            if not camera_ok:
                stable_count = 0
                last_frame_shape = None
                now = time.monotonic()
                if now - last_reopen_time >= CAMERA_REOPEN_DELAY_SEC:
                    last_reopen_time = now
                    cap.release()
                    cap = open_camera()

                camera_view = np.zeros((480, 640, 3), dtype=np.uint8)
                put_lines(camera_view, ["Camera not available", "Check DroidCam/IP camera URL."], 25, 45, (0, 0, 255))
                top_view_display = make_blank_top_view("Waiting for camera")
                mask_display = make_blank_mask("Waiting for camera")
                latest_detection = None
                latest_base_angle = None
            else:
                frame_shape = frame.shape
                if last_frame_shape == frame_shape:
                    stable_count += 1
                else:
                    stable_count = 1
                    last_frame_shape = frame_shape

                if camera_confirmed:
                    calib.allow_clicks = True

                camera_view = draw_calibration(frame, calib, camera_confirmed, stable_count)

                if not camera_confirmed:
                    top_view_display = make_blank_top_view("Confirm stable camera first")
                    mask_display = make_blank_mask("Camera not confirmed")
                    latest_detection = None
                    latest_base_angle = None
                elif calib.has_homography():
                    top_view = cv2.warpPerspective(frame, calib.homography, (TOP_WIDTH_PX, TOP_HEIGHT_PX))
                    if calib.is_ready():
                        latest_detection, mask_display = detect_yellow_square(top_view)
                        latest_base_angle = compute_base_angle(latest_detection.x_cm) if latest_detection is not None else None
                        top_view_display = draw_detection(
                            top_view,
                            latest_detection,
                            latest_base_angle,
                            serial_link.servos_enabled,
                        )
                    else:
                        top_view_display = top_view.copy()
                        put_lines(top_view_display, ["Check Top View, then confirm calibration."], 20, 35, (0, 255, 255))
                        mask_display = make_blank_mask("Calibration not confirmed")
                        latest_detection = None
                        latest_base_angle = None
                else:
                    top_view_display = make_blank_top_view("Click 4 corners on Camera window")
                    mask_display = make_blank_mask("No calibration")
                    latest_detection = None
                    latest_base_angle = None

            cv2.imshow(WINDOW_CAMERA, camera_view)
            cv2.imshow(WINDOW_TOP, top_view_display)
            cv2.imshow(WINDOW_MASK, mask_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r"):
                calib.reset()
                latest_detection = None
                latest_base_angle = None
            if key == ord("p"):
                if not camera_confirmed:
                    print("Camera is not confirmed. PICK not sent.")
                elif not calib.is_ready():
                    print("Calibration is not confirmed. PICK not sent.")
                else:
                    confirm_and_send_pick(latest_detection, latest_base_angle, serial_link, dialog)

            if camera_ok and not camera_confirmed and stable_count >= CAMERA_STABLE_FRAMES:
                if confirm_camera_stable(dialog):
                    camera_confirmed = True
                    calib.allow_clicks = True
                    print("Camera confirmed. Click 4 calibration corners.")
                else:
                    stable_count = 0
                    print("Camera confirmation rejected. Waiting for stable camera again.")

            if calib.needs_confirmation:
                if confirm_calibration(dialog):
                    calib.confirmed = True
                    calib.needs_confirmation = False
                    print("Calibration confirmed. Detection is enabled.")
                else:
                    calib.reset()
                    print("Calibration rejected. Click 4 corners again.")
    finally:
        serial_link.close()
        cap.release()
        cv2.destroyAllWindows()
        dialog.close()


if __name__ == "__main__":
    main()
