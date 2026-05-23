import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


# =========================
# Serial configuration
# =========================

SEND_SERIAL = True
DEFAULT_SERIAL_PORT = "COM1"
DEFAULT_BAUDRATE = 9600


# =========================
# Remembered default angles
# =========================
# These are the saved startup/default values from the servo GUI:
# D11 Base = 82, D10 Lift = 0, D9 Extend = 0, D6 Gripper = 135.

SERVO_DEFAULTS = {
    "D11": 82,
    "D10": 0,
    "D9": 0,
    "D6": 135,
}

SERVO_NAMES = {
    "D11": "Base",
    "D10": "Lift",
    "D9": "Extend",
    "D6": "Gripper",
}

SERVO_MIN = 0
SERVO_MAX = 180


class SerialLink:
    def __init__(self) -> None:
        self.link = None
        self.port = ""
        self.baudrate = DEFAULT_BAUDRATE

    def is_connected(self) -> bool:
        return self.link is not None and self.link.is_open

    def connect(self, port: str, baudrate: int) -> str:
        if not SEND_SERIAL:
            return "SEND_SERIAL = False"
        if serial is None:
            return "Missing pyserial. Install: py -m pip install pyserial"
        if self.is_connected():
            return f"Already connected: {self.port}"

        port = port.strip()
        if not port:
            return "No COM port selected"

        try:
            self.link = serial.Serial(port, baudrate, timeout=0.05)
            time.sleep(2.0)
            self.link.reset_input_buffer()
            self.port = port
            self.baudrate = baudrate
            return f"Connected: {port} @ {baudrate}"
        except Exception as exc:
            self.link = None
            return f"Connect failed: {exc}"

    def disconnect(self) -> str:
        if self.is_connected():
            self.link.close()
        return "Disconnected"

    def send(self, command: str) -> str:
        if not SEND_SERIAL:
            return f"Serial off. Not sent: {command}"
        if not self.is_connected():
            return f"Not connected. Not sent: {command}"

        try:
            self.link.write((command + "\n").encode("ascii"))
            self.link.flush()
            return f"Sent: {command}"
        except Exception as exc:
            self.disconnect()
            return f"Send failed: {exc}"

    def read_lines(self) -> list[str]:
        if not self.is_connected():
            return []

        lines = []
        try:
            while self.link.in_waiting > 0:
                line = self.link.readline().decode("ascii", errors="replace").strip()
                if line:
                    lines.append(line)
        except Exception as exc:
            lines.append(f"Read failed: {exc}")
            self.disconnect()

        return lines


class ServoControlApp:
    def __init__(self) -> None:
        self.serial_link = SerialLink()

        self.root = tk.Tk()
        self.root.title("Robot Arm Servo Control")
        self.root.geometry("940x560")
        self.root.minsize(820, 500)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._setup_style()

        self.status_var = tk.StringVar(value="Serial: disconnected")
        self.port_var = tk.StringVar(value=DEFAULT_SERIAL_PORT)
        self.baud_var = tk.IntVar(value=DEFAULT_BAUDRATE)
        self.enabled_var = tk.StringVar(value="Servos: disabled")

        self.angle_vars = {
            servo_id: tk.IntVar(value=angle)
            for servo_id, angle in SERVO_DEFAULTS.items()
        }

        self._build_ui()
        self.refresh_ports()
        self.log("Ready. Defaults loaded: D11=82, D10=0, D9=0, D6=135.")
        self.root.after(100, self.poll_serial)

    def _setup_style(self) -> None:
        self.root.configure(bg="#f5f6f8")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#f5f6f8")
        style.configure("Panel.TFrame", background="#ffffff", relief="solid", borderwidth=1)
        style.configure("TLabel", background="#f5f6f8", font=("Segoe UI", 10))
        style.configure("Panel.TLabel", background="#ffffff", font=("Segoe UI", 10))
        style.configure("Title.TLabel", background="#f5f6f8", font=("Segoe UI", 16, "bold"))
        style.configure("Small.TLabel", background="#f5f6f8", font=("Segoe UI", 9))
        style.configure("TButton", font=("Segoe UI", 10), padding=(10, 5))
        style.configure("Danger.TButton", font=("Segoe UI", 10, "bold"), padding=(10, 5))

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(2, weight=1)

        header = ttk.Frame(main)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text="Robot Arm Servo Control", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            header,
            text="Default: D11=82 | D10=0 | D9=0 | D6=135",
            style="Small.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(3, 0))

        left = ttk.Frame(main, style="Panel.TFrame", padding=10)
        left.grid(row=1, column=0, rowspan=2, sticky="ns", padx=(0, 10))

        right = ttk.Frame(main, style="Panel.TFrame", padding=10)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(1, weight=1)

        log_panel = ttk.Frame(main, style="Panel.TFrame", padding=10)
        log_panel.grid(row=2, column=1, sticky="nsew", pady=(10, 0))
        log_panel.rowconfigure(1, weight=1)
        log_panel.columnconfigure(0, weight=1)

        self._build_connection_panel(left)
        self._build_actions_panel(left)
        self._build_servo_panel(right)
        self._build_log_panel(log_panel)

    def _build_connection_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Connection", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(0, 8),
        )

        ttk.Label(parent, textvariable=self.status_var, style="Panel.TLabel").grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(0, 8),
        )

        ttk.Label(parent, text="COM", style="Panel.TLabel").grid(row=2, column=0, sticky="w")
        self.port_combo = ttk.Combobox(parent, textvariable=self.port_var, width=16)
        self.port_combo.grid(row=2, column=1, sticky="ew", pady=3)

        ttk.Label(parent, text="Baud", style="Panel.TLabel").grid(row=3, column=0, sticky="w")
        ttk.Combobox(
            parent,
            textvariable=self.baud_var,
            values=(9600, 19200, 38400, 57600, 115200),
            width=16,
        ).grid(row=3, column=1, sticky="ew", pady=3)

        ttk.Button(parent, text="Refresh", command=self.refresh_ports).grid(row=4, column=0, sticky="ew", pady=(8, 3))
        ttk.Button(parent, text="Connect", command=self.connect).grid(row=4, column=1, sticky="ew", pady=(8, 3))
        ttk.Button(parent, text="Disconnect", command=self.disconnect).grid(row=5, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(parent, text="Status", command=self.ask_status).grid(row=6, column=0, columnspan=2, sticky="ew", pady=3)

    def _build_actions_panel(self, parent: ttk.Frame) -> None:
        ttk.Separator(parent).grid(row=7, column=0, columnspan=2, sticky="ew", pady=14)

        ttk.Label(parent, text="Robot", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).grid(
            row=8,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(0, 8),
        )
        ttk.Label(parent, textvariable=self.enabled_var, style="Panel.TLabel").grid(
            row=9,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(0, 8),
        )

        ttk.Button(parent, text="ENABLE SERVOS", command=self.enable_servos, style="Danger.TButton").grid(
            row=10,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=3,
        )
        ttk.Button(parent, text="DETACH SERVOS", command=self.detach_servos).grid(
            row=11,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=3,
        )
        ttk.Button(parent, text="HOME", command=self.home).grid(row=12, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(parent, text="Load Defaults", command=self.load_defaults).grid(
            row=13,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=3,
        )
        ttk.Button(parent, text="Send All", command=self.send_all).grid(row=14, column=0, columnspan=2, sticky="ew", pady=3)

    def _build_servo_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Servo Angles", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).grid(
            row=0,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(0, 10),
        )

        for row, servo_id in enumerate(("D11", "D10", "D9", "D6"), start=1):
            self._add_servo_row(parent, row, servo_id)

    def _build_log_panel(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Log", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        self.log_text = tk.Text(
            parent,
            height=9,
            state="disabled",
            bg="#111827",
            fg="#e5e7eb",
            insertbackground="#e5e7eb",
            relief="flat",
            font=("Consolas", 10),
        )
        self.log_text.grid(row=1, column=0, sticky="nsew", pady=(8, 0))

    def _add_servo_row(self, parent: ttk.Frame, row: int, servo_id: str) -> None:
        var = self.angle_vars[servo_id]
        label = f"{servo_id} {SERVO_NAMES[servo_id]}"

        ttk.Label(parent, text=label, style="Panel.TLabel", width=14).grid(row=row, column=0, sticky="w", pady=8)

        slider = tk.Scale(
            parent,
            from_=SERVO_MIN,
            to=SERVO_MAX,
            orient="horizontal",
            resolution=1,
            variable=var,
            showvalue=True,
            length=430,
            bg="#ffffff",
            highlightthickness=0,
            troughcolor="#d6d9de",
        )
        slider.grid(row=row, column=1, sticky="ew", padx=8, pady=8)

        spinbox = ttk.Spinbox(parent, from_=SERVO_MIN, to=SERVO_MAX, textvariable=var, width=6)
        spinbox.grid(row=row, column=2, sticky="w", padx=8, pady=8)

        ttk.Button(parent, text="Send", command=lambda sid=servo_id: self.send_one(sid)).grid(
            row=row,
            column=3,
            sticky="ew",
            padx=(0, 2),
            pady=8,
        )

    def angle(self, servo_id: str) -> int:
        value = self.angle_vars[servo_id].get()
        return max(SERVO_MIN, min(SERVO_MAX, int(value)))

    def log(self, message: str) -> None:
        print(message)
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def refresh_ports(self) -> None:
        ports = []
        if list_ports is not None:
            ports = [port.device for port in list_ports.comports()]

        current = self.port_var.get().strip()
        for port in (current, DEFAULT_SERIAL_PORT):
            if port and port not in ports:
                ports.insert(0, port)

        self.port_combo.configure(values=ports)
        if not current and ports:
            self.port_var.set(ports[0])

    def update_status(self) -> None:
        if self.serial_link.is_connected():
            self.status_var.set(f"Serial: connected {self.serial_link.port} @ {self.serial_link.baudrate}")
        else:
            self.status_var.set("Serial: disconnected")

    def connect(self) -> None:
        try:
            baudrate = int(self.baud_var.get())
        except (tk.TclError, ValueError):
            self.log("Invalid baudrate")
            return

        self.log(self.serial_link.connect(self.port_var.get(), baudrate))
        self.update_status()

    def disconnect(self) -> None:
        self.log(self.serial_link.disconnect())
        self.enabled_var.set("Servos: disabled")
        self.update_status()

    def enable_servos(self) -> None:
        d11 = self.angle("D11")
        d10 = self.angle("D10")
        d9 = self.angle("D9")
        d6 = self.angle("D6")

        ok = messagebox.askyesno(
            "Enable servos",
            "Attach servos at these angles?\n\n"
            f"D11={d11}\nD10={d10}\nD9={d9}\nD6={d6}",
        )
        if not ok:
            self.log("ENABLE cancelled")
            return

        self.log(self.serial_link.send(f"ENABLE,{d11},{d10},{d9},{d6}"))
        self.enabled_var.set("Servos: enabled")
        self.update_status()

    def detach_servos(self) -> None:
        self.log(self.serial_link.send("DETACH"))
        self.enabled_var.set("Servos: disabled")
        self.update_status()

    def send_one(self, servo_id: str) -> None:
        angle = self.angle(servo_id)
        self.log(self.serial_link.send(f"SET,{servo_id},{angle}"))
        self.update_status()

    def send_all(self) -> None:
        d11 = self.angle("D11")
        d10 = self.angle("D10")
        d9 = self.angle("D9")
        d6 = self.angle("D6")
        self.log(self.serial_link.send(f"SETALL,{d11},{d10},{d9},{d6}"))
        self.update_status()

    def home(self) -> None:
        ok = messagebox.askyesno("HOME", "Move robot to HOME now?")
        if not ok:
            self.log("HOME cancelled")
            return

        self.log(self.serial_link.send("HOME"))
        self.update_status()

    def ask_status(self) -> None:
        self.log(self.serial_link.send("STATUS"))
        self.update_status()

    def load_defaults(self) -> None:
        for servo_id, angle in SERVO_DEFAULTS.items():
            self.angle_vars[servo_id].set(angle)
        self.log("Defaults loaded into sliders. Nothing sent.")

    def poll_serial(self) -> None:
        for line in self.serial_link.read_lines():
            self.log(f"Arduino: {line}")
            if "SERVOS ENABLED" in line:
                self.enabled_var.set("Servos: enabled")
            elif "SERVOS DETACHED" in line:
                self.enabled_var.set("Servos: disabled")
        self.update_status()
        self.root.after(100, self.poll_serial)

    def on_close(self) -> None:
        self.serial_link.disconnect()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    app = ServoControlApp()
    app.run()
