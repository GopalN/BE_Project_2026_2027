import csv
import os
import queue
import threading
import time
import uuid
from collections import deque
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
except ImportError:
    Figure = None
    FigureCanvasTkAgg = None

APP_TITLE = "EOG Event-Triggered Dataset Recorder"
DEFAULT_PORT = "COM17"
DEFAULT_BAUD = 115200
DEFAULT_FS = 250
PRETRIGGER_SECONDS = 0.5
PLOT_SECONDS = 8


class EOGRecorderApp:
    def __init__(self, root):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1220x820")
        self.root.minsize(1000, 700)

        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.fs_var = tk.StringVar(value=str(DEFAULT_FS))
        self.duration_var = tk.StringVar(value="5")
        self.angle_var = tk.StringVar(value="30")
        self.subject_var = tk.StringVar(value="P01")
        self.montage_var = tk.StringVar(value="M1")
        self.status_var = tk.StringVar(value="Disconnected")
        self.capture_var = tk.StringVar(value="Not recording")
        self.samples_var = tk.StringVar(value="Saved samples: 0")
        self.last_var = tk.StringVar(value="Last sample: —")

        self.ser = None
        self.reader_thread = None
        self.stop_reader = threading.Event()
        self.incoming = queue.Queue(maxsize=10000)
        self.lock = threading.Lock()

        self.session_id = datetime.now().strftime("S_%Y%m%d_%H%M%S")
        self.trial_counter = 0
        self.active_trial = None
        self.writer_file = None
        self.writer = None
        self.saved_samples = 0
        self.rest_timer_id = None

        self.pretrigger_n = int(DEFAULT_FS * PRETRIGGER_SECONDS)
        self.prebuffer = deque(maxlen=self.pretrigger_n)
        self.plot_h = deque(maxlen=DEFAULT_FS * PLOT_SECONDS)
        self.plot_v = deque(maxlen=DEFAULT_FS * PLOT_SECONDS)
        self.plot_t = deque(maxlen=DEFAULT_FS * PLOT_SECONDS)
        self.latest_sample = None
        self.last_timestamp = None
        self.last_host_time = None
        self.sample_count = 0
        self.bad_lines = 0

        self._build_ui()
        self.root.after(30, self._drain_queue)
        self.root.after(200, self._refresh_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text=APP_TITLE, font=("Segoe UI", 17, "bold")).pack(anchor="w")
        ttk.Label(
            outer,
            text="Only committed trials are written to CSV. Idle serial samples stay in a short RAM pre-trigger buffer.",
            foreground="#555"
        ).pack(anchor="w", pady=(2, 10))

        connection = ttk.LabelFrame(outer, text="Acquisition connection", padding=8)
        connection.pack(fill="x", pady=(0, 8))

        ttk.Label(connection, text="Serial port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=12, values=self._ports())
        self.port_combo.grid(row=0, column=1, padx=(5, 12), sticky="w")
        ttk.Button(connection, text="Refresh ports", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 12))
        ttk.Label(connection, text="Baud").grid(row=0, column=3, sticky="w")
        ttk.Entry(connection, textvariable=self.baud_var, width=9).grid(row=0, column=4, padx=(5, 12))
        ttk.Label(connection, text="Target Fs (Hz)").grid(row=0, column=5, sticky="w")
        ttk.Entry(connection, textvariable=self.fs_var, width=7).grid(row=0, column=6, padx=(5, 12))
        self.connect_button = ttk.Button(connection, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=7, padx=(0, 5))
        self.disconnect_button = ttk.Button(connection, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_button.grid(row=0, column=8)
        ttk.Label(connection, textvariable=self.status_var).grid(row=1, column=0, columnspan=9, sticky="w", pady=(7, 0))

        metadata = ttk.LabelFrame(outer, text="Trial metadata", padding=8)
        metadata.pack(fill="x", pady=(0, 8))
        ttk.Label(metadata, text="Participant ID").grid(row=0, column=0, sticky="w")
        ttk.Entry(metadata, textvariable=self.subject_var, width=12).grid(row=0, column=1, padx=(5, 18))
        ttk.Label(metadata, text="Montage ID").grid(row=0, column=2, sticky="w")
        ttk.Entry(metadata, textvariable=self.montage_var, width=10).grid(row=0, column=3, padx=(5, 18))
        ttk.Label(metadata, text="REST duration (seconds)").grid(row=0, column=4, sticky="w")
        ttk.Entry(metadata, textvariable=self.duration_var, width=7).grid(row=0, column=5, padx=(5, 18))
        ttk.Label(metadata, text="Target angle (°)").grid(row=0, column=6, sticky="w")
        self.angle_combo = ttk.Combobox(
            metadata, textvariable=self.angle_var, width=8, state="readonly",
            values=[str(a) for a in range(-30, 31, 5)]
        )
        self.angle_combo.grid(row=0, column=7, padx=(5, 18), sticky="w")
        ttk.Label(metadata, text=f"Session: {self.session_id}").grid(row=0, column=8, sticky="w")

        actions = ttk.LabelFrame(outer, text="Capture controls", padding=8)
        actions.pack(fill="x", pady=(0, 8))
        labels = [
            ("UP", "#dcecff"), ("DOWN", "#dcecff"),
            ("LEFT", "#dcecff"), ("RIGHT", "#dcecff"),
            ("BLINK_1", "#fce8d5"), ("BLINK_2", "#fce8d5"), ("BLINK_3", "#fce8d5"),
        ]
        for idx, (label, _) in enumerate(labels):
            ttk.Button(actions, text=f"Capture {label}", command=lambda x=label: self.start_capture(x)).grid(
                row=0, column=idx, padx=3, pady=3, sticky="ew"
            )
            actions.columnconfigure(idx, weight=1)
        ttk.Button(actions, text="Capture REST (timed)", command=self.start_rest_capture).grid(
            row=1, column=0, columnspan=2, padx=3, pady=5, sticky="ew"
        )
        ttk.Button(actions, text="Stop Capture / Save Trial", command=self.stop_capture).grid(
            row=1, column=2, columnspan=2, padx=3, pady=5, sticky="ew"
        )
        ttk.Button(actions, text="Discard Current Trial", command=self.discard_capture).grid(
            row=1, column=4, columnspan=2, padx=3, pady=5, sticky="ew"
        )
        ttk.Button(actions, text="Choose CSV Folder", command=self.choose_folder).grid(
            row=1, column=6, padx=3, pady=5, sticky="ew"
        )

        info = ttk.Frame(outer)
        info.pack(fill="x", pady=(0, 8))
        ttk.Label(info, textvariable=self.capture_var, font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Label(info, textvariable=self.samples_var).pack(side="left", padx=18)
        ttk.Label(info, textvariable=self.last_var).pack(side="left", padx=18)

        self.csv_folder = os.path.join(os.getcwd(), "recordings")
        os.makedirs(self.csv_folder, exist_ok=True)
        self.folder_var = tk.StringVar(value=self.csv_folder)
        folder_line = ttk.Frame(outer)
        folder_line.pack(fill="x", pady=(0, 6))
        ttk.Label(folder_line, text="Output folder:").pack(side="left")
        ttk.Label(folder_line, textvariable=self.folder_var, foreground="#444").pack(side="left", padx=6)

        if Figure is not None:
            self.fig = Figure(figsize=(10, 4.2), dpi=100)
            self.ax_h = self.fig.add_subplot(211)
            self.ax_v = self.fig.add_subplot(212, sharex=self.ax_h)
            self.ax_h.set_title("Horizontal EOG (raw ADC counts)")
            self.ax_v.set_title("Vertical EOG (raw ADC counts)")
            self.ax_h.set_ylabel("ADC")
            self.ax_v.set_ylabel("ADC")
            self.ax_v.set_xlabel("Seconds from latest sample")
            self.line_h, = self.ax_h.plot([], [], linewidth=0.8)
            self.line_v, = self.ax_v.plot([], [], linewidth=0.8)
            self.fig.tight_layout()
            self.canvas = FigureCanvasTkAgg(self.fig, master=outer)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            ttk.Label(outer, text="Live plots unavailable. Install matplotlib to enable plots.").pack()

        ttk.Label(
            outer,
            text="Choose Target angle before Capture UP/DOWN/LEFT/RIGHT; selected angle is saved with every sample. REST=0°, blink angle=N/A. Raw ADC data are saved without filtering.",
            foreground="#555"
        ).pack(anchor="w", pady=(6, 0))

    def _ports(self):
        if list_ports is None:
            return []
        try:
            return [p.device for p in list_ports.comports()]
        except Exception:
            return []

    def refresh_ports(self):
        ports = self._ports()
        self.port_combo["values"] = ports
        if self.port_var.get() not in ports and ports:
            self.port_var.set(ports[0])

    def connect(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "pyserial is not installed.\nRun: pip install -r requirements.txt")
            return
        if self.ser is not None:
            return
        try:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            fs = int(self.fs_var.get())
            if not port:
                raise ValueError("Select a serial port.")
            if fs < 1:
                raise ValueError("Target sample rate must be positive.")
            self.pretrigger_n = max(1, int(fs * PRETRIGGER_SECONDS))
            self.prebuffer = deque(self.prebuffer, maxlen=self.pretrigger_n)
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(1.8)  # Arduino Uno may reset when serial opens
            self.ser.reset_input_buffer()
            self.stop_reader.clear()
            self.reader_thread = threading.Thread(target=self._serial_worker, daemon=True)
            self.reader_thread.start()
            self.status_var.set(f"Connected to {port} at {baud} baud")
            self.connect_button.configure(state="disabled")
            self.disconnect_button.configure(state="normal")
        except Exception as exc:
            self.ser = None
            messagebox.showerror("Connection failed", str(exc))

    def _serial_worker(self):
        while not self.stop_reader.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) < 3:
                    continue
                try:
                    ts = int(parts[0])
                    h = int(parts[1])
                    v = int(parts[2])
                except ValueError:
                    self.bad_lines += 1
                    continue
                sample = {
                    "timestamp_us": ts,
                    "horizontal_adc": h,
                    "vertical_adc": v,
                    "host_time": time.time(),
                }
                try:
                    self.incoming.put_nowait(sample)
                except queue.Full:
                    self.bad_lines += 1
            except Exception as exc:
                if not self.stop_reader.is_set():
                    try:
                        self.incoming.put_nowait({"error": str(exc)})
                    except queue.Full:
                        pass
                break

    def disconnect(self):
        if self.active_trial is not None:
            self.stop_capture()
        self.stop_reader.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")
        self.status_var.set("Disconnected")

    def start_capture(self, label):
        if self.ser is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino/ADS1115 stream first.")
            return
        if self.active_trial is not None:
            messagebox.showwarning("Capture active", "Stop or discard the current trial first.")
            return
        self._begin_trial(label, timed_seconds=None)

    def start_rest_capture(self):
        if self.ser is None:
            messagebox.showwarning("Not connected", "Connect to the Arduino/ADS1115 stream first.")
            return
        if self.active_trial is not None:
            messagebox.showwarning("Capture active", "Stop or discard the current trial first.")
            return
        try:
            duration = float(self.duration_var.get())
            if duration <= 0 or duration > 300:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid duration", "REST duration must be between 0 and 300 seconds.")
            return
        self._begin_trial("REST", timed_seconds=duration)

    def _begin_trial(self, label, timed_seconds=None):
        self.trial_counter += 1
        trial_id = f"T{self.trial_counter:04d}"
        # The selected angle is captured as metadata for every movement trial.
        # REST is fixed at 0 degrees; blink events have no angular target.
        if label in ("UP", "DOWN", "LEFT", "RIGHT"):
            try:
                target_angle = int(self.angle_var.get())
            except (TypeError, ValueError):
                messagebox.showerror("Invalid angle", "Select a target angle before capturing a movement.")
                self.trial_counter -= 1
                return
        elif label == "REST":
            target_angle = 0
        else:
            target_angle = ""
        self.active_trial = {
            "trial_id": trial_id,
            "label": label,
            "target_angle_deg": target_angle,
            "target_angle_label": (f"{target_angle:+d} deg" if target_angle != "" else ""),
            "session_id": self.session_id,
            "participant_id": self.subject_var.get().strip() or "P01",
            "montage_id": self.montage_var.get().strip() or "M1",
            "started_host_time": time.time(),
            "sample_count": 0,
            "timed_seconds": timed_seconds,
            "start_timestamp_us": None,
        }
        # Pre-trigger data are saved with phase=pretrigger so downstream training can exclude them.
        for sample in list(self.prebuffer):
            self._write_sample(sample, phase="pretrigger")
        self.capture_var.set(f"RECORDING: {label} ({trial_id})")
        self.status_var.set(f"Capture started: {label}")
        if timed_seconds is not None:
            self.rest_timer_id = self.root.after(int(timed_seconds * 1000), self.stop_capture)

    def _write_sample(self, sample, phase="action"):
        if self.writer is None:
            path = os.path.join(self.csv_folder, f"eog_samples_{self.session_id}.csv")
            new_file = not os.path.exists(path) or os.path.getsize(path) == 0
            self.writer_file = open(path, "a", newline="", encoding="utf-8")
            self.writer = csv.writer(self.writer_file)
            if new_file:
                self.writer.writerow([
                    "timestamp_us", "host_time_iso", "horizontal_adc", "vertical_adc",
                    "label", "target_angle_deg", "target_angle_label", "phase", "trial_id",
                    "session_id", "participant_id", "montage_id"
                ])
        trial = self.active_trial
        if trial is None:
            return
        if trial["start_timestamp_us"] is None:
            trial["start_timestamp_us"] = sample["timestamp_us"]
        iso = datetime.fromtimestamp(sample["host_time"]).isoformat(timespec="milliseconds")
        self.writer.writerow([
            sample["timestamp_us"], iso, sample["horizontal_adc"], sample["vertical_adc"],
            trial["label"], trial["target_angle_deg"], trial["target_angle_label"],
            phase, trial["trial_id"], trial["session_id"], trial["participant_id"], trial["montage_id"]
        ])
        trial["sample_count"] += 1
        self.saved_samples += 1

    def stop_capture(self):
        if self.rest_timer_id is not None:
            try:
                self.root.after_cancel(self.rest_timer_id)
            except Exception:
                pass
            self.rest_timer_id = None
        if self.active_trial is None:
            return
        trial = self.active_trial
        if self.writer_file:
            self.writer_file.flush()
            self.writer_file.close()
            self.writer_file = None
            self.writer = None
        self.active_trial = None
        self.capture_var.set(f"Saved {trial['label']} — {trial['trial_id']} ({trial['sample_count']} samples)")
        self.status_var.set("Trial saved")
        self.samples_var.set(f"Saved samples this session: {self.saved_samples}")

    def discard_capture(self):
        if self.active_trial is None:
            return
        if self.rest_timer_id is not None:
            try:
                self.root.after_cancel(self.rest_timer_id)
            except Exception:
                pass
            self.rest_timer_id = None
        trial_id = self.active_trial["trial_id"]
        # Samples may already have been appended; remove this trial from the session CSV.
        if self.writer_file:
            self.writer_file.flush()
            self.writer_file.close()
            self.writer_file = None
            self.writer = None
        path = os.path.join(self.csv_folder, f"eog_samples_{self.session_id}.csv")
        if os.path.exists(path):
            try:
                with open(path, "r", newline="", encoding="utf-8") as f:
                    rows = list(csv.reader(f))
                if rows:
                    header = rows[0]
                    tid_idx = header.index("trial_id")
                    kept = [rows[0]] + [r for r in rows[1:] if len(r) > tid_idx and r[tid_idx] != trial_id]
                    with open(path, "w", newline="", encoding="utf-8") as f:
                        csv.writer(f).writerows(kept)
            except Exception as exc:
                messagebox.showerror("Discard failed", f"Could not remove trial from CSV:\n{exc}")
        self.active_trial = None
        self.capture_var.set(f"Discarded {trial_id}")
        self.status_var.set("Current trial discarded")

    def choose_folder(self):
        folder = filedialog.askdirectory(initialdir=self.csv_folder, title="Choose dataset output folder")
        if folder:
            self.csv_folder = folder
            self.folder_var.set(folder)

    def _drain_queue(self):
        processed = 0
        while processed < 3000:
            try:
                sample = self.incoming.get_nowait()
            except queue.Empty:
                break
            processed += 1
            if "error" in sample:
                self.status_var.set(f"Serial read error: {sample['error']}")
                continue
            self.latest_sample = sample
            self.sample_count += 1
            self.last_timestamp = sample["timestamp_us"]
            self.last_host_time = sample["host_time"]
            self.prebuffer.append(sample)
            self.plot_h.append(sample["horizontal_adc"])
            self.plot_v.append(sample["vertical_adc"])
            self.plot_t.append(sample["host_time"])
            if self.active_trial is not None:
                # First samples already in prebuffer were saved as pretrigger at start.
                # Avoid duplicate writes for samples that were present in prebuffer.
                self._write_sample(sample, phase="action")
        if self.latest_sample is not None:
            self.last_var.set(
                f"Last sample: H={self.latest_sample['horizontal_adc']}  "
                f"V={self.latest_sample['vertical_adc']}  "
                f"t={self.latest_sample['timestamp_us']}"
            )
        self.samples_var.set(f"Saved samples this session: {self.saved_samples}")
        self.root.after(30, self._drain_queue)

    def _refresh_plot(self):
        if Figure is not None and self.plot_t:
            now = self.plot_t[-1]
            xs = [t - now for t in self.plot_t]
            self.line_h.set_data(xs, list(self.plot_h))
            self.line_v.set_data(xs, list(self.plot_v))
            self.ax_h.set_xlim(-PLOT_SECONDS, 0)
            self.ax_v.set_xlim(-PLOT_SECONDS, 0)
            self.ax_h.relim()
            self.ax_h.autoscale_view(scalex=False, scaley=True)
            self.ax_v.relim()
            self.ax_v.autoscale_view(scalex=False, scaley=True)
            self.canvas.draw_idle()
        self.root.after(200, self._refresh_plot)

    def close(self):
        if self.active_trial is not None:
            choice = messagebox.askyesnocancel("Exit", "A trial is recording. Save it before exit?")
            if choice is None:
                return
            if choice:
                self.stop_capture()
            else:
                self.discard_capture()
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = EOGRecorderApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
