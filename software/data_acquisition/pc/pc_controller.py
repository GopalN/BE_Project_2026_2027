"""EOG SVR controller with a Tkinter button to cycle J1, J2 and J3."""
import os
import json
import time
import threading
import queue
import tkinter as tk
import subprocess
from collections import deque

import joblib
import numpy as np
import pandas as pd
import serial

# ---------------- USER CONFIGURATION ----------------
ACQUISITION_PORT = "COM20"  # ESP32-C3 Mini EOG stream
ROBOT_PORT = "COM19"        # Shrike-FI USB serial receiver
BAUD = 115200
FS = 250
WINDOW_SECONDS = 0.5
OVERLAP = 0.5
WINDOW = int(FS * WINDOW_SECONDS)
HOP = max(1, int(WINDOW * (1 - OVERLAP)))

# Continuous control with prediction smoothing.
# Safety: set ROBOT_ARMED=False for a dry run before the first physical test.
ROBOT_ARMED = True
SVR_MIN_ANGLE_DEG = -20.0
SVR_MAX_ANGLE_DEG = 20.0
SVR_ANGLE_GAIN = 1.5  # increases travel while retaining the same -20..+20 clamp

# Conservative J1 neutral-drift suppression. Applied only after the saved
# neutral calibration offset and prediction smoothing; J2/J3 are untouched.
J1_DEADBAND_ENTER_DEG = 2.0
J1_DEADBAND_EXIT_DEG = 1.2
JOINT_MIN_POSITION = 300
JOINT_MAX_POSITION = 800
JOINT_HOME_POSITION = 500

# Filters run once per SVR prediction (not at the servo's 100 Hz internal rate).
# At FS=250 Hz, WINDOW=125 and HOP=62, predictions arrive about every 0.248 s.
MOVING_AVERAGE_PREDICTIONS = 3
LOW_PASS_ALPHA = 0.55       # 0..1; smaller = smoother but more lag
MIN_POSITION_CHANGE = 3     # ignore tiny changes in servo counts
MIN_COMMAND_INTERVAL_S = 0.02  # 20 ms minimum interval; model predictions are slower
MAX_POSITION_STEP_PER_UPDATE = 50  # slew limit in servo counts per SVR prediction

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SVR_PATH = os.path.join(BASE_DIR, "eog_svr_angle_model.joblib")
SVM_PATH = os.path.join(BASE_DIR, "eog_svm_movement_model.joblib")
META_PATH = os.path.join(BASE_DIR, "training_metadata.json")

for path in (SVR_PATH, SVM_PATH, META_PATH):
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Required model file missing: {path}\n"
            "Copy eog_svr_angle_model.joblib, eog_svm_movement_model.joblib, "
            "and training_metadata.json into this pc/ folder."
        )

with open(META_PATH, "r", encoding="utf-8") as f:
    metadata = json.load(f)
FEATURE_COLUMNS = metadata["feature_columns"]
svr_model = joblib.load(SVR_PATH)
# Load SVM only to preserve model bundle compatibility; it does not gate motion.
svm_model = joblib.load(SVM_PATH)

def channel_features(values, prefix):
    x = np.asarray(values, dtype=np.float64)
    d = np.diff(x)
    slope = np.polyfit(np.arange(len(x)), x, 1)[0] if len(x) > 1 else 0.0
    return {
        f"{prefix}_mean": float(np.mean(x)),
        f"{prefix}_std": float(np.std(x)),
        f"{prefix}_median": float(np.median(x)),
        f"{prefix}_min": float(np.min(x)),
        f"{prefix}_max": float(np.max(x)),
        f"{prefix}_range": float(np.ptp(x)),
        f"{prefix}_rms": float(np.sqrt(np.mean(x ** 2))),
        f"{prefix}_p10": float(np.percentile(x, 10)),
        f"{prefix}_p90": float(np.percentile(x, 90)),
        f"{prefix}_slope": float(slope),
        f"{prefix}_delta": float(x[-1] - x[0]),
        f"{prefix}_diff_abs_mean": float(np.mean(np.abs(d))) if len(d) else 0.0,
        f"{prefix}_diff_std": float(np.std(d)) if len(d) else 0.0,
    }

def extract_features(horizontal, vertical):
    h = np.asarray(horizontal, dtype=np.float64)
    v = np.asarray(vertical, dtype=np.float64)
    features = {}
    features.update(channel_features(h, "h"))
    features.update(channel_features(v, "v"))
    diff = h - v
    features["hv_mean_difference"] = float(np.mean(diff))
    features["hv_std_difference"] = float(np.std(diff))
    missing = [name for name in FEATURE_COLUMNS if name not in features]
    if missing:
        raise ValueError(f"Feature implementation does not match metadata. Missing: {missing}")
    return pd.DataFrame([[features[name] for name in FEATURE_COLUMNS]], columns=FEATURE_COLUMNS)

def map_angle_to_position(angle):
    """
    Piecewise-linear SVR angle mapping for Joint 1:
      -20 -> 300
      -15 -> 350
      -10 -> 400
       -5 -> 450
        0 -> 500
       +5 -> 550
      +10 -> 600
      +15 -> 650
      +20 -> 800

    Joint 2 and Joint 3 remain fixed at 500 in the command sender.
    """
    angle = max(-20.0, min(20.0, float(angle)))

    calibration_points = [
        (-20.0, 300),
        (-15.0, 350),
        (-10.0, 400),
        (-5.0, 450),
        (0.0, 500),
        (5.0, 550),
        (10.0, 600),
        (15.0, 650),
        (20.0, 800),
    ]

    for i in range(len(calibration_points) - 1):
        a0, p0 = calibration_points[i]
        a1, p1 = calibration_points[i + 1]

        if a0 <= angle <= a1:
            fraction = (angle - a0) / (a1 - a0)
            position = p0 + fraction * (p1 - p0)
            return int(round(position)), angle

    # Defensive fallback; angle is already clamped.
    return 500, angle


def map_position_to_eye_angle(position):
    """
    Inverse of map_angle_to_position(). Converts the servo position count
    back into the equivalent EOG/SVR angle using the same piecewise mapping.
    """
    position = max(300.0, min(800.0, float(position)))
    calibration_points = [
        (-20.0, 300.0),
        (-15.0, 350.0),
        (-10.0, 400.0),
        (-5.0, 450.0),
        (0.0, 500.0),
        (5.0, 550.0),
        (10.0, 600.0),
        (15.0, 650.0),
        (20.0, 800.0),
    ]
    for i in range(len(calibration_points) - 1):
        a0, p0 = calibration_points[i]
        a1, p1 = calibration_points[i + 1]
        if p0 <= position <= p1:
            fraction = (position - p0) / (p1 - p0)
            return a0 + fraction * (a1 - a0)
    return 20.0 if position > 650.0 else -20.0


def apply_j1_deadband(angle, active):
    """Remove small residual J1 neutral fluctuations without shifting target."""
    value = float(angle)
    magnitude = abs(value)
    if active:
        if magnitude <= J1_DEADBAND_EXIT_DEG:
            return 0.0, False
    else:
        if magnitude < J1_DEADBAND_ENTER_DEG:
            return 0.0, False
        active = True
    # Continuous output at the entry threshold avoids a position step.
    corrected = max(0.0, magnitude - J1_DEADBAND_ENTER_DEG)
    return float(np.copysign(corrected, value)), active


class PredictionSmoother:
    """Moving average followed by a first-order low-pass (EMA) on SVR outputs."""
    def __init__(self, moving_average_size=3, alpha=0.40):
        self.values = deque(maxlen=max(1, int(moving_average_size)))
        self.alpha = float(alpha)
        self.filtered = None

    def update(self, angle):
        self.values.append(float(angle))
        moving_average = float(np.mean(self.values))
        if self.filtered is None:
            self.filtered = moving_average
        else:
            self.filtered += self.alpha * (moving_average - self.filtered)
        return self.filtered, moving_average


def read_sample(line):
    parts = line.split(",")
    if len(parts) != 3:
        return None
    try:
        timestamp = int(float(parts[0]))
        h = float(parts[1])
        v = float(parts[2])
    except ValueError:
        return None
    if not (np.isfinite(h) and np.isfinite(v)):
        return None
    return timestamp, h, v


JOINT_NAMES = {1: "J1", 2: "J2", 3: "J3"}

def predict_angle_for_joint(horizontal, vertical, selected_joint):
    """
    Reuse the existing SVR model for the selected axis.
    J1: normal channel order (H features, V features).
    J2/J3: swap channel roles so vertical EOG occupies the model's
    horizontal-feature input positions.
    """
    if selected_joint == 1:
        X = extract_features(horizontal, vertical)
    else:
        X = extract_features(vertical, horizontal)
    return float(svr_model.predict(X)[0])


class DoubleBlinkDetector:
    """Count two distinct SVM BLINK-class events within a time window."""
    def __init__(self, group_window_s=1.2, refractory_s=0.35):
        self.group_window_s = group_window_s
        self.refractory_s = refractory_s
        self.count = 0
        self.first_time = None
        self.was_blink = False
        self.last_blink_time = -1e9

    def update(self, label, now=None):
        now = time.monotonic() if now is None else now
        is_blink = "BLINK" in str(label).upper()
        double = False

        # Count only the transition into BLINK, not consecutive overlapping windows.
        if is_blink and not self.was_blink and now - self.last_blink_time >= self.refractory_s:
            if self.count == 0 or self.first_time is None or now - self.first_time > self.group_window_s:
                self.count = 1
                self.first_time = now
            else:
                self.count += 1
                if self.count >= 2:
                    double = True
                    self.count = 0
                    self.first_time = None
            self.last_blink_time = now

        self.was_blink = is_blink
        if self.count and self.first_time is not None and now - self.first_time > self.group_window_s:
            self.count = 0
            self.first_time = None
        return double




# Four separately labelled target-fixation captures per direction. Each capture
# is a continuous EOG segment, not a single ADC sample. Keep the chart 45 cm away
# and move only the eyes while keeping the head still.
CALIBRATION_STEPS = [
    ("Step 1/20 — Neutral", "Sit 45 cm from the chart. Keep your head still, look at the centre target and relax.", 10.0, "neutral"),
]
for _direction, _label, _instruction in (
    ("left", "LEFT", "Look at the indicated LEFT target on the chart."),
    ("right", "RIGHT", "Look at the indicated RIGHT target on the chart."),
    ("up", "UP", "Look at the indicated UP target on the chart."),
    ("down", "DOWN", "Look at the indicated DOWN target on the chart."),
):
    for _sample_number in range(1, 5):
        _step_number = len(CALIBRATION_STEPS) + 1
        CALIBRATION_STEPS.append((
            f"Step {_step_number}/20 — {_label} sample {_sample_number}/4",
            f"{_instruction} Keep your head still, hold your gaze steadily, then press NEXT STEP after capture.",
            5.0,
            f"{_direction}_{_sample_number}",
        ))
CALIBRATION_STEPS.extend([
    ("Step 18/20 — Neutral confirmation", "Return your gaze to the centre target and relax.", 8.0, "center_final"),
    ("Step 19/20 — Single blinks", "Blink naturally one time, relax, then repeat several times. Do not squeeze your eyes tightly.", 12.0, "blink"),
    ("Step 20/20 — Double blinks", "Blink twice quickly, wait, and repeat several times.", 15.0, "double_blink"),
])


def speak_instruction(text):
    """Speak instructions using Windows built-in System.Speech without blocking."""
    try:
        safe = text.replace("'", "''")
        script = (
            "Add-Type -AssemblyName System.Speech; "
            "$s=New-Object System.Speech.Synthesis.SpeechSynthesizer; "
            f"$s.Speak('{safe}')"
        )
        subprocess.Popen(
            ["powershell", "-NoProfile", "-Command", script],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except Exception:
        pass


def run_startup_calibration(acq, status_queue, calibration_queue, stop_event):
    """Capture one phase at a time; Next is accepted only after minimum capture time."""
    hb, vb = deque(maxlen=WINDOW), deque(maxlen=WINDOW)
    hop_count = 0
    neutral_h, neutral_v = [], []
    phase_stats = {}

    for idx, (title, instruction, minimum, key) in enumerate(CALIBRATION_STEPS):
        if stop_event.is_set():
            raise RuntimeError("Calibration cancelled.")
        phase_h, phase_v, blink_labels = [], [], []
        started = time.monotonic()
        next_unlocked = False
        status_queue.put(("calibration_step", idx, len(CALIBRATION_STEPS), title, instruction, minimum))
        threading.Thread(target=speak_instruction, args=(title + ". " + instruction,), daemon=True).start()

        while not stop_event.is_set():
            elapsed = time.monotonic() - started
            if elapsed >= minimum and not next_unlocked:
                next_unlocked = True
                status_queue.put(("calibration_next_enabled", f"Minimum {minimum:.0f}s captured. Press NEXT STEP when ready."))

            sample = read_sample(acq.readline().decode("utf-8", errors="ignore").strip())
            if sample:
                _, h, v = sample
                hb.append(h); vb.append(v)
                phase_h.append(h); phase_v.append(v)
                hop_count += 1
                if len(hb) == WINDOW and hop_count >= HOP:
                    hop_count = 0
                    hwin, vwin = list(hb), list(vb)
                    Xh = extract_features(hwin, vwin)
                    Xv = extract_features(vwin, hwin)
                    if key == "neutral":
                        neutral_h.append(float(svr_model.predict(Xh)[0]))
                        neutral_v.append(float(svr_model.predict(Xv)[0]))
                    if key in ("blink", "double_blink"):
                        blink_labels.append(str(svm_model.predict(Xh)[0]))

            try:
                action = calibration_queue.get_nowait()
            except queue.Empty:
                action = None
            if action == "CANCEL":
                raise RuntimeError("Calibration cancelled by user.")
            if action == "NEXT" and next_unlocked:
                break

        if stop_event.is_set():
            raise RuntimeError("Calibration cancelled.")
        if not phase_h:
            raise RuntimeError(f"No EOG samples captured during {title}.")
        phase_stats[key] = {
            "horizontal_median_adc": float(np.median(phase_h)),
            "vertical_median_adc": float(np.median(phase_v)),
            "horizontal_range_adc": float(np.ptp(phase_h)),
            "vertical_range_adc": float(np.ptp(phase_v)),
            "samples": len(phase_h),
            "svm_labels": blink_labels,
            "blink_label_count": sum("BLINK" in x.upper() for x in blink_labels),
        }
        status_queue.put(("calibration_step_done", title))

    if not neutral_h or not neutral_v:
        raise RuntimeError("No neutral SVR predictions collected.")
    offsets = {
        "horizontal": float(np.median(neutral_h)),
        "vertical": float(np.median(neutral_v)),
    }
    profile = {
        "created_unix_time": time.time(),
        "neutral_svr_angle_offset_deg": offsets,
        "phase_measurements": phase_stats,
        "notes": "20-step guided calibration at 45 cm. Four separate continuous captures per direction (left_1..4, right_1..4, up_1..4, down_1..4). Captures diagnostic data; does not retrain models or alter trained feature scaling.",
        "chart_distance_cm": 45,
        "directional_samples_required": 4,
        "directional_sample_keys": {d: [f"{d}_{i}" for i in range(1, 5)] for d in ("left", "right", "up", "down")},
    }
    with open(os.path.join(BASE_DIR, "calibration_profile.json"), "w", encoding="utf-8") as f:
        json.dump(profile, f, indent=2)
    status_queue.put(("calibration_done", "Calibration complete. Press START ROBOT to enable movement."))
    return offsets


def run_controller(selected_joint_getter, selected_joint_state, selected_lock, status_queue, command_queue, calibration_queue, robot_state, stop_event):
    h_buffer = deque(maxlen=WINDOW)
    v_buffer = deque(maxlen=WINDOW)
    samples_after_prediction = 0
    sequence_id = 0
    # Remember which joint was actively controlled for each outgoing EOG command.
    pending_feedback_joint = {}
    smoother = PredictionSmoother(
        moving_average_size=MOVING_AVERAGE_PREDICTIONS,
        alpha=LOW_PASS_ALPHA,
    )
    last_sent_targets = [JOINT_HOME_POSITION, JOINT_HOME_POSITION, JOINT_HOME_POSITION]
    last_command_time = 0.0
    previous_joint = None
    # Must exist before any live prediction can be processed.
    j1_deadband_active = False
    blink_detector = DoubleBlinkDetector(group_window_s=1.2, refractory_s=0.35)

    status_queue.put(("status", f"Opening {ACQUISITION_PORT} and {ROBOT_PORT}..."))

    try:
        with serial.Serial(ACQUISITION_PORT, BAUD, timeout=0.05) as acq, \
             serial.Serial(ROBOT_PORT, BAUD, timeout=0.01, write_timeout=0.2) as robot:
            time.sleep(2)
            acq.reset_input_buffer()
            robot.reset_input_buffer()
            status_queue.put(("status", "Connected. Press START CALIBRATION."))
            angle_offsets = None

            while not stop_event.is_set():
                # Process GUI commands on the same thread that owns the serial port.
                # HOME sends all three joints to 500,500,500 immediately.
                try:
                    while True:
                        requested_action = command_queue.get_nowait()
                        if requested_action == "CALIBRATE":
                            robot_state["enabled"] = False
                            robot_state["calibrated"] = False
                            j1_deadband_active = False
                            angle_offsets = run_startup_calibration(acq, status_queue, calibration_queue, stop_event)
                            robot_state["calibrated"] = True
                            status_queue.put(("calibration_ready", "Calibration saved. Press START ROBOT."))
                        elif requested_action == "START":
                            if robot_state["calibrated"] and angle_offsets is not None:
                                robot_state["enabled"] = True
                                status_queue.put(("status", "ROBOT STARTED — EOG control enabled."))
                            else:
                                status_queue.put(("status", "Complete calibration before starting the robot."))
                        elif requested_action == "STOP":
                            robot_state["enabled"] = False
                            status_queue.put(("status", "ROBOT STOPPED — no new movement commands will be sent."))
                        elif requested_action == "HOME":
                            sequence_id = (sequence_id + 1) % 65536
                            home_command = f"SET,{sequence_id},500,500,500\n"
                            if ROBOT_ARMED:
                                robot.write(home_command.encode("ascii"))
                                robot.flush()
                                status_queue.put(("status", "HOME command sent: J1=500, J2=500, J3=500"))
                            else:
                                status_queue.put(("status", "DRY RUN: HOME target is J1=500, J2=500, J3=500"))
                            last_sent_targets = [500, 500, 500]
                            j1_deadband_active = False
                            last_command_time = time.monotonic()
                            smoother = PredictionSmoother(
                                moving_average_size=MOVING_AVERAGE_PREDICTIONS,
                                alpha=LOW_PASS_ALPHA,
                            )
                except queue.Empty:
                    pass

                try:
                    while robot.in_waiting:
                        reply = robot.readline().decode("utf-8", errors="ignore").strip()
                        if reply:
                            if reply.startswith("FEEDBACK,"):
                                fields = reply.split(",")
                                try:
                                    seq = fields[1]
                                    # Resolve the joint that generated this movement command.
                                    pending = pending_feedback_joint.pop(int(seq), None)
                                    if pending is None:
                                        active_joint = selected_joint_getter()
                                        predicted_eye_angle = None
                                        commanded_eye_angle = None
                                    else:
                                        active_joint, predicted_eye_angle, commanded_eye_angle = pending
                                    joint_fields = {
                                        1: ("J1", 3, 4),
                                        2: ("J2", 6, 7),
                                        3: ("J3", 9, 10),
                                    }
                                    joint_name, ok_idx, pos_idx = joint_fields[active_joint]
                                    if fields[ok_idx] == "1":
                                        raw_position = int(fields[pos_idx])
                                        received_eye_angle = map_position_to_eye_angle(raw_position)
                                        if commanded_eye_angle is not None:
                                            error_angle = commanded_eye_angle - received_eye_angle
                                            predicted_text = (
                                                f"SVR={predicted_eye_angle:+.2f}°  |  "
                                                if predicted_eye_angle is not None else ""
                                            )
                                            message = (
                                                f"ANGLE TRACKING  |  ACTIVE {joint_name}  |  "
                                                f"{predicted_text}"
                                                f"SENT={commanded_eye_angle:+.2f}°  |  "
                                                f"RECEIVED={received_eye_angle:+.2f}°  |  "
                                                f"ERROR={error_angle:+.2f}°  |  "
                                                f"SERVO={raw_position}/1000  |  Seq {seq}"
                                            )
                                        else:
                                            message = (
                                                f"ANGLE TRACKING  |  ACTIVE {joint_name}  |  "
                                                f"RECEIVED={received_eye_angle:+.2f}°  |  "
                                                f"SERVO={raw_position}/1000  |  "
                                                f"NO MATCHED SENT COMMAND  |  Seq {seq}"
                                            )
                                    else:
                                        message = f"ANGLE TRACKING  |  ACTIVE {joint_name}  |  NO SERVO RESPONSE  |  Seq {seq}"
                                    # Keep actual Shrike feedback in the diagnostic line above.
                                    # The validation panel is populated from the selected joint
                                    # target shown in the live TX line (J1=..., J2=..., J3=...).
                                    status_queue.put(("feedback", message))
                                except (IndexError, ValueError):
                                    status_queue.put(("log", f"[SHRIKE] Invalid feedback frame: {reply}"))
                            else:
                                status_queue.put(("log", f"[SHRIKE] {reply}"))
                except serial.SerialException:
                    raise

                line = acq.readline().decode("utf-8", errors="ignore").strip()
                sample = read_sample(line)
                if sample is None:
                    continue

                timestamp_us, h_value, v_value = sample
                h_buffer.append(h_value)
                v_buffer.append(v_value)
                samples_after_prediction += 1

                if len(h_buffer) < WINDOW:
                    continue
                if samples_after_prediction < HOP:
                    continue
                samples_after_prediction = 0

                if not robot_state["enabled"] or not robot_state["calibrated"] or angle_offsets is None:
                    continue

                selected_joint = selected_joint_getter()
                if selected_joint != previous_joint:
                    # Avoid carrying the previous axis' smoothing history into
                    # the newly selected joint.
                    smoother = PredictionSmoother(
                        moving_average_size=MOVING_AVERAGE_PREDICTIONS,
                        alpha=LOW_PASS_ALPHA,
                    )
                    previous_joint = selected_joint
                    j1_deadband_active = False

                horizontal_window = list(h_buffer)
                vertical_window = list(v_buffer)

                # Preserve the existing axis selection and SVR control.
                if selected_joint == 1:
                    X = extract_features(horizontal_window, vertical_window)
                else:
                    X = extract_features(vertical_window, horizontal_window)

                # Additive feature: use the existing SVM movement classifier
                # to detect two separate BLINK-labelled events.
                svm_label = svm_model.predict(X)[0]
                if blink_detector.update(svm_label):
                    with selected_lock:
                        selected_joint_state["value"] = (selected_joint_state["value"] % 3) + 1
                        new_joint = selected_joint_state["value"]
                    selected_joint = new_joint
                    j1_deadband_active = False
                    smoother = PredictionSmoother(
                        moving_average_size=MOVING_AVERAGE_PREDICTIONS,
                        alpha=LOW_PASS_ALPHA,
                    )
                    status_queue.put(("joint", f"Double blink detected. Switched to {JOINT_NAMES[new_joint]}."))

                raw_angle_unadjusted = float(svr_model.predict(X)[0])
                axis_offset = angle_offsets["horizontal"] if selected_joint == 1 else angle_offsets["vertical"]
                raw_angle = raw_angle_unadjusted - axis_offset
                filtered_angle, moving_average_angle = smoother.update(raw_angle)
                control_angle = filtered_angle
                if selected_joint == 1:
                    control_angle, j1_deadband_active = apply_j1_deadband(
                        filtered_angle, j1_deadband_active
                    )
                else:
                    j1_deadband_active = False
                gained_angle = control_angle * SVR_ANGLE_GAIN
                requested_target, bounded_angle = map_angle_to_position(gained_angle)

                joint_index = selected_joint - 1
                last_target = last_sent_targets[joint_index]
                delta = requested_target - last_target
                target = requested_target
                if abs(delta) > MAX_POSITION_STEP_PER_UPDATE:
                    target = last_target + (
                        MAX_POSITION_STEP_PER_UPDATE if delta > 0
                        else -MAX_POSITION_STEP_PER_UPDATE
                    )

                now = time.monotonic()
                changed_enough = abs(target - last_target) >= MIN_POSITION_CHANGE
                interval_ok = (now - last_command_time) >= MIN_COMMAND_INTERVAL_S
                status = "HOLD"

                if changed_enough and interval_ok:
                    positions = list(last_sent_targets)
                    positions[joint_index] = int(target)
                    sequence_id = (sequence_id + 1) % 65536
                    command = (
                        f"SET,{sequence_id},{positions[0]},"
                        f"{positions[1]},{positions[2]}\n"
                    )
                    if ROBOT_ARMED:
                        command_eye_angle = map_position_to_eye_angle(target)
                        pending_feedback_joint[sequence_id] = (
                            selected_joint,
                            float(gained_angle),
                            float(command_eye_angle),
                        )
                        # Bound the lookup table in case a servo never replies.
                        if len(pending_feedback_joint) > 64:
                            oldest_seq = next(iter(pending_feedback_joint))
                            pending_feedback_joint.pop(oldest_seq, None)
                        robot.write(command.encode("ascii"))
                        robot.flush()
                        status = "TX"
                    else:
                        status = "DRY_RUN"

                    last_sent_targets = positions
                    last_command_time = now

                status_queue.put((
                    "reading",
                    f"{status} | Selected {JOINT_NAMES[selected_joint]} | "
                    f"raw={raw_angle:+.2f}° | filtered={filtered_angle:+.2f}° | "
                    f"control={control_angle:+.2f}° | SVR={gained_angle:+.2f}° | target={target} | "
                    f"J1={last_sent_targets[0]} J2={last_sent_targets[1]} "
                    f"J3={last_sent_targets[2]} | H={h_value:.0f} V={v_value:.0f}"
                ))

                # Validation panel follows the selected joint count printed in the
                # TX/status line (J1/J2/J3), not the asynchronous FEEDBACK frame.
                selected_joint_position = int(last_sent_targets[joint_index])
                returned_equivalent_angle = map_position_to_eye_angle(selected_joint_position)
                commanded_equivalent_angle = returned_equivalent_angle
                tracking_error = float(gained_angle) - returned_equivalent_angle
                status_queue.put((
                    "validation",
                    JOINT_NAMES[selected_joint],
                    float(gained_angle),
                    commanded_equivalent_angle,
                    selected_joint_position,
                    returned_equivalent_angle,
                    tracking_error,
                ))

    except Exception as exc:
        status_queue.put(("error", f"{type(exc).__name__}: {exc}"))


def main():
    status_queue = queue.Queue()
    command_queue = queue.Queue()
    calibration_queue = queue.Queue()
    stop_event = threading.Event()
    robot_state = {"calibrated": False, "enabled": False}
    selected_joint = {"value": 1}
    selected_lock = threading.Lock()

    def get_selected_joint():
        with selected_lock:
            return selected_joint["value"]

    def cycle_joint():
        with selected_lock:
            selected_joint["value"] = (selected_joint["value"] % 3) + 1
            current = selected_joint["value"]
        joint_button.config(text=f"Control {JOINT_NAMES[current]}  •  Switch Joint")
        axis = "Horizontal EOG" if current == 1 else "Vertical EOG"
        selection_label.config(text=f"Active: {JOINT_NAMES[current]}  |  Input: {axis}")
        status_label.config(text=f"Switched to {JOINT_NAMES[current]}.")

    def start_calibration():
        command_queue.put("CALIBRATE")
        calibration_start_button.config(state="disabled")
        next_button.config(state="disabled")
        start_robot_button.config(state="disabled")
        instruction_label.config(text="Calibration starting. Follow the spoken instructions.")
        status_label.config(text="Calibration in progress...")

    def next_calibration_step():
        calibration_queue.put("NEXT")
        next_button.config(state="disabled")
        status_label.config(text="Advancing to the next calibration step...")

    def start_robot():
        command_queue.put("START")
        status_label.config(text="Starting robot control...")

    def stop_robot():
        command_queue.put("STOP")
        status_label.config(text="Stopping new robot movement commands...")

    def go_home():
        command_queue.put("HOME")
        status_label.config(text="Home requested: moving all joints to 500, 500, 500...")

    def on_close():
        stop_event.set()
        root.after(100, root.destroy)

    root = tk.Tk()
    root.title("EOG-Based Robotic Manipulator Control")
    root.geometry("980x820")
    root.minsize(760, 700)

    outer = tk.Frame(root, padx=20, pady=18)
    outer.pack(fill="both", expand=True)

    tk.Label(
        outer, text="EOG ROBOT JOINT CONTROLLER",
        font=("Segoe UI", 17, "bold")
    ).pack(anchor="w")

    tk.Label(
        outer, text="Follow each spoken instruction, wait for capture, then press NEXT STEP.",
        font=("Segoe UI", 10)
    ).pack(anchor="w", pady=(4, 10))

    calibration_start_button = tk.Button(
        outer, text="START CALIBRATION", command=start_calibration,
        font=("Segoe UI", 11, "bold"), padx=14, pady=9
    )
    calibration_start_button.pack(fill="x")

    instruction_label = tk.Label(
        outer, text="Calibration has not started.",
        anchor="w", justify="left", wraplength=840,
        font=("Segoe UI", 11, "bold")
    )
    instruction_label.pack(fill="x", pady=(10, 4))

    calibration_progress_label = tk.Label(
        outer, text="Step 0 of 9", anchor="w", font=("Segoe UI", 9)
    )
    calibration_progress_label.pack(fill="x")

    next_button = tk.Button(
        outer, text="NEXT STEP", command=next_calibration_step,
        font=("Segoe UI", 10, "bold"), padx=12, pady=8, state="disabled"
    )
    next_button.pack(fill="x", pady=(6, 8))

    robot_controls = tk.Frame(outer)
    robot_controls.pack(fill="x", pady=(0, 8))
    start_robot_button = tk.Button(
        robot_controls, text="START ROBOT", command=start_robot,
        font=("Segoe UI", 10, "bold"), padx=10, pady=8, state="disabled"
    )
    start_robot_button.pack(side="left", expand=True, fill="x", padx=(0, 4))
    stop_robot_button = tk.Button(
        robot_controls, text="STOP ROBOT", command=stop_robot,
        font=("Segoe UI", 10, "bold"), padx=10, pady=8
    )
    stop_robot_button.pack(side="left", expand=True, fill="x", padx=(4, 0))

    joint_button = tk.Button(
        outer,
        text="Control J1  •  Switch Joint",
        command=cycle_joint,
        font=("Segoe UI", 13, "bold"),
        padx=18, pady=12
    )
    joint_button.pack(fill="x")

    home_button = tk.Button(
        outer,
        text="GO TO HOME POSITION  (500, 500, 500)",
        command=go_home,
        font=("Segoe UI", 11, "bold"),
        padx=16, pady=10
    )
    home_button.pack(fill="x", pady=(8, 0))

    selection_label = tk.Label(
        outer, text="Active: J1  |  Input: Horizontal EOG",
        font=("Segoe UI", 12, "bold")
    )
    selection_label.pack(anchor="w", pady=(14, 8))

    mapping_label = tk.Label(
        outer,
        text=("Angle mapping for selected joint: -20°→300, -15°→350, "
              "-10°→400, -5°→450, 0°→500, +5°→550, "
              "+10°→600, +15°→650, +20°→800"),
        justify="left", wraplength=700, font=("Segoe UI", 9)
    )
    mapping_label.pack(anchor="w", pady=(0, 12))

    status_label = tk.Label(
        outer, text="Starting controller...",
        anchor="w", justify="left", font=("Segoe UI", 10)
    )
    status_label.pack(fill="x", pady=(4, 6))

    reading_label = tk.Label(
        outer, text="Waiting for predictions...",
        anchor="w", justify="left", wraplength=700,
        font=("Consolas", 9)
    )
    reading_label.pack(fill="x", pady=4)

    feedback_label = tk.Label(
        outer, text="Robot actual position: waiting for servo feedback...",
        anchor="w", justify="left", wraplength=700,
        font=("Consolas", 10, "bold")
    )
    feedback_label.pack(fill="x", pady=(4, 2))

    # Persistent validation panel: present from application launch, then populated
    # whenever a matching servo FEEDBACK frame is received.
    validation_frame = tk.LabelFrame(
        outer, text="ROBOT ANGLE VALIDATION", padx=12, pady=8,
        font=("Segoe UI", 11, "bold"),
    )
    validation_frame.pack(fill="x", pady=(10, 4))
    validation_values = {}
    validation_rows = [
        ("ACTIVE JOINT", "active_joint", "J1"),
        ("SVR PREDICTED EOG ANGLE", "predicted", "Waiting for prediction..."),
        ("COMMANDED EOG ANGLE", "commanded", "Waiting for command..."),
        ("RETURNED SERVO POSITION", "servo_position", "Waiting for servo feedback..."),
        ("RETURNED EOG ANGLE", "returned", "Waiting for servo feedback..."),
        ("TRACKING ERROR", "error", "Waiting for servo feedback..."),
    ]
    for row_index, (caption, key, initial) in enumerate(validation_rows):
        tk.Label(
            validation_frame, text=caption, anchor="w",
            font=("Consolas", 10, "bold"), width=29,
        ).grid(row=row_index, column=0, sticky="w", padx=(0, 12), pady=3)
        value_label = tk.Label(
            validation_frame, text=initial, anchor="w",
            font=("Consolas", 10),
        )
        value_label.grid(row=row_index, column=1, sticky="w", pady=3)
        validation_values[key] = value_label

    safety_label = tk.Label(
        outer,
        text=("Voice and on-screen instructions guide calibration. START ROBOT enables "
              "EOG motion only after calibration. STOP ROBOT stops new commands; "
              "servos hold their last commanded positions."),
        anchor="w", justify="left", wraplength=700,
        font=("Segoe UI", 9)
    )
    safety_label.pack(fill="x", pady=(12, 0))

    worker = threading.Thread(
        target=run_controller,
        args=(get_selected_joint, selected_joint, selected_lock, status_queue, command_queue, calibration_queue, robot_state, stop_event),
        daemon=True
    )
    worker.start()

    def poll_status():
        try:
            while True:
                item = status_queue.get_nowait()
                kind = item[0]
                message = item[1] if len(item) > 1 else ""
                if kind in ("status", "error", "calibration", "calibration_done"):
                    status_label.config(text=message)
                elif kind == "calibration_step":
                    _, idx, total, title, instruction, minimum = item
                    instruction_label.config(text=instruction)
                    calibration_progress_label.config(text=f"{title} — minimum capture {minimum:.0f} seconds")
                    next_button.config(state="disabled")
                    status_label.config(text="Follow the instruction. NEXT STEP unlocks after the minimum capture time.")
                elif kind == "calibration_next_enabled":
                    next_button.config(state="normal")
                    status_label.config(text=message)
                elif kind == "calibration_step_done":
                    pass
                elif kind == "calibration_ready":
                    status_label.config(text=message)
                    calibration_start_button.config(state="normal")
                    next_button.config(state="disabled")
                    start_robot_button.config(state="normal")
                elif kind == "reading":
                    reading_label.config(text=message)
                elif kind == "feedback":
                    feedback_label.config(text=message)
                elif kind == "validation":
                    _, joint_name, predicted, commanded, servo_position, returned, error = item
                    validation_values["active_joint"].config(text=joint_name)
                    validation_values["predicted"].config(
                        text=f"{predicted:+.2f}°" if predicted is not None else "Not available"
                    )
                    validation_values["commanded"].config(
                        text=f"{commanded:+.2f}°" if commanded is not None else "Not available"
                    )
                    validation_values["servo_position"].config(
                        text=f"{servo_position} / 1000" if servo_position is not None else "No response"
                    )
                    validation_values["returned"].config(
                        text=f"{returned:+.2f}°" if returned is not None else "No response"
                    )
                    validation_values["error"].config(
                        text=f"{error:+.2f}°" if error is not None else "Not available"
                    )
                elif kind == "joint":
                    current = get_selected_joint()
                    joint_button.config(text=f"Control {JOINT_NAMES[current]}  •  Switch Joint")
                    axis = "Horizontal EOG" if current == 1 else "Vertical EOG"
                    selection_label.config(text=f"Active: {JOINT_NAMES[current]}  |  Input: {axis}")
                    status_label.config(text=message)
                elif kind == "log":
                    status_label.config(text=message)
        except queue.Empty:
            pass
        if root.winfo_exists():
            root.after(100, poll_status)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, poll_status)
    root.mainloop()


if __name__ == "__main__":
    main()
