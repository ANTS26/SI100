# pid.py (macOS-safe: no GUI backend issues)

import os
import time

import mujoco
import mujoco.viewer
import numpy as np

# --- IMPORTANT: set non-interactive backend BEFORE importing pyplot ---
import matplotlib
matplotlib.use("Agg")  # prevents MacOSX GUI FigureManager errors in non-main thread contexts
import matplotlib.pyplot as plt


# -------------------------
# Model path
# -------------------------
xml_path = "../../models/Pendulum/pendulum.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.normpath(os.path.join(dirname, xml_path))


# -------------------------
# PID controller
# -------------------------
def pid(kp, ki, kd, state, dt):
    """
    Args:
        kp, ki, kd: Controller gains
        state: dict with 'error', 'integral', 'last_error'
        dt: time step
    Returns:
        output torque
    """
    current_error = state["error"]

    # I term
    state["integral"] += current_error * dt

    # D term
    derivative = (current_error - state["last_error"]) / dt if dt > 0 else 0.0

    # PID output
    output = (kp * current_error) + (ki * state["integral"]) + (kd * derivative)

    # update
    state["last_error"] = current_error
    return output


# -------------------------
# Build MuJoCo model/data
# -------------------------
model = mujoco.MjModel.from_xml_path(abspath)
data = mujoco.MjData(model)

# -------------------------
# Control parameters
# -------------------------
KP = 0.0
KI = 0.0
KD = 0.0
TARGET_ANGLE = np.pi
DT = model.opt.timestep

control_state = {"error": 0.0, "integral": 0.0, "last_error": 0.0}

# -------------------------
# Data storage
# -------------------------
time_history = []
error_history = []

print(f"Loading model from: {abspath}")

# -------------------------
# Simulation with passive viewer
# -------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_sim_time = data.time

    while viewer.is_running() and (data.time - start_sim_time) < 10.0:
        step_start = time.time()

        # error = target - current
        control_state["error"] = TARGET_ANGLE - data.qpos[0]

        # PID torque
        torque = pid(KP, KI, KD, control_state, DT)

        # apply torque
        data.ctrl[0] = torque

        # log
        time_history.append(float(data.time))
        error_history.append(float(control_state["error"]))

        # step physics
        mujoco.mj_step(model, data)

        # sync viewer
        viewer.sync()

        # real-time pacing
        elapsed = time.time() - step_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

# -------------------------
# Plot (save to file; macOS-safe)
# -------------------------
plt.figure(figsize=(10, 6))
plt.plot(time_history, error_history, label="Error (target - current)")
plt.axhline(y=0, linestyle="--", alpha=0.5)
plt.title("PID Control Performance: Error Convergence")
plt.xlabel("Time (s)")
plt.ylabel("Error (rad)")
plt.grid(True, linestyle=":", alpha=0.6)
plt.legend()

out_path = os.path.join(dirname, "pid_error.png")
plt.savefig(out_path, dpi=200, bbox_inches="tight")
print(f"Saved plot to: {out_path}")

# If you REALLY want to try showing (may still fail under some mjpython contexts),
# you can uncomment below. Otherwise keep it saved-only.
# plt.show()
