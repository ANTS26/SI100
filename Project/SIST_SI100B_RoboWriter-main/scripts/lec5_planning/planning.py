###
# Planning practice with MuJoCo
# SI100B Robotics Programming
# This code is modified based on the MuJoCo template code at https://github.com/pab47/pab47.github.io/tree/master.
# Date: Dec., 2025
###

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import scipy as sp
import matplotlib.pyplot as plt

xml_path = '../../models/universal_robots_ur5e/scene.xml' #xml file (assumes this is in the same folder as this file)
simend = 180 #simulation time (second)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Helper function
def IK_controller(model, data, X_ref, q_pos):
    # Compute Jacobian
    position_Q = data.site_xpos[0]

    jacp = np.zeros((3, 6))
    jacr = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, jacr, position_Q, 7)

    J = np.vstack((jacp, jacr))
    Jinv = np.linalg.pinv(J)

    # Reference point
    X = position_Q.copy()
    dX = X_ref - X

    # Reference orientation
    x_ref = np.array([1, 0, 0])
    y_ref = np.array([0, -1, 0])
    z_ref = np.array([0, 0, -1])

    R_curr = data.site_xmat[0].reshape(3, 3)
    x_curr = R_curr[:, 0]
    y_curr = R_curr[:, 1]
    z_curr = R_curr[:, 2]

    w_err = 0.5 * (np.cross(x_curr, x_ref) + np.cross(y_curr, y_ref) + np.cross(z_curr, z_ref))

    # Compute control input
    dq = Jinv @ np.concatenate((dX, w_err))

    return q_pos + dq

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

# Get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1920, 1080, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
cam.azimuth =  89.8300000000001 #摄像机水平旋转角度
cam.elevation =  -87.16333333333334 #摄像机垂直旋转角度
cam.distance =  1.66 #摄像机距离模型的距离


# Initialize the controller
init_controller(model,data)

# Set the controller
mj.set_mjcb_control(controller)

# Initialize joint configuration
init_qpos = np.array([-1.6353559, -1.28588984, 2.14838487, -2.61087434, -1.5903009, -0.06818645])
data.qpos[:] = init_qpos
cur_q_pos = init_qpos.copy()


MAX_TRAJ = 3000################################################################################################
traj_points = np.zeros((MAX_TRAJ, 3))
traj_cursor = 0
traj_count = 0
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])

# Joint state logging (only when writing)
LOG_WHEN_WRITING_ONLY = True
WRITE_Z_THRESHOLD = 0.1
joint_log_time = []
joint_log_qpos = []
joint_log_qvel = []

def SaveJointStatePlots(output_dir, t_arr, qpos_arr, qvel_arr=None):
    try:
        import importlib
        plt = importlib.import_module("matplotlib.pyplot")
    except Exception as e:
        print("matplotlib not available; saving joint logs as CSV instead.")
        print(f"Import error: {e}")
        csv_path = os.path.join(output_dir, "joint_states.csv")
        header_cols = ["time"] + [f"q{i+1}" for i in range(qpos_arr.shape[1])]
        data_mat = np.column_stack([t_arr, qpos_arr])
        np.savetxt(csv_path, data_mat, delimiter=",", header=",".join(header_cols), comments="")
        print(f"Saved: {csv_path}")
        return

    # Plot joint positions
    fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    for j in range(min(6, qpos_arr.shape[1])):
        axes[j].plot(t_arr, qpos_arr[:, j], linewidth=1.0)
        axes[j].set_ylabel(f"q{j+1} (rad)")
        axes[j].grid(True, alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Joint Positions During Writing")
    fig.tight_layout()
    out_path = os.path.join(output_dir, "joint_states_qpos.png")
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Saved: {out_path}")

    # Optional: plot joint velocities
    if qvel_arr is not None and qvel_arr.size > 0:
        fig2, axes2 = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
        for j in range(min(6, qvel_arr.shape[1])):
            axes2[j].plot(t_arr, qvel_arr[:, j], linewidth=1.0)
            axes2[j].set_ylabel(f"dq{j+1} (rad/s)")
            axes2[j].grid(True, alpha=0.3)
        axes2[-1].set_xlabel("time (s)")
        fig2.suptitle("Joint Velocities During Writing")
        fig2.tight_layout()
        out_path2 = os.path.join(output_dir, "joint_states_qvel.png")
        fig2.savefig(out_path2, dpi=200)
        plt.close(fig2)
        print(f"Saved: {out_path2}")

# Load trajectory from file
traj_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectory_output.txt")
if os.path.exists(traj_file):
    print(f"Loading trajectory from {traj_file}")
    with open(traj_file, "r") as f:
        exec(f.read())
else:
    print(f"Warning: {traj_file} not found. Using default trajectory.")
    qn = [np.array([0.5, 0.1, 0.1])]


qn_filtered = [qn[0]]
for i in range(1, len(qn)):
    dist = np.linalg.norm(qn[i] - qn_filtered[-1])
    if dist > 1e-4:  # 只有距离大于 0.1mm 才保留
        qn_filtered.append(qn[i])
qn = qn_filtered
# 插值处理：如果相邻点距离过大，插入中间点 #######用来避免前几笔画不上 
qn_interp = [qn[0]]
MAX_DIST = 0.05 # 不写字时候的插值间距
for i in range(1, len(qn)):
    dist = np.linalg.norm(qn[i] - qn[i-1])
    if dist > MAX_DIST:
        steps = int(dist / MAX_DIST)
        for k in range(1, steps + 1):
            qn_interp.append(qn[i-1] + (qn[i] - qn[i-1]) * k / (steps + 1))
    qn_interp.append(qn[i])
qn = qn_interp

#t_total = simend
t_total = 30.0
# Use quadratic interpolation (Quadratic Bezier) per segment.
# Each segment uses 3 consecutive points: (q0, q1, q2).
# Therefore, the number of segments is len(qn) - 2.
if len(qn) >= 3:
    n_segments = len(qn) - 2
else:
    n_segments = max(len(qn) - 1, 1)

seg_dur = float(t_total) / float(n_segments)
######################################
### BAISIC INTERPOLATION FUNCTIONS ###
def LinearInterpolate(q0, q1, t, t_total):
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)

    if t_total <= 0:
        return q1.copy()

    s = float(t) / float(t_total)
    s = float(np.clip(s, 0.0, 1.0))
    return (1.0 - s) * q0 + s * q1
######################################

############################################
### BONUS: BEZIER INTERPOLATION FUNCTION ###
def QuadBezierInterpolate(q0, q1, q2, t, t_total):
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)

    if t_total <= 0:
        return q2.copy()

    s = float(t) / float(t_total)
    s = float(np.clip(s, 0.0, 1.0))

    one_minus = 1.0 - s
    return (one_minus * one_minus) * q0 + (2.0 * one_minus * s) * q1 + (s * s) * q2
############################################

def IsCollinear3Pts(p0, p1, p2, *, atol=1e-9, rtol=1e-6):
    """Return True if 3D points p0, p1, p2 are collinear (within tolerance)."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)

    v1 = p1 - p0
    v2 = p2 - p0
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))

    # If points are (nearly) identical, treat as collinear to avoid numerical issues.
    if n1 <= atol or n2 <= atol:
        return True

    cross_norm = float(np.linalg.norm(np.cross(v1, v2)))
    return cross_norm <= max(atol, rtol * n1 * n2)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # Store trajectory
        mj_end_eff_pos = data.site_xpos[0]
        #print(mj_end_eff_pos)
        is_writing = (mj_end_eff_pos[2] < WRITE_Z_THRESHOLD)
        if is_writing:
            # Check distance before adding
            should_add = False
            if traj_count == 0:
                should_add = True
            else:
                last_idx = (traj_cursor - 1 + MAX_TRAJ) % MAX_TRAJ
                last_pos = traj_points[last_idx]
                if np.linalg.norm(mj_end_eff_pos - last_pos) >= 0.003:
                    should_add = True

            if should_add:
                traj_points[traj_cursor] = mj_end_eff_pos
                traj_cursor = (traj_cursor + 1) % MAX_TRAJ
                if traj_count < MAX_TRAJ:
                    traj_count += 1

        # Log joint states (during writing by default)
        if (not LOG_WHEN_WRITING_ONLY) or is_writing:
            joint_log_time.append(float(data.time))
            joint_log_qpos.append(data.qpos.copy())
            joint_log_qvel.append(data.qvel.copy())
            
        # Get current joint configuration
        cur_q_pos = data.qpos.copy()
        
        # Compute reference position across points, evenly split total time
        t_sim = min(max(data.time, 0.0), t_total)
        seg_idx = int(t_sim // seg_dur)
        if seg_idx >= n_segments:
            seg_idx = n_segments - 1
        t_local = t_sim - seg_idx * seg_dur
        # Quadratic interpolation (Bezier) using 3 consecutive points.
        # If 3 points are collinear, fall back to linear interpolation.
        if len(qn) >= 3:
            q0 = qn[seg_idx]
            q1 = qn[seg_idx + 1]
            q2 = qn[seg_idx + 2]
            if IsCollinear3Pts(q0, q1, q2):
                X_ref = LinearInterpolate(q0, q2, t_local, seg_dur)
            else:
                X_ref = QuadBezierInterpolate(q0, q1, q2, t_local, seg_dur)
        else:
            # Fallback: not enough points for quadratic interpolation.
            X_ref = LinearInterpolate(qn[0], qn[-1], t_local, seg_dur)

        # Compute control input using IK
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos)
        
        # Apply control input
        data.ctrl[:] = cur_ctrl
        mj.mj_step(model, data)
        #data.time += 0.01
######
    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    # Add trajectory as spheres
    start_idx = (traj_cursor - traj_count + MAX_TRAJ) % MAX_TRAJ
    sphere_count = 0
    
    for j in range(0, traj_count):
        idx = (start_idx + j) % MAX_TRAJ
        pos = traj_points[idx]
        
        if scene.ngeom >= scene.maxgeom:
            break  # avoid overflow

        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        sphere_count += 1
        #print(scene.ngeom)
        
        # Configure this geom as a sphere
        geom.type = mj.mjtGeom.mjGEOM_SPHERE
        geom.rgba[:] = LINE_RGBA
        geom.size[:] = np.array([0.002, 0.002, 0.002])
        geom.pos[:] = pos
        geom.mat[:] = np.eye(3)  # no rotation
        geom.dataid = -1
        geom.segid = -1
        geom.objtype = 0
        geom.objid = 0
        
    print(f"Sphere count: {sphere_count}")
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()

# Export joint state plots after simulation
if len(joint_log_time) > 1:
    out_dir = os.path.dirname(os.path.abspath(__file__))
    t_arr = np.asarray(joint_log_time, dtype=float)
    qpos_arr = np.asarray(joint_log_qpos, dtype=float)
    qvel_arr = np.asarray(joint_log_qvel, dtype=float)
    SaveJointStatePlots(out_dir, t_arr, qpos_arr, qvel_arr)
else:
    print("No joint states logged (writing condition may never be met).")
