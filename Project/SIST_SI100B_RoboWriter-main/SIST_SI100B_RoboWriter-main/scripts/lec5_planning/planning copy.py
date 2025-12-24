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

xml_path = '../../models/universal_robots_ur5e/scene.xml' #xml file (assumes this is in the same folder as this file)
simend = 10 #simulation time (second)
print_camera_config = 1 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Helper function
def IK_controller(model, data, X_ref, q_pos):
    # IK tuning (stability)
    # Larger IK_ALPHA -> faster, but can oscillate; smaller -> smoother.
    IK_ALPHA = 0.1
    # Damped least sIK_ALs (helps near singularities)
    IK_DAMPING = 1e-2
    # Max joint update per simulation step (rad)
    IK_DQ_MAX = 0.05

    # Compute Jacobian
    position_Q = data.site_xpos[0]

    jacp = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, None, position_Q, 7)

    J = jacp.copy()

    # Reference point
    X = position_Q.copy()
    dX = X_ref - X

    # Compute control input (DLS): (J^T J + λ^2 I) dq = J^T dX
    JT = J.T
    A = JT @ J + (IK_DAMPING ** 2) * np.eye(J.shape[1])
    b = JT @ dX
    dq = np.linalg.solve(A, b)

    # Step size and clamp
    dq = IK_ALPHA * dq
    dq = np.clip(dq, -IK_DQ_MAX, IK_DQ_MAX)

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
cam.azimuth =  179.8300000000001 
cam.elevation =  87.16333333333334
cam.distance =  2.22
cam.lookat = np.array([ 0.29723477517870245 , 0.28277006411151073 , 0.6082647377843177 ])

# Initialize the controller
init_controller(model,data)

# Set the controller
mj.set_mjcb_control(controller)

# Initialize joint configuration
init_qpos = np.array([-1.6353559, -1.28588984, 2.14838487, -2.61087434, -1.5903009, -0.06818645])
data.qpos[:] = init_qpos
cur_q_pos = init_qpos.copy()

traj_points = []
# Keep enough points to show the whole trajectory.
# We record one point per physics step, so needed length is roughly simend / dt.
_dt = float(model.opt.timestep) if hasattr(model, "opt") else 1.0 / 60.0
MAX_TRAJ = int(simend / _dt) + 10
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])




##############调参区域#####################

q0 = np.array([0, 0, 0])
q1 = np.array([0, 0.3, 0])
q2 = np.array([0.3, 0.3, 0])
q3= np.array([0.3, 0, 0])
t_total = simend

# Waypoints in task space (end-effector position targets).
# You can add as many points as you want; the fitted curve will be generated automatically.
waypoints = [q0, q1, q2, q3]

# Spline fitting hyperparameters (SciPy splprep)
# Larger s -> smoother but less faithful to the waypoints.
SPLINE_SMOOTHING = 1e-4

######################################
# 
# 
# 
# 
# 
# ### BAISIC INTERPOLATION FUNCTIONS ###
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

###################################################
### FITTED SPLINE (APPROXIMATE / "拟合") CURVE ###
def FittedSplineInterpolate(waypoints, t, t_total, smoothing=SPLINE_SMOOTHING):
    """Parametric B-spline fit in task space.

    - Input: waypoints (N x 3)
    - Output: X_ref (3,)

    This is a *fit* (approximation), not a strict interpolation.
    """
    pts = np.asarray(waypoints, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("waypoints must be an array-like of shape (N, 3)")
    if pts.shape[0] == 0:
        raise ValueError("waypoints must contain at least 1 point")
    if pts.shape[0] == 1 or t_total <= 0:
        return pts[-1].copy()

    # Cache spline parameters to avoid recomputing every simulation step.
    cache_key = (pts.shape[0], float(smoothing), pts.tobytes())
    if getattr(FittedSplineInterpolate, "_cache_key", None) != cache_key:
        # splprep expects data as a list/array of dimensions: (dim, N)
        data_for_fit = pts.T
        k = min(3, pts.shape[0] - 1)
        tck, _u = sp.interpolate.splprep(data_for_fit, s=smoothing, k=k)
        FittedSplineInterpolate._cache_key = cache_key
        FittedSplineInterpolate._tck = tck

    u = float(t) / float(t_total)
    u = float(np.clip(u, 0.0, 1.0))
    x, y, z = sp.interpolate.splev(u, FittedSplineInterpolate._tck)
    return np.array([x, y, z], dtype=float)
###################################################

###################################################
### OFFLINE JOINT TRAJECTORY PLANNING (REPLAN q) ###
def SolveIK_DLS(model, data, X_target, q_init, *,
                max_iters=200, tol=1e-4, damping=1e-2, alpha=0.5, dq_max=0.1):
    """Iterative IK in task space (position only) using DLS.

    Returns (q_sol, ok).
    """
    q = np.asarray(q_init, dtype=float).copy()
    X_target = np.asarray(X_target, dtype=float)

    for _ in range(int(max_iters)):
        data.qpos[:] = q
        mj.mj_forward(model, data)

        X = data.site_xpos[0].copy()
        dX = X_target - X
        if float(np.linalg.norm(dX)) < float(tol):
            return q, True

        jacp = np.zeros((3, 6))
        mj.mj_jac(model, data, jacp, None, X, 7)
        J = jacp

        JT = J.T
        A = JT @ J + (float(damping) ** 2) * np.eye(J.shape[1])
        b = JT @ dX
        dq = np.linalg.solve(A, b)

        dq = float(alpha) * dq
        dq = np.clip(dq, -float(dq_max), float(dq_max))
        q = q + dq

    return q, False


def JointTrajInterpolate(q_traj, t, t_total):
    """Linear interpolation over a preplanned joint trajectory."""
    q_traj = np.asarray(q_traj, dtype=float)
    if q_traj.ndim != 2:
        raise ValueError("q_traj must be 2D: (M, dof)")
    M = q_traj.shape[0]
    if M == 0:
        raise ValueError("q_traj must contain at least 1 sample")
    if M == 1 or t_total <= 0:
        return q_traj[-1].copy()

    u = float(np.clip(float(t) / float(t_total), 0.0, 1.0)) * (M - 1)
    i = int(np.floor(u))
    if i >= M - 1:
        return q_traj[-1].copy()

    s = u - i
    return (1.0 - s) * q_traj[i] + s * q_traj[i + 1]


# Pre-plan joint trajectory by sampling the fitted task-space curve.
# This is much more stable than doing online IK every step when targets are hard/unreachable.
PLAN_SAMPLES = 250
IK_PLAN_DAMPING = 5e-2
IK_PLAN_ALPHA = 0.6
IK_PLAN_DQ_MAX = 0.15
IK_PLAN_TOL = 2e-3

_qpos_backup = data.qpos.copy()
mj.mj_forward(model, data)

plan_times = np.linspace(0.0, float(t_total), int(PLAN_SAMPLES))
q_plan = np.zeros((plan_times.shape[0], data.qpos.shape[0]), dtype=float)

q_guess = data.qpos.copy()
plan_ok = True
for i, tt in enumerate(plan_times):
    X_tgt = FittedSplineInterpolate(waypoints, tt, t_total)
    q_sol, ok = SolveIK_DLS(
        model, data, X_tgt, q_guess,
        max_iters=250,
        tol=IK_PLAN_TOL,
        damping=IK_PLAN_DAMPING,
        alpha=IK_PLAN_ALPHA,
        dq_max=IK_PLAN_DQ_MAX,
    )
    q_plan[i, :] = q_sol
    q_guess = q_sol
    if not ok:
        plan_ok = False

# Restore simulation state after planning
data.qpos[:] = _qpos_backup
mj.mj_forward(model, data)

if not plan_ok:
    print("[WARN] IK planning did not fully converge for all samples. Targets may be unreachable or too aggressive.")
###################################################

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # Store trajectory
        mj_end_eff_pos = data.site_xpos[0]
        traj_points.append(mj_end_eff_pos.copy())
        if len(traj_points) > MAX_TRAJ:
            traj_points.pop(0)
            
        # Get current joint configuration
        cur_q_pos = data.qpos.copy()

        # Track the pre-planned joint trajectory
        cur_ctrl = JointTrajInterpolate(q_plan, data.time, t_total)
        
        # Apply control input
        data.ctrl[:] = cur_ctrl
        mj.mj_step(model, data)


    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth = ',cam.azimuth,'\n','cam.elevation = ',cam.elevation,'\n','cam.distance = ',cam.distance)
        print('cam.lookat = np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    # Add trajectory as spheres
    for j in range(1, len(traj_points)):
        if scene.ngeom >= scene.maxgeom:
            break  # avoid overflow

        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        
        p1 = traj_points[j-1]
        p2 = traj_points[j]
        direction = p2 - p1
        midpoint = (p1 + p2) / 2.0
        
        # Configure this geom as a line
        geom.type = mj.mjtGeom.mjGEOM_SPHERE  # Use sphere for endpoints
        geom.rgba[:] = LINE_RGBA
        geom.size[:] = np.array([0.002, 0.002, 0.002])
        geom.pos[:] = midpoint
        geom.mat[:] = np.eye(3)  # no rotation
        geom.dataid = -1
        geom.segid = -1
        geom.objtype = 0
        geom.objid = 0
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
