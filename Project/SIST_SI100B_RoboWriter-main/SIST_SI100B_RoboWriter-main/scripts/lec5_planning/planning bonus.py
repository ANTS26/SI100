###6789
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
    # Compute Jacobian
    position_Q = data.site_xpos[0]

    jacp = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, None, position_Q, 7)

    J = jacp.copy()
    Jinv = np.linalg.pinv(J)

    # Reference point
    X = position_Q.copy()
    dX = X_ref - X

    # Compute control input
    dq = Jinv @ dX

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
MAX_TRAJ = 1000
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])

task_points = np.array([
    [0.5, 0.1, 0.1],
    [0.3, 0.2, 0.1],
    [0.6, 0.6, 0.1],
], dtype=float)
t_total = simend

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

def BuildBezierSegments(task_points):
    task_points = np.asarray(task_points, dtype=float)
    if task_points.ndim != 2 or task_points.shape[1] != 3:
        raise ValueError("task_points must be an array of shape (N, 3)")

    n_points = int(task_points.shape[0])
    if n_points <= 0:
        raise ValueError("task_points must contain at least one point")

    n_segments = int(np.ceil(n_points / 3.0))
    segments = np.empty((n_segments, 3, 3), dtype=float)

    for seg_idx in range(n_segments):
        start = seg_idx * 3
        group = task_points[start:start + 3]
        if group.shape[0] == 1:
            segments[seg_idx] = np.repeat(group, 3, axis=0)
        elif group.shape[0] == 2:
            segments[seg_idx] = np.vstack([group, group[-1]])
        else:
            segments[seg_idx] = group

    return segments

def BezierPathInterpolate(segments, t, t_total):
    segments = np.asarray(segments, dtype=float)
    if segments.ndim != 3 or segments.shape[1:] != (3, 3):
        raise ValueError("segments must be an array of shape (M, 3, 3)")

    n_segments = int(segments.shape[0])
    if n_segments <= 0:
        raise ValueError("segments must contain at least one segment")

    if t_total <= 0:
        return segments[-1, 2].copy()

    seg_duration = float(t_total) / float(n_segments)
    if seg_duration <= 0:
        return segments[-1, 2].copy()

    if t >= t_total:
        return segments[-1, 2].copy()
    if t <= 0:
        return segments[0, 0].copy()

    seg_idx = int(t / seg_duration)
    seg_idx = int(np.clip(seg_idx, 0, n_segments - 1))
    local_t = float(t) - float(seg_idx) * seg_duration

    q0, q1, q2 = segments[seg_idx]
    return QuadBezierInterpolate(q0, q1, q2, local_t, seg_duration)


bezier_segments = BuildBezierSegments(task_points)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # Store trajectory
        mj_end_eff_pos = data.site_xpos[0]
        if (mj_end_eff_pos[2] < 0.1):
            traj_points.append(mj_end_eff_pos.copy())
        if len(traj_points) > MAX_TRAJ:
            traj_points.pop(0)
            
        # Get current joint configuration
        cur_q_pos = data.qpos.copy()
        
        # Compute reference position
        X_ref = BezierPathInterpolate(bezier_segments, data.time, t_total)

        # Compute control input using IK
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos)
        
        # Apply control input
        data.ctrl[:] = cur_ctrl
        mj.mj_step(model, data)
        data.time += 0.02

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
