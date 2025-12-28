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
simend = 100 #simulation time (second)
print_camera_config = 0 #set to 1 to print camera config
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
cam.azimuth =  89.8300000000001 #摄像机水平旋转角度
cam.elevation =  -87.16333333333334 #摄像机垂直旋转角度
cam.distance =  1.66 #摄像机距离模型的距离
#cam.lookat = np.array([ 0.29723477517870245 , 0.28277006411151073 , 0.6082647377843177 ])   #摄像机注视点坐标

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
SPHERE_SIZE = np.array([0.002, 0.002, 0.002])
IDENTITY_MAT = np.eye(3)

# qn = [
#     np.array([0.5,0.1,0.1]),
#     np.array([0.5,0.6,0.1]),
#     np.array([-0.5,0.6,0.1]),
#     np.array([-0.5,0.1,0.1])
#     # 可继续添加更多点
# ]
qn = [
    np.array([-0.2200, 0.5525, 0.12]),
    np.array([-0.2200, 0.5525, 0.1]),
    np.array([-0.2100, 0.5300, 0.1]),
    np.array([-0.2100, 0.5300, 0.12]),
    # End of stroke
    np.array([-0.2425, 0.5137, 0.12]),
    np.array([-0.2425, 0.5137, 0.1]),
    np.array([-0.2000, 0.5237, 0.1]),
    np.array([-0.2238, 0.4912, 0.1]),
    np.array([-0.2475, 0.4700, 0.1]),
    np.array([-0.2475, 0.4700, 0.12]),
    # End of stroke
    np.array([-0.2188, 0.4925, 0.12]),
    np.array([-0.2188, 0.4925, 0.1]),
    np.array([-0.2188, 0.4462, 0.1]),
    np.array([-0.2150, 0.4400, 0.1]),
    np.array([-0.2150, 0.4400, 0.12]),
    # End of stroke
    np.array([-0.1975, 0.4850, 0.12]),
    np.array([-0.1975, 0.4850, 0.1]),
    np.array([-0.1962, 0.4625, 0.1]),
    np.array([-0.1962, 0.4625, 0.12]),
    # End of stroke
    np.array([-0.1750, 0.5112, 0.12]),
    np.array([-0.1750, 0.5112, 0.1]),
    np.array([-0.1350, 0.5162, 0.1]),
    np.array([-0.1300, 0.5225, 0.1]),
    np.array([-0.1300, 0.5225, 0.12]),
    # End of stroke
    np.array([-0.1512, 0.5375, 0.12]),
    np.array([-0.1512, 0.5375, 0.1]),
    np.array([-0.1550, 0.4975, 0.1]),
    np.array([-0.1575, 0.4900, 0.1]),
    np.array([-0.1575, 0.4900, 0.12]),
    # End of stroke
    np.array([-0.1962, 0.4800, 0.12]),
    np.array([-0.1962, 0.4800, 0.1]),
    np.array([-0.1525, 0.4887, 0.1]),
    np.array([-0.1150, 0.5000, 0.1]),
    np.array([-0.1150, 0.5000, 0.12]),
    # End of stroke
    np.array([-0.1188, 0.5237, 0.12]),
    np.array([-0.1188, 0.5237, 0.1]),
    np.array([-0.1475, 0.4812, 0.1]),
    np.array([-0.1838, 0.4500, 0.1]),
    np.array([-0.1763, 0.4575, 0.1]),
    np.array([-0.1763, 0.4575, 0.12]),
    # End of stroke
    np.array([-0.1638, 0.4650, 0.12]),
    np.array([-0.1638, 0.4650, 0.1]),
    np.array([-0.1250, 0.4800, 0.1]),
    np.array([-0.1250, 0.4375, 0.1]),
    np.array([-0.1275, 0.4362, 0.1]),
    np.array([-0.1275, 0.4362, 0.12]),
    # End of stroke
    np.array([-0.1588, 0.4525, 0.12]),
    np.array([-0.1588, 0.4525, 0.1]),
    np.array([-0.1250, 0.4575, 0.1]),
    np.array([-0.1250, 0.4575, 0.12]),
    # End of stroke
    np.array([-0.1600, 0.4325, 0.12]),
    np.array([-0.1600, 0.4325, 0.1]),
    np.array([-0.1188, 0.4375, 0.1]),
    np.array([-0.1188, 0.4350, 0.1]),
    np.array([-0.1188, 0.4350, 0.12]),
    # End of stroke
    np.array([-0.0425, 0.5387, 0.12]),
    np.array([-0.0425, 0.5387, 0.1]),
    np.array([-0.0212, 0.5025, 0.1]),
    np.array([-0.0338, 0.4612, 0.1]),
    np.array([-0.0475, 0.4700, 0.1]),
    np.array([-0.0475, 0.4700, 0.12]),
    # End of stroke
    np.array([-0.0513, 0.5375, 0.12]),
    np.array([-0.0513, 0.5375, 0.1]),
    np.array([-0.0500, 0.4950, 0.1]),
    np.array([-0.0500, 0.4500, 0.1]),
    np.array([-0.0500, 0.4300, 0.1]),
    np.array([-0.0500, 0.4300, 0.12]),
    # End of stroke
    np.array([0.0000, 0.5125, 0.12]),
    np.array([0.0000, 0.5125, 0.1]),
    np.array([0.0400, 0.5175, 0.1]),
    np.array([0.0463, 0.5175, 0.1]),
    np.array([0.0463, 0.5175, 0.12]),
    # End of stroke
    np.array([0.0238, 0.5387, 0.12]),
    np.array([0.0238, 0.5387, 0.1]),
    np.array([0.0025, 0.4950, 0.1]),
    np.array([0.0363, 0.4800, 0.1]),
    np.array([0.0363, 0.4800, 0.12]),
    # End of stroke
    np.array([0.0200, 0.5012, 0.12]),
    np.array([0.0200, 0.5012, 0.1]),
    np.array([0.0188, 0.4575, 0.1]),
    np.array([0.0162, 0.4387, 0.1]),
    np.array([0.0162, 0.4387, 0.12]),
    # End of stroke
    np.array([0.0000, 0.4600, 0.12]),
    np.array([0.0000, 0.4600, 0.1]),
    np.array([-0.0137, 0.4350, 0.1]),
    np.array([-0.0137, 0.4350, 0.12]),
    # End of stroke
    np.array([0.0312, 0.4700, 0.12]),
    np.array([0.0312, 0.4700, 0.1]),
    np.array([0.0613, 0.4462, 0.1]),
    np.array([0.0613, 0.4462, 0.12]),
    # End of stroke
    np.array([0.1138, 0.5237, 0.12]),
    np.array([0.1138, 0.5237, 0.1]),
    np.array([0.1575, 0.5300, 0.1]),
    np.array([0.1725, 0.5350, 0.1]),
    np.array([0.1725, 0.5350, 0.12]),
    # End of stroke
    np.array([0.1475, 0.5463, 0.12]),
    np.array([0.1475, 0.5463, 0.1]),
    np.array([0.1250, 0.5100, 0.1]),
    np.array([0.1275, 0.5025, 0.1]),
    np.array([0.1275, 0.5025, 0.12]),
    # End of stroke
    np.array([0.1250, 0.5088, 0.12]),
    np.array([0.1250, 0.5088, 0.1]),
    np.array([0.1575, 0.4825, 0.1]),
    np.array([0.1562, 0.4800, 0.1]),
    np.array([0.1562, 0.4800, 0.12]),
    # End of stroke
    np.array([0.1312, 0.4900, 0.12]),
    np.array([0.1312, 0.4900, 0.1]),
    np.array([0.1625, 0.4962, 0.1]),
    np.array([0.1625, 0.4962, 0.12]),
    # End of stroke
    np.array([0.1300, 0.4725, 0.12]),
    np.array([0.1300, 0.4725, 0.1]),
    np.array([0.1637, 0.4825, 0.1]),
    np.array([0.1637, 0.4825, 0.12]),
    # End of stroke
    np.array([0.1200, 0.4550, 0.12]),
    np.array([0.1200, 0.4550, 0.1]),
    np.array([0.1613, 0.4650, 0.1]),
    np.array([0.1650, 0.4700, 0.1]),
    np.array([0.1650, 0.4700, 0.12]),
    # End of stroke
    np.array([0.1450, 0.4750, 0.12]),
    np.array([0.1450, 0.4750, 0.1]),
    np.array([0.1538, 0.4350, 0.1]),
    np.array([0.1562, 0.4263, 0.1]),
    np.array([0.1562, 0.4263, 0.12]),
    # End of stroke
    np.array([0.1813, 0.5187, 0.12]),
    np.array([0.1813, 0.5187, 0.1]),
    np.array([0.2225, 0.5250, 0.1]),
    np.array([0.2375, 0.5312, 0.1]),
    np.array([0.2375, 0.5312, 0.12]),
    # End of stroke
    np.array([0.1863, 0.5000, 0.12]),
    np.array([0.1863, 0.5000, 0.1]),
    np.array([0.2275, 0.5075, 0.1]),
    np.array([0.2400, 0.5100, 0.1]),
    np.array([0.2400, 0.5100, 0.12]),
    # End of stroke
    np.array([0.1737, 0.4750, 0.12]),
    np.array([0.1737, 0.4750, 0.1]),
    np.array([0.2150, 0.4900, 0.1]),
    np.array([0.2200, 0.4487, 0.1]),
    np.array([0.2050, 0.4675, 0.1]),
    np.array([0.2050, 0.4675, 0.12]),
    # End of stroke
    np.array([0.1987, 0.5475, 0.12]),
    np.array([0.1987, 0.5475, 0.1]),
    np.array([0.2000, 0.5075, 0.1]),
    np.array([0.2013, 0.4637, 0.1]),
    np.array([0.2050, 0.4237, 0.1]),
    np.array([0.2075, 0.4225, 0.1]),
    np.array([0.2075, 0.4225, 0.12]),
    # End of stroke
    np.array([-0.2325, 0.3862, 0.12]),
    np.array([-0.2325, 0.3862, 0.1]),
    np.array([-0.2687, 0.3675, 0.1]),
    np.array([-0.2463, 0.3337, 0.1]),
    np.array([-0.2225, 0.3525, 0.1]),
    np.array([-0.2225, 0.3525, 0.12]),
    # End of stroke
    np.array([-0.2087, 0.3937, 0.12]),
    np.array([-0.2087, 0.3937, 0.1]),
    np.array([-0.2100, 0.3538, 0.1]),
    np.array([-0.1850, 0.3225, 0.1]),
    np.array([-0.1838, 0.3250, 0.1]),
    np.array([-0.1838, 0.3250, 0.12]),
    # End of stroke
    np.array([-0.1663, 0.3675, 0.12]),
    np.array([-0.1663, 0.3675, 0.1]),
    np.array([-0.1638, 0.3262, 0.1]),
    np.array([-0.1337, 0.3550, 0.1]),
    np.array([-0.1275, 0.3237, 0.1]),
    np.array([-0.1275, 0.3237, 0.12]),
    # End of stroke
    np.array([-0.0413, 0.3900, 0.12]),
    np.array([-0.0413, 0.3900, 0.1]),
    np.array([-0.0800, 0.3700, 0.1]),
    np.array([-0.0750, 0.3287, 0.1]),
    np.array([-0.0363, 0.3412, 0.1]),
    np.array([-0.0350, 0.3450, 0.1]),
    np.array([-0.0350, 0.3450, 0.12]),
    # End of stroke
    np.array([-0.0225, 0.3962, 0.12]),
    np.array([-0.0225, 0.3962, 0.1]),
    np.array([-0.0262, 0.3500, 0.1]),
    np.array([-0.0025, 0.3175, 0.1]),
    np.array([0.0000, 0.3175, 0.1]),
    np.array([0.0000, 0.3175, 0.12]),
    # End of stroke
    np.array([0.0100, 0.3375, 0.12]),
    np.array([0.0100, 0.3375, 0.1]),
    np.array([0.0450, 0.3250, 0.1]),
    np.array([0.0450, 0.3250, 0.12]),
    # End of stroke
    np.array([0.0525, 0.3638, 0.12]),
    np.array([0.0525, 0.3638, 0.1]),
    np.array([0.0475, 0.3237, 0.1]),
    np.array([0.0587, 0.3638, 0.1]),
    np.array([0.0850, 0.3325, 0.1]),
    np.array([0.0875, 0.3100, 0.1]),
    np.array([0.0875, 0.3100, 0.12]),
    # End of stroke
    np.array([0.1325, 0.3950, 0.12]),
    np.array([0.1325, 0.3950, 0.1]),
    np.array([0.1300, 0.3538, 0.1]),
    np.array([0.1300, 0.3200, 0.1]),
    np.array([0.1300, 0.3200, 0.12]),
    # End of stroke
    np.array([0.1775, 0.4000, 0.12]),
    np.array([0.1775, 0.4000, 0.1]),
    np.array([0.1750, 0.3500, 0.1]),
    np.array([0.1700, 0.3187, 0.1]),
    np.array([0.1700, 0.3187, 0.12]),
    # End of stroke
    np.array([0.1250, 0.3575, 0.12]),
    np.array([0.1250, 0.3575, 0.1]),
    np.array([0.1700, 0.3600, 0.1]),
    np.array([0.1750, 0.3612, 0.1]),
    np.array([0.1750, 0.3612, 0.12]),
    # End of stroke
    np.array([0.2200, 0.3550, 0.12]),
    np.array([0.2200, 0.3550, 0.1]),
    np.array([0.1775, 0.3500, 0.1]),
    np.array([0.2025, 0.3175, 0.1]),
    np.array([0.2225, 0.3187, 0.1]),
    np.array([0.2225, 0.3187, 0.12]),
    # End of stroke
    np.array([0.2375, 0.3600, 0.12]),
    np.array([0.2375, 0.3600, 0.1]),
    np.array([0.2400, 0.3150, 0.1]),
    np.array([0.2500, 0.3550, 0.1]),
    np.array([0.2775, 0.3225, 0.1]),
    np.array([0.2775, 0.3075, 0.1]),
    np.array([0.2775, 0.3075, 0.12]),
    # End of stroke
    np.array([-0.3650, 0.2375, 0.12]),
    np.array([-0.3650, 0.2375, 0.1]),
    np.array([-0.3375, 0.2687, 0.1]),
    np.array([-0.3125, 0.2325, 0.1]),
    np.array([-0.3325, 0.1962, 0.1]),
    np.array([-0.3675, 0.1725, 0.1]),
    np.array([-0.3237, 0.1775, 0.1]),
    np.array([-0.2925, 0.1875, 0.1]),
    np.array([-0.2925, 0.1875, 0.12]),
    # End of stroke
    np.array([-0.2575, 0.2637, 0.12]),
    np.array([-0.2575, 0.2637, 0.1]),
    np.array([-0.2737, 0.2238, 0.1]),
    np.array([-0.2500, 0.1875, 0.1]),
    np.array([-0.2188, 0.2162, 0.1]),
    np.array([-0.2263, 0.2575, 0.1]),
    np.array([-0.2675, 0.2550, 0.1]),
    np.array([-0.2712, 0.2475, 0.1]),
    np.array([-0.2712, 0.2475, 0.12]),
    # End of stroke
    np.array([-0.1975, 0.2612, 0.12]),
    np.array([-0.1975, 0.2612, 0.1]),
    np.array([-0.1575, 0.2575, 0.1]),
    np.array([-0.1663, 0.2175, 0.1]),
    np.array([-0.1900, 0.1825, 0.1]),
    np.array([-0.1500, 0.1887, 0.1]),
    np.array([-0.1425, 0.1900, 0.1]),
    np.array([-0.1425, 0.1900, 0.12]),
    # End of stroke
    np.array([-0.1125, 0.2775, 0.12]),
    np.array([-0.1125, 0.2775, 0.1]),
    np.array([-0.1150, 0.2375, 0.1]),
    np.array([-0.0762, 0.2225, 0.1]),
    np.array([-0.0787, 0.1825, 0.1]),
    np.array([-0.1188, 0.1775, 0.1]),
    np.array([-0.1188, 0.1775, 0.12]),
    # End of stroke
    np.array([-0.1125, 0.2800, 0.12]),
    np.array([-0.1125, 0.2800, 0.1]),
    np.array([-0.0725, 0.2812, 0.1]),
    np.array([-0.0700, 0.2825, 0.1]),
    np.array([-0.0700, 0.2825, 0.12]),
    # End of stroke
    np.array([-0.0363, 0.2812, 0.12]),
    np.array([-0.0363, 0.2812, 0.1]),
    np.array([-0.0413, 0.2412, 0.1]),
    np.array([0.0025, 0.2400, 0.1]),
    np.array([0.0075, 0.1975, 0.1]),
    np.array([-0.0275, 0.1725, 0.1]),
    np.array([-0.0400, 0.1900, 0.1]),
    np.array([-0.0400, 0.1900, 0.12]),
    # End of stroke
    np.array([-0.0350, 0.2762, 0.12]),
    np.array([-0.0350, 0.2762, 0.1]),
    np.array([0.0075, 0.2787, 0.1]),
    np.array([0.0150, 0.2775, 0.1]),
    np.array([0.0150, 0.2775, 0.12]),
    # End of stroke
    np.array([0.0500, 0.2750, 0.12]),
    np.array([0.0500, 0.2750, 0.1]),
    np.array([0.0450, 0.2350, 0.1]),
    np.array([0.0837, 0.2150, 0.1]),
    np.array([0.0625, 0.1787, 0.1]),
    np.array([0.0312, 0.1750, 0.1]),
    np.array([0.0312, 0.1750, 0.12]),
    # End of stroke
    np.array([0.1175, 0.2687, 0.12]),
    np.array([0.1175, 0.2687, 0.1]),
    np.array([0.1125, 0.2262, 0.1]),
    np.array([0.1050, 0.1837, 0.1]),
    np.array([0.1038, 0.1675, 0.1]),
    np.array([0.1038, 0.1675, 0.12]),
    # End of stroke
    np.array([0.1625, 0.2575, 0.12]),
    np.array([0.1625, 0.2575, 0.1]),
    np.array([0.1425, 0.2188, 0.1]),
    np.array([0.1525, 0.1800, 0.1]),
    np.array([0.1925, 0.1875, 0.1]),
    np.array([0.2087, 0.2250, 0.1]),
    np.array([0.1850, 0.2600, 0.1]),
    np.array([0.1425, 0.2575, 0.1]),
    np.array([0.1362, 0.2387, 0.1]),
    np.array([0.1362, 0.2387, 0.12]),
    # End of stroke
    np.array([0.2325, 0.2662, 0.12]),
    np.array([0.2325, 0.2662, 0.1]),
    np.array([0.2738, 0.2700, 0.1]),
    np.array([0.2700, 0.2238, 0.1]),
    np.array([0.2612, 0.1787, 0.1]),
    np.array([0.2588, 0.1562, 0.1]),
    np.array([0.2588, 0.1562, 0.12]),
    # End of stroke
    np.array([0.3200, 0.2725, 0.12]),
    np.array([0.3200, 0.2725, 0.1]),
    np.array([0.3187, 0.2287, 0.1]),
    np.array([0.3200, 0.1875, 0.1]),
    np.array([0.3187, 0.1650, 0.1]),
    np.array([0.3187, 0.1650, 0.12]),
    # End of stroke
]



qn_filtered = [qn[0]]
for i in range(1, len(qn)):
    dist = np.linalg.norm(qn[i] - qn_filtered[-1])
    if dist > 1e-4:  # 只有距离大于 0.1mm 才保留
        qn_filtered.append(qn[i])
qn = qn_filtered
#t_total = simend
t_total = 50.0
n_segments = len(qn) - 1
seg_dur = float(t_total) / n_segments
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

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        # Store trajectory
        mj_end_eff_pos = data.site_xpos[0]
        if (mj_end_eff_pos[2] < 0.105):
            traj_points.append(mj_end_eff_pos.copy())
        if len(traj_points) > MAX_TRAJ:
            traj_points.pop(0)
            
        # Get current joint configuration
        cur_q_pos = data.qpos.copy()
        
        # Compute reference position across len(qn) points, evenly split total time
        t_sim = min(max(data.time, 0.0), t_total)
        seg_idx = int(t_sim // seg_dur)
        if seg_idx >= n_segments:
            seg_idx = n_segments - 1
        t_local = t_sim - seg_idx * seg_dur
        X_ref = LinearInterpolate(qn[seg_idx], qn[seg_idx+1], t_local, seg_dur)

        # Compute control input using IK
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos)
        
        # Apply control input
        data.ctrl[:] = cur_ctrl
        mj.mj_step(model, data)
        data.time += 0.001
######
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
    for j in range(1, len(traj_points), 3):
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
        geom.size[:] = SPHERE_SIZE
        geom.pos[:] = midpoint
        geom.mat[:] = IDENTITY_MAT  # no rotation
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
