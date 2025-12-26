###
# Final project with MuJoCo
# SI100B Robotics Programming
# This code is modified based on the MuJoCo template code at https://github.com/pab47/pab47.github.io/tree/master.
# Date: Dec., 2025
###

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

xml_path = '../../models/universal_robots_ur5e/scene.xml' #xml file (assumes this is in the same folder as this file)
#################################
## USER CODE: Set simulation parameters here
#################################
simend = 180 #simulation time (second)
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)
#################################

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


########################################
## USER CODE: Set camera view here
########################################
# Example on how to set camera configuration
cam.azimuth =  85.66333333333347 
cam.elevation =  -35.33333333333329
cam.distance =  2.22
cam.lookat = np.array([ -0.09343103051557476 , 0.31359595076587915 , 0.22170312166086661 ])
########################################

# Initialize the controller
init_controller(model,data)

# Set the controller
mj.set_mjcb_control(controller)

# Initialize joint configuration
init_qpos = np.array([-1.6353559, -1.28588984, 2.14838487, -2.61087434, -1.5903009, -0.06818645])
data.qpos[:] = init_qpos
cur_q_pos = init_qpos.copy()

traj_points = []
MAX_TRAJ = 5e5  # Maximum number of tinit_qory points to store
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])

######################################
## USER CODE STARTS HERE
######################################

# 关节空间 PID：让机械臂更稳地跟踪目标关节角（用于任务2终止姿态）

IK_controller_base = IK_controller

# 插值函数（写笔画/轨迹会用到）
def LinearInterpolate(q0, q1, t, t_total):
    """线性插值：t=0 返回 q0，t=t_total 返回 q1。"""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if t_total <= 0:
        return q1.copy()
    s = float(t) / float(t_total)
    s = float(np.clip(s, 0.0, 1.0))
    return (1.0 - s) * q0 + s * q1


def QuadBezierInterpolate(q0, q1, q2, t, t_total):
    """二次 Bezier 插值：q0(起点), q1(控制点), q2(终点)。"""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    if t_total <= 0:
        return q2.copy()
    s = float(t) / float(t_total)
    s = float(np.clip(s, 0.0, 1.0))
    one_minus = 1.0 - s
    return (one_minus * one_minus) * q0 + (2.0 * one_minus * s) * q1 + (s * s) * q2


def CubicBezierInterpolate(q0, q1, q2, q3, t, t_total):
    """三次 Bezier 插值：q0(起点), q1/q2(控制点), q3(终点)。"""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    q3 = np.asarray(q3, dtype=float)
    if t_total <= 0:
        return q3.copy()
    s = float(t) / float(t_total)
    s = float(np.clip(s, 0.0, 1.0))
    one_minus = 1.0 - s
    return (
        (one_minus ** 3) * q0
        + (3.0 * (one_minus ** 2) * s) * q1
        + (3.0 * one_minus * (s ** 2)) * q2
        + (s ** 3) * q3
    )


# 画框：把写字区域边界“画”在 z=0.1 平面（蓝色，仅做标记，不驱动机械臂去走）
SQUARE_Z = 0.1
SQUARE_CORNERS = np.array(
    [
        [0.5, 0.1, SQUARE_Z],
        [0.5, 0.6, SQUARE_Z],
        [-0.5, 0.6, SQUARE_Z],
        [-0.5, 0.1, SQUARE_Z],
        [0.5, 0.1, SQUARE_Z],
    ],
    dtype=float,
)

# 蓝色
LINE_RGBA[:] = np.array([0.0, 0.0, 1.0, 1.0])

# 边界点密度（越大越“实线”，但渲染更慢）
BOUNDARY_RES = 600
BOUNDARY_POINTS = []
for i in range(4):
    p0 = SQUARE_CORNERS[i]
    p1 = SQUARE_CORNERS[i + 1]
    for k in range(BOUNDARY_RES):
        BOUNDARY_POINTS.append(LinearInterpolate(p0, p1, k, BOUNDARY_RES))
BOUNDARY_POINTS.append(SQUARE_CORNERS[0].copy())

# 从 final_project/output 读取笔画（image_to_strokes.py 的输出）
import ast


def _load_strokes_txt(path: str):
    """读取三维嵌套表格：strokes = [ [(x,y),(x,y)], ... ]。"""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            txt = f.read().strip()
        raw = ast.literal_eval(txt) if txt else []
    except Exception:
        return []

    cleaned = []
    for stroke in raw:
        pts = []
        for p in stroke:
            if p is None or len(p) < 2:
                continue
            pts.append((float(p[0]), float(p[1])))
        if len(pts) >= 2:
            cleaned.append(pts[:3])  # 每笔最多 3 点
    return cleaned


STROKES_TXT_PATH = os.path.join(dirname, 'output', 'final_strokes.txt')
STROKES_2D = _load_strokes_txt(STROKES_TXT_PATH)

# 写字执行状态（逐笔、逐线段）
WRITE_Z = 0.1
SEG_STEPS = 35  # 每条线分成多少步（越大越慢越平滑）
g_stroke_i = 0
g_seg_i = 0
g_seg_step = 0

# 轨迹显示：边框只初始化一次；写字轨迹追加后永久保留
g_traj_inited = False


def _get_write_xref(data):
    """按 STROKES_2D 输出当前时刻的末端参考点 (x,y,z)。

    - 一笔有 2 点：只画 1 段
    - 一笔有 3 点：画 2 段（0-1, 1-2）
    """
    global g_stroke_i, g_seg_i, g_seg_step, g_write_finished

    if not STROKES_2D:
        g_write_finished = True
        return data.site_xpos[0].copy()

    # 所有笔画写完
    if g_stroke_i >= len(STROKES_2D):
        g_write_finished = True
        return data.site_xpos[0].copy()

    stroke = STROKES_2D[g_stroke_i]
    if len(stroke) < 2:
        g_stroke_i += 1
        g_seg_i = 0
        g_seg_step = 0
        return data.site_xpos[0].copy()

    # 当前笔画一共多少段
    seg_count = len(stroke) - 1
    if g_seg_i >= seg_count:
        g_stroke_i += 1
        g_seg_i = 0
        g_seg_step = 0
        return _get_write_xref(data)

    x0, y0 = stroke[g_seg_i]
    x1, y1 = stroke[g_seg_i + 1]
    p0 = np.array([x0, y0, WRITE_Z], dtype=float)
    p1 = np.array([x1, y1, WRITE_Z], dtype=float)

    xref = LinearInterpolate(p0, p1, g_seg_step, SEG_STEPS)
    g_seg_step += 1

    # 段结束：切到下一段 / 下一笔
    if g_seg_step >= SEG_STEPS:
        g_seg_step = 0
        g_seg_i += 1
        if g_seg_i >= seg_count:
            g_stroke_i += 1
            g_seg_i = 0

    return xref


def _append_traj_point(p_xyz):
    """把点追加到 traj_points（用于渲染），并保证在 z=0.1 平面。"""
    p = np.asarray(p_xyz, dtype=float).copy()
    p[2] = WRITE_Z
    traj_points.append(p)

############################################
## 调参区（主要就调这里）
############################################
# PID 增益（6 维=6个关节）。KP大更快，KD大更稳，KI一般先不用。
PID_KP = np.ones(6) * 1.0
PID_KI = np.zeros(6) * 0.0
PID_KD = np.zeros(6) * 0.1*PID_KP

# 积分限幅（一般不用动）
PID_INTEGRAL_LIMIT = 0.5

# 任务2：最终必须停在这个关节角（rad）
TERMINAL_Q = np.array([0.0, -2.32, -1.38, -2.45, 1.57, 0.0], dtype=float)

# 终止阶段预留时间：初始姿态改得越远，这里越要大（建议 20~60 秒）
TERMINAL_SWITCH_BEFORE_END_SEC = 30.0
############################################


PID_STATE = {
    'integral': np.zeros(6),
    'last_error': np.zeros(6),
}

# IK 输出的目标关节角（内部使用）
g_q_ref = None

# 阶段：write=写字，terminal=回终止姿态
g_mode = 'write'
# 你写完字时把它置 True
g_write_finished = False


def _pid_vec(kp, ki, kd, state, error, dt):
    """6 维离散 PID。"""
    state['integral'] = state['integral'] + error * dt
    state['integral'] = np.clip(state['integral'], -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
    derivative = (error - state['last_error']) / dt
    output = (kp * error) + (ki * state['integral']) + (kd * derivative)
    state['last_error'] = error
    return output


def IK_controller(model, data, X_ref, q_pos):
    """包装 IK：把目标关节角缓存起来，供 PID 使用。"""
    global g_q_ref, g_mode

    # 终止阶段：直接用 TERMINAL_Q 作为目标
    if g_mode == 'terminal':
        q_ref = TERMINAL_Q.copy()
        g_q_ref = q_ref.copy()
        return q_ref

    # 写字阶段：正常用末端 IK
    q_ref = IK_controller_base(model, data, X_ref, q_pos)
    g_q_ref = q_ref.copy()
    return q_ref


def controller(model, data):
    """MuJoCo 控制回调：PID 跟踪目标关节角。"""
    global g_q_ref
    if g_q_ref is None:
        return

    dt = model.opt.timestep
    q = data.qpos[:6].copy()
    q_ref = g_q_ref[:6].copy()
    err = q_ref - q

    dq_cmd = _pid_vec(PID_KP, PID_KI, PID_KD, PID_STATE, err, dt)

    # position actuator：ctrl 是目标关节角
    q_cmd = q + dq_cmd

    # 安全裁剪（有 ctrlrange 才生效）
    if getattr(model, 'actuator_ctrlrange', None) is not None and model.actuator_ctrlrange.shape[0] >= 6:
        lo = model.actuator_ctrlrange[:6, 0]
        hi = model.actuator_ctrlrange[:6, 1]
        q_cmd = np.clip(q_cmd, lo, hi)

    data.ctrl[:] = q_cmd


# 重新绑定控制回调
mj.set_mjcb_control(controller)

######################################
## USER CODE ENDS HERE
######################################

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
        
        ######################################
        ## USER CODE STARTS HERE
        ######################################
        # 用蓝色在 z=0.1 平面标记写字区域边界（仅标记）
        # 注意：不要每帧覆盖 traj_points，否则字的轨迹无法“永久显示”。
        if not g_traj_inited:
            traj_points[:] = list(BOUNDARY_POINTS)
            g_traj_inited = True

        # 写字阶段：逐笔执行（每笔用线性插值把两点连起来）
        if g_mode == 'write' and not g_write_finished:
            X_ref = _get_write_xref(data)
            # 把计划轨迹点也画出来（永久保留在 z=0.1 平面）
            _append_traj_point(X_ref)
        else:
            X_ref = data.site_xpos[0].copy()

        # 任务2：切到终止姿态阶段（画完后，或到结束前 N 秒）
        if g_write_finished or (data.time >= simend - TERMINAL_SWITCH_BEFORE_END_SEC):
            if g_mode != 'terminal':
                g_mode = 'terminal'
                PID_STATE['integral'][:] = 0.0
                PID_STATE['last_error'][:] = 0.0
                g_q_ref = TERMINAL_Q.copy()
        
        ######################################
        ## USER CODE ENDS HERE
        ######################################

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
