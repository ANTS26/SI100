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
import ast  # 新增：用于解析笔画数据
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
    
    # 6D Jacobian
    jacp = np.zeros((3, 6))
    jacr = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, jacr, position_Q, 7)
    
    J = np.vstack([jacp, jacr]) # 6x6
    Jinv = np.linalg.pinv(J)

    # Position error
    X = position_Q.copy()
    dX = X_ref - X
    
    # Orientation error
    # 目标：保持初始姿态（假设初始姿态是垂直的）
    # 或者简单地，我们希望角速度为0，即保持当前姿态不变，但这会漂移。
    # 更好的方法是：定义一个固定的目标姿态 R_ref。
    # 这里我们假设初始时刻的姿态就是理想的垂直姿态。
    # 由于我们没有在函数外存储 R_ref，这里做一个简化：
    # 我们希望末端 Z 轴始终垂直向下 (0, 0, -1)
    # 但为了简单且不引入复杂数学库，我们尝试让姿态误差为 0 (即尽力维持当前姿态)
    # 配合初始姿态是好的这一事实。
    # 如果要更强力地纠正，需要计算旋转误差。
    
    # 简易版姿态维持：希望角速度为0。
    # 这意味着 dW = 0。
    # 但是如果已经歪了，就回不来了。
    
    # 改进版：使用全局变量存储初始姿态
    global g_R_ref
    if 'g_R_ref' not in globals() or g_R_ref is None:
        g_R_ref = data.site_xmat[0].reshape(3, 3).copy()
        
    R_cur = data.site_xmat[0].reshape(3, 3)
    R_ref = g_R_ref
    
    # 计算姿态误差 (近似: dW = 0.5 * sum(cross(n_cur, n_ref) ...))
    # R = [n, o, a]
    # error = 0.5 * (cross(n, n_d) + cross(o, o_d) + cross(a, a_d))
    
    n_cur, o_cur, a_cur = R_cur[:, 0], R_cur[:, 1], R_cur[:, 2]
    n_ref, o_ref, a_ref = R_ref[:, 0], R_ref[:, 1], R_ref[:, 2]
    
    dW = 0.5 * (np.cross(n_cur, n_ref) + np.cross(o_cur, o_ref) + np.cross(a_cur, a_ref))
    
    # 限制姿态误差的大小，防止剧烈抖动
    dW = np.clip(dW, -1.0, 1.0)

    # Combine errors
    error = np.hstack([dX, dW])

    # Compute control input
    dq = Jinv @ error
    
    # 增加阻尼，防止发散
    dq = dq * 0.5

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

# 新增：读取笔画数据
strokes_path = os.path.join(os.path.dirname(__file__), "output/final_strokes.txt")
strokes_data = []
if os.path.exists(strokes_path):
    with open(strokes_path, 'r') as f:
        try:
            strokes_data = ast.literal_eval(f.read())
        except:
            print("Error reading strokes file")

# 新增：写字状态变量
w_state = {'s': 0, 'p': 0, 'phase': 'move', 't0': 0, 'x0': None, 'tgt': None, 'dur': 0}
Z_LIFT = 0.15   # 抬笔高度
Z_WRITE = 0.095 # 写字高度
SPEED_AIR = 0.3 # 空中移动速度
SPEED_WRITE = 0.05 # 写字速度
WAIT_TIME = 0.2 # 停顿时间 (秒)

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
# LINE_RGBA[:] = np.array([0.0, 0.0, 1.0, 1.0])

# 边界点密度（越大越“实线”，但渲染更慢）
BOUNDARY_RES = 60
BOUNDARY_POINTS = []
for i in range(4):
    p0 = SQUARE_CORNERS[i]
    p1 = SQUARE_CORNERS[i + 1]
    for k in range(BOUNDARY_RES):
        BOUNDARY_POINTS.append(LinearInterpolate(p0, p1, k, BOUNDARY_RES))
BOUNDARY_POINTS.append(SQUARE_CORNERS[0].copy())
traj_points.extend(BOUNDARY_POINTS) # 把框也加进去（红色）

############################################
## 调参区（主要就调这里）
############################################
# PID 增益（6 维=6个关节）。KP大更快，KD大更稳，KI一般先不用。
PID_KP = np.ones(6) * 1.0
PID_KI = np.zeros(6) * 0.0
PID_KD = np.ones(6) * 0.1

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
        # 用蓝色在 z=0.1 平面标记写字区域边界
        # traj_points[:] = BOUNDARY_POINTS

        # ---------------------------------------------------------
        # 修改：写字逻辑（状态机与插值分离）
        # ---------------------------------------------------------
        X_ref = data.site_xpos[0].copy() # 默认保持当前位置
        InterpolateFunc = LinearInterpolate # 这里可以随时更换插值函数

        if g_mode == 'write' and not g_write_finished:
            # 1. 状态规划：如果当前段结束（x0 is None），规划下一段
            if w_state['x0'] is None:
                if w_state['s'] < len(strokes_data):
                    w_state['x0'] = data.site_xpos[0].copy()
                    w_state['t0'] = data.time
                    stroke = strokes_data[w_state['s']]
                    
                    if w_state['phase'] == 'move':
                        w_state['tgt'] = np.array([stroke[0][0], stroke[0][1], Z_LIFT])
                        w_state['dur'] = np.linalg.norm(w_state['tgt'] - w_state['x0']) / SPEED_AIR
                    elif w_state['phase'] == 'down':
                        w_state['tgt'] = np.array([stroke[0][0], stroke[0][1], Z_WRITE])
                        w_state['dur'] = 0.1
                    elif w_state['phase'] == 'wait_write': # 下笔后停顿
                        w_state['tgt'] = w_state['x0'].copy()
                        w_state['dur'] = WAIT_TIME
                    elif w_state['phase'] == 'write':
                        p_next = stroke[w_state['p'] + 1]
                        w_state['tgt'] = np.array([p_next[0], p_next[1], Z_WRITE])
                        w_state['dur'] = np.linalg.norm(w_state['tgt'] - w_state['x0']) / SPEED_WRITE
                        if w_state['dur'] < 0.01: w_state['dur'] = 0.01
                    elif w_state['phase'] == 'lift':
                        p_last = stroke[-1]
                        w_state['tgt'] = np.array([p_last[0], p_last[1], Z_LIFT])
                        w_state['dur'] = 0.1
                    elif w_state['phase'] == 'wait_move': # 抬笔后停顿
                        w_state['tgt'] = w_state['x0'].copy()
                        w_state['dur'] = WAIT_TIME
                else:
                    g_write_finished = True

            # 2. 执行运动与状态跳转
            if w_state['x0'] is not None:
                t_run = data.time - w_state['t0']
                if t_run < w_state['dur']:
                    X_ref = InterpolateFunc(w_state['x0'], w_state['tgt'], t_run, w_state['dur'])
                else:
                    # 完成当前段，准备跳转
                    X_ref = w_state['tgt'].copy()
                    w_state['x0'] = None 
                    
                    # 状态跳转逻辑
                    if w_state['phase'] == 'move': w_state['phase'] = 'down'
                    elif w_state['phase'] == 'down': w_state['phase'] = 'wait_write'
                    elif w_state['phase'] == 'wait_write': w_state['phase'] = 'write'; w_state['p'] = 0
                    elif w_state['phase'] == 'write':
                        if w_state['p'] < len(strokes_data[w_state['s']]) - 2: w_state['p'] += 1
                        else: w_state['phase'] = 'lift'
                    elif w_state['phase'] == 'lift': w_state['phase'] = 'wait_move'
                    elif w_state['phase'] == 'wait_move': w_state['phase'] = 'move'; w_state['s'] += 1

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
