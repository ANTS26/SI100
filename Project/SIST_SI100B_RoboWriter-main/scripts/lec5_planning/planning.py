###
# MuJoCo 规划练习
# SI100B 机器人编程
# 说明：本文件基于 MuJoCo 模板代码修改（模板来源：pab47.github.io 仓库）。
# 日期：2025 年 12 月
###

######
# 一、总览（先看这个）
# - 本文件大部分是 MuJoCo/GLFW 的模板框架：开窗、相机、交互、仿真循环、渲染。
# - 你真正需要重点理解/会改的主要是：轨迹读取与插值、写字判定、IK/关节控制、写完复位、日志与可视化。
######

######
# 二、导库与基础参数（依赖 + 路径 + 时长）
######
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import scipy as sp
import s

xml_path = '../../models/universal_robots_ur5e/scene.xml'  # 模型 XML 路径（相对本文件所在目录）
simend = 120  # 仿真总时长（秒）

######
# 三、窗口可视化与键鼠交互（均为框架函数，不做标记）
######
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # 更新鼠标按键状态
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # 更新鼠标位置
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # 计算鼠标位移并更新缓存
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # 没有按键按下：不做任何事
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # 获取窗口尺寸
    width, height = glfw.get_window_size(window)

    # 获取 shift 键状态
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # 根据鼠标按键决定相机操作类型
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

def _resolve_xml_path(xml_path_str: str) -> str:
    dirname = os.path.dirname(__file__)
    return os.path.join(dirname, xml_path_str)

def _init_mujoco(xml_path_str: str):
    model_obj = mj.MjModel.from_xml_path(xml_path_str)
    data_obj = mj.MjData(model_obj)
    cam_obj = mj.MjvCamera()
    opt_obj = mj.MjvOption()
    return model_obj, data_obj, cam_obj, opt_obj

def _init_glfw_window(width: int = 1920, height: int = 1080, title: str = "Demo"):
    glfw.init()
    window_obj = glfw.create_window(width, height, title, None, None)
    glfw.make_context_current(window_obj)
    glfw.swap_interval(1)
    return window_obj

def _init_rendering(model_obj, cam_obj, opt_obj):
    mj.mjv_defaultCamera(cam_obj)
    mj.mjv_defaultOption(opt_obj)
    scene_obj = mj.MjvScene(model_obj, maxgeom=10000)
    context_obj = mj.MjrContext(model_obj, mj.mjtFontScale.mjFONTSCALE_150.value)
    return scene_obj, context_obj

def _register_callbacks(window_obj):
    glfw.set_key_callback(window_obj, keyboard)
    glfw.set_cursor_pos_callback(window_obj, mouse_move)
    glfw.set_mouse_button_callback(window_obj, mouse_button)
    glfw.set_scroll_callback(window_obj, scroll)


def _configure_camera(cam_obj):
    cam_obj.azimuth = 89.8300000000001   # 摄像机水平旋转角度
    cam_obj.elevation = -87.16333333333334  # 摄像机垂直旋转角度
    cam_obj.distance = 1.66  # 摄像机距离模型的距离

# 统一的初始化入口：把“窗口/交互/渲染”放在一起，避免散落在各处
xml_path = _resolve_xml_path(xml_path)
model, data, cam, opt = _init_mujoco(xml_path)
window = _init_glfw_window()
scene, context = _init_rendering(model, cam, opt)
_register_callbacks(window)
_configure_camera(cam)



######
# 四、关节与控制（IK、复位目标、关节曲线图导出）
######

# 4.1 算关节增量的 IK 控制器
def IK_controller(model, data, X_ref, q_pos):
    # 计算雅可比（末端位置/姿态）
    position_Q = data.site_xpos[0]

    jacp = np.zeros((3, 6))
    jacr = np.zeros((3, 6))
    mj.mj_jac(model, data, jacp, jacr, position_Q, 7)

    J = np.vstack((jacp, jacr))
    Jinv = np.linalg.pinv(J)

    # 位置误差（参考位置 - 当前末端位置）
    X = position_Q.copy()
    dX = X_ref - X

    # 参考姿态：把末端朝向固定为一个常量基（作业里可理解为“笔尖姿态固定”）
    x_ref = np.array([1, 0, 0])
    y_ref = np.array([0, -1, 0])
    z_ref = np.array([0, 0, -1])

    R_curr = data.site_xmat[0].reshape(3, 3)
    x_curr = R_curr[:, 0]
    y_curr = R_curr[:, 1]
    z_curr = R_curr[:, 2]

    # 姿态误差（小角度近似形式）
    w_err = 0.5 * (np.cross(x_curr, x_ref) + np.cross(y_curr, y_ref) + np.cross(z_curr, z_ref))

    # 拼成 6 维误差并用伪逆求关节增量
    dq = Jinv @ np.concatenate((dX, w_err))

    return q_pos + dq

# 4.2关节位姿：初始姿态 + 写完后复位目标
init_qpos = np.array([-1.6353559, -1.28588984, 2.14838487, -2.61087434, -1.5903009, -0.06818645])
data.qpos[:] = init_qpos

FINAL_QPOS = np.array([0.0, -2.32, -1.38, -2.45, 1.57, 0.0], dtype=float)# 复位参数
RESET_DURATION = 5.0  # 复位用时（秒）
RESET_TOL = 1e-2      # 判定复位完成的关节误差阈值（弧度）
reset_started = False
reset_done = False
reset_start_time = 0.0
reset_q_start = None

#4.3 MuJoCo 控制回调：每个仿真步都会调用一次，用于写入 data.ctrl。
def controller(model, data):
    
    global reset_started
    global reset_done
    global reset_start_time
    global reset_q_start

    # 当前关节角（用于 IK 与复位插值）
    cur_q_pos_local = data.qpos.copy()

    # 阶段 A：写字（按轨迹 qn 走）
    if data.time < t_total:
        # 把总时间均分到各段，计算当前属于第几段以及段内时间
        t_sim = min(max(data.time, 0.0), t_total)
        seg_idx = int(t_sim // seg_dur)
        if seg_idx >= n_segments:
            seg_idx = n_segments - 1
        t_local = t_sim - seg_idx * seg_dur

        # 用局部二次拉格朗日插值生成更平滑的参考点（用上一点可减小抖动）
        q0 = qn[seg_idx]
        q1 = qn[min(seg_idx + 1, len(qn) - 1)]
        q_prev = qn[max(seg_idx - 1, 0)]
        X_ref = Lagrange3Interpolate(q_prev, q0, q1, t_local, seg_dur)

        # IK：把参考末端点 X_ref 转成关节角控制量
        cur_ctrl = IK_controller(model, data, X_ref, cur_q_pos_local)

    # 阶段 B：写完后复位到 FINAL_QPOS
    else:
        if not reset_started:
            reset_started = True
            reset_start_time = float(data.time)
            reset_q_start = cur_q_pos_local.copy()

        tau = float(data.time) - reset_start_time
        s = 1.0 if RESET_DURATION <= 0 else float(np.clip(tau / RESET_DURATION, 0.0, 1.0))
        q_ref = (1.0 - s) * reset_q_start + s * FINAL_QPOS
        cur_ctrl = q_ref

        if s >= 1.0 and (not reset_done):
            if float(np.linalg.norm(cur_q_pos_local - FINAL_QPOS)) <= RESET_TOL:
                reset_done = True

    data.ctrl[:] = cur_ctrl
mj.set_mjcb_control(controller)

# 4.4 关节状态曲线图导出（可选）
def SaveJointStatePlots(output_dir, t_arr, qpos_arr, qvel_arr=None):
    import importlib
    plt = importlib.import_module("matplotlib.pyplot")

    # 绘制关节角 qpos
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

    # 绘制关节速度 qvel
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

        

######
# 五、点渲染与日志（轨迹点缓存、关节日志）
######

# 5.1末端轨迹渲染缓存：用红点画“写字轨迹”（循环队列）
MAX_TRAJ = 3000
traj_points = np.zeros((MAX_TRAJ, 3))
traj_cursor = 0
traj_count = 0
LINE_RGBA = np.array([1.0, 0.0, 0.0, 1.0])

# 5.2关节状态日志：默认只在“写字状态”记录 qpos/qvel
LOG_WHEN_WRITING_ONLY = True
WRITE_Z_THRESHOLD = 0.099
joint_log_time = []
joint_log_qpos = []
joint_log_qvel = []



######
# 六、轨迹与写字判定（读配置、读轨迹、去重插值、判定是否写字）
######

# 6.1 配参数（曲面/球参数/容差）
CURVE_SWITCH = bool(getattr(s, "DEFAULT_CURVE_SWITCH", True))  # 默认值：会被轨迹文件里的 Curve_Switch 覆盖
SPHERE_CENTER = np.array([0.0, 0.35, 1.3], dtype=float)
SPHERE_R2 = 1.69
SPHERE_EQ_TOL = 0.01 #曲面容错


# 6.2 生成“是否写字”的判定函数
def MakeShouldRenderPoint(*, curve_switch: bool, write_z_threshold: float, sphere_center: np.ndarray, sphere_r2: float, sphere_eq_tol: float):
    def _ShouldRenderPoint(pos: np.ndarray):
        z = float(pos[2])
        z_ok = (0.0 <= z <= float(write_z_threshold))
        if not z_ok:
            return False
        if not curve_switch:
            return True
        dx = float(pos[0]) - float(sphere_center[0])
        dy = float(pos[1]) - float(sphere_center[1])
        dz = float(pos[2]) - float(sphere_center[2])
        sphere_expr = dx * dx + dy * dy + dz * dz
        return (sphere_expr - float(sphere_r2)) >= -float(sphere_eq_tol)

    return _ShouldRenderPoint


# 6.3 轨迹清洗：去重 + 插值补点
def IsCollinear3Pts(p0, p1, p2, *, atol=1e-9, rtol=1e-6):
    # 三点近似共线则视作共线（带容差）
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    v1 = p1 - p0
    v2 = p2 - p0
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 <= atol or n2 <= atol:
        return True
    cross_norm = float(np.linalg.norm(np.cross(v1, v2)))
    return cross_norm <= max(atol, rtol * n1 * n2)

def DeduplicateTrajectoryPoints(points, *, min_dist: float = 1e-4):
    # 轨迹去重：相邻点距离小于阈值的点丢弃
    points = list(points)
    if not points:
        return points
    out = [points[0]]
    for i in range(1, len(points)):
        if float(np.linalg.norm(points[i] - out[-1])) > float(min_dist):
            out.append(points[i])
    return out

def InterpolateTrajectoryPoints(points, *, max_dist: float = 0.05):
    # 轨迹补点：相邻点距离过大时插入中间点，避免跳变太快
    points = list(points)
    if not points:
        return points
    out = [points[0]]
    for i in range(1, len(points)):
        dist = float(np.linalg.norm(points[i] - points[i - 1]))
        if dist > float(max_dist):
            steps = int(dist / float(max_dist))
            for k in range(1, steps + 1):
                out.append(points[i - 1] + (points[i] - points[i - 1]) * k / (steps + 1))
        out.append(points[i])
    return out

def BuildTrajectoryPoints(*, min_dist: float, max_dist: float):
    # 从 s.py 读取轨迹与模式，并完成去重 + 插值补点
    qn_local, curve_switch_local = s.load_trajectory_output_with_mode()  # 读表
    qn_local = DeduplicateTrajectoryPoints(qn_local, min_dist=min_dist)
    qn_local = InterpolateTrajectoryPoints(qn_local, max_dist=max_dist)
    return qn_local, bool(curve_switch_local)

def ComputeSegmentTiming(points, *, total_time: float):
    # 根据轨迹点数计算分段数 n_segments 与每段时长 seg_dur
    n_segments_local = max(len(points) - 1, 1)
    seg_dur_local = float(total_time) / float(n_segments_local)
    return float(total_time), int(n_segments_local), float(seg_dur_local)


#6.4插值函数（线性 + 局部二次拉格朗日）
def LinearInterpolate(q0, q1, t, t_total):
    # 线性插值函数
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if t_total <= 0:
        return q1.copy()
    s = float(t) / float(t_total)
    s = float(np.clip(s, 0.0, 1.0))
    return (1.0 - s) * q0 + s * q1
def Lagrange3Interpolate(p_prev, p0, p1, t, t_total):
    # 局部二次拉格朗日插值
    p_prev = np.asarray(p_prev, dtype=float)
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    if t_total <= 0:
        return p1.copy()
    u = float(t) / float(t_total)
    u = float(np.clip(u, 0.0, 1.0))
    # 三个基函数（对应 u=-1/0/1）
    Lm1 = 0.5 * u * (u - 1.0)      # 对应 -1 的权重
    L0 = 1.0 - u * u               # 对应 0 的权重
    L1 = 0.5 * u * (u + 1.0)       # 对应 1 的权重
    return Lm1 * p_prev + L0 * p0 + L1 * p1



######
# 七、主循环执行（写字阶段/复位阶段/控制输入/渲染）
######

#7.1初始化
def InitTrajectoryAndWriting(*, total_time: float, min_dist: float, max_dist: float):
    qn_local, curve_switch_local = BuildTrajectoryPoints(min_dist=min_dist, max_dist=max_dist)
    t_total_local, n_segments_local, seg_dur_local = ComputeSegmentTiming(qn_local, total_time=total_time)
    should_render = MakeShouldRenderPoint(
        curve_switch=curve_switch_local,
        write_z_threshold=WRITE_Z_THRESHOLD,
        sphere_center=SPHERE_CENTER,
        sphere_r2=SPHERE_R2,
        sphere_eq_tol=SPHERE_EQ_TOL,
    )
    return qn_local, t_total_local, n_segments_local, seg_dur_local, should_render, bool(curve_switch_local)

qn, t_total, n_segments, seg_dur, ShouldRenderPoint, CURVE_SWITCH = InitTrajectoryAndWriting(total_time=simend,min_dist=1e-4,max_dist=0.05,)


#7.2主循环
while not glfw.window_should_close(window):
    time_prev = data.time
    while (data.time - time_prev < 1.0/60.0):
        # （1）末端轨迹缓存：只在“写字状态”采样末端点用于可视化
        mj_end_eff_pos = data.site_xpos[0]
        #print(mj_end_eff_pos)
        is_writing = ShouldRenderPoint(mj_end_eff_pos)
        if is_writing:
            # 控制点密度：与上一次记录点的距离足够大才加入（避免点太密）
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

        # （2）关节日志：默认只在写字阶段记录
        if (not LOG_WHEN_WRITING_ONLY) or is_writing:
            joint_log_time.append(float(data.time))
            joint_log_qpos.append(data.qpos.copy())
            joint_log_qvel.append(data.qvel.copy())
            
        # 控制输入由 mjcb_control 回调 controller() 自动写入 data.ctrl
        mj.mj_step(model, data)
        #data.time += 0.01

    # 进入渲染与事件处理（模板框架）
    if (data.time>=simend):
        break

    # 获取帧缓冲尺寸
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # 更新场景并渲染
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    # 将缓存的轨迹点画成小球（红色点列）
    start_idx = (traj_cursor - traj_count + MAX_TRAJ) % MAX_TRAJ
    sphere_count = 0
    
    for j in range(0, traj_count):
        idx = (start_idx + j) % MAX_TRAJ
        pos = traj_points[idx]
        
        if scene.ngeom >= scene.maxgeom:
            break  # 防止超过渲染几何体上限

        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        sphere_count += 1
        #print(scene.ngeom)
        
        # 配置该几何体为小球
        geom.type = mj.mjtGeom.mjGEOM_SPHERE
        geom.rgba[:] = LINE_RGBA
        geom.size[:] = np.array([0.002, 0.002, 0.002])
        geom.pos[:] = pos
        geom.mat[:] = np.eye(3)  # 不旋转
        geom.dataid = -1
        geom.segid = -1
        geom.objtype = 0
        geom.objid = 0
        
    print(f"Sphere count: {sphere_count}")
    mj.mjr_render(viewport, scene, context)

    # 交换 OpenGL 缓冲（由于 v-sync 可能阻塞）
    glfw.swap_buffers(window)

    # 处理 GUI 事件（触发 GLFW 回调）
    glfw.poll_events()

glfw.terminate()



######
# 八、结束输出
######

#8.1导出关节曲线图
out_dir = os.path.dirname(os.path.abspath(__file__))
t_arr = np.asarray(joint_log_time, dtype=float)
qpos_arr = np.asarray(joint_log_qpos, dtype=float)
qvel_arr = np.asarray(joint_log_qvel, dtype=float)
SaveJointStatePlots(out_dir, t_arr, qpos_arr, qvel_arr)
