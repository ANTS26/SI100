###
# RoboWriter 轨迹采集器（s.py）
# - Tk 画笔画 -> 生成机械臂三维轨迹点 -> 保存 trajectory_output.txt
# - 轨迹文件同时写入 Curve_Switch，供 planning.py 自动读取（避免手改变量）
###

######
# 一、导库（尽量少）
######
import tkinter as tk
from tkinter import messagebox
import math
import os
import sys
import numpy as np


######
# 二、路径与文件
######
def _script_dir() -> str:
    # 返回本文件所在目录
    return os.path.dirname(os.path.abspath(__file__))


def _trajectory_output_path() -> str:
    # 返回轨迹输出文件路径
    return os.path.join(_script_dir(), "trajectory_output.txt")


######
# 三、配置参数（画布/范围/采样/高度）
######
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 400

ROBOT_X_MIN = -0.5
ROBOT_X_MAX = 0.5
ROBOT_Y_MIN = 0.1
ROBOT_Y_MAX = 0.6

SAMPLE_DIST = 0.01
Z_UP = 0.12
Z_DOWN = 0.1


######
# 四、模式开关（平面/曲面）
######
DEFAULT_CURVE_SWITCH = True  # False: 平面书写；True: 曲面书写
_curve_switch = bool(DEFAULT_CURVE_SWITCH)


def get_curve_switch() -> bool:
    # 获取当前“曲面书写”开关
    return bool(_curve_switch)


def set_curve_switch(v: bool) -> None:
    # 设置当前“曲面书写”开关
    global _curve_switch
    _curve_switch = bool(v)


######
# 五、轨迹读写（planning.py 会 import 调用这些函数）
######
def load_trajectory_output_with_mode():
    # 读取 trajectory_output.txt，并返回 (qn, Curve_Switch)
    filename = _trajectory_output_path()
    with open(filename, "r", encoding="utf-8") as f:
        src = f.read()
    ns = {"np": np}
    exec(src, ns)
    qn = list(ns["qn"])
    curve_switch = bool(ns.get("Curve_Switch", DEFAULT_CURVE_SWITCH))
    return qn, curve_switch


def load_trajectory_output():
    # 读取 trajectory_output.txt，只返回 qn（兼容旧调用）
    qn, _ = load_trajectory_output_with_mode()
    return qn


def _build_trajectory_output_lines(strokes) -> list[str]:
    # 把 strokes 编码成可 exec 的 Python 文本行（含 Curve_Switch 与 qn）
    lines = [f"Curve_Switch = {bool(get_curve_switch())}", "qn = ["]
    lines.append("    np.array([0.5, 0.1, 0.2]),")

    for stroke in strokes:
        start_pt = stroke[0]
        p_up_start = make_pen_up_point(start_pt[0], start_pt[1])
        lines.append(f"    np.array([{p_up_start[0]:.4f}, {p_up_start[1]:.4f}, {p_up_start[2]:.4f}]),")

        for pt in stroke:
            p_down = make_pen_down_point(pt[0], pt[1])
            lines.append(f"    np.array([{p_down[0]:.4f}, {p_down[1]:.4f}, {p_down[2]:.4f}]),")

        end_pt = stroke[-1]
        p_up_end = make_pen_up_point(end_pt[0], end_pt[1])
        lines.append(f"    np.array([{p_up_end[0]:.4f}, {p_up_end[1]:.4f}, {p_up_end[2]:.4f}]),")
        lines.append("    # End of stroke")

    lines.append("]")
    return lines


def _save_trajectory_output(strokes) -> str:
    # 保存 trajectory_output.txt，并返回文件名
    filename = _trajectory_output_path()
    lines = _build_trajectory_output_lines(strokes)
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return filename


######
# 六、曲面几何（球面帽）
######
SPHERE_CX = 0.0
SPHERE_CY = 0.35
SPHERE_CZ = 1.3
SPHERE_R2 = 1.69

WRITE_Z_MIN = 0.01
WRITE_Z_MAX = 0.1

_CAP_DZ = SPHERE_CZ - WRITE_Z_MAX
_CAP_R_XY = math.sqrt(max(0.0, SPHERE_R2 - _CAP_DZ * _CAP_DZ))


def clamp_xy_to_sphere_cap(x, y):
    # 把 (x,y) 限制到球面帽投影圆盘内（圆外点投到圆周）
    x = float(x)
    y = float(y)
    dx = x - SPHERE_CX
    dy = y - SPHERE_CY
    r = math.hypot(dx, dy)
    if r <= _CAP_R_XY or r <= 0.0:
        return x, y
    s = _CAP_R_XY / r
    return SPHERE_CX + dx * s, SPHERE_CY + dy * s


def sphere_surface_z_lower(x, y):
    # 对应球面下半部（内表面）的 z（带数值与边界保护）
    x, y = clamp_xy_to_sphere_cap(x, y)
    dx = float(x) - SPHERE_CX
    dy = float(y) - SPHERE_CY
    radicand = SPHERE_R2 - dx * dx - dy * dy
    if radicand < 0.0:
        radicand = 0.0
    z = SPHERE_CZ - math.sqrt(radicand)
    if z < WRITE_Z_MIN:
        z = WRITE_Z_MIN
    if z > WRITE_Z_MAX:
        z = WRITE_Z_MAX
    return float(z)


def project_to_sphere_cap(x, y):
    # 返回落在球面帽区域上的 (x,y,z)
    x2, y2 = clamp_xy_to_sphere_cap(x, y)
    z2 = sphere_surface_z_lower(x2, y2)
    return float(x2), float(y2), float(z2)


def make_pen_up_point(x, y):
    # 抬笔点：固定绝对高度 z=Z_UP
    return np.array([float(x), float(y), float(Z_UP)], dtype=float)


def make_pen_down_point(x, y):
    # 落笔点：平面 z=Z_DOWN；曲面 z=球面下支
    if get_curve_switch():
        x2, y2, z = project_to_sphere_cap(x, y)
        return np.array([x2, y2, float(z)], dtype=float)
    return np.array([float(x), float(y), float(Z_DOWN)], dtype=float)


######
# 七、Tk 画板（采样/可视化/保存/启动仿真）
######
class RoboWriterApp:
    def __init__(self, root):
        # 初始化 UI 与数据
        self.root = root
        self.root.title("RoboWriter Trajectory Recorder")

        self._launch_planning = False
        self.strokes = []
        self.current_stroke = []

        self.canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
        self.canvas.pack(pady=10)

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)

        self.curve_var = tk.BooleanVar(value=bool(get_curve_switch()))
        tk.Checkbutton(btn_frame, text="曲面书写", variable=self.curve_var, command=self.on_toggle_curve).pack(
            side=tk.LEFT, padx=10
        )
        tk.Button(btn_frame, text="重画", command=self.clear_canvas).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="保存", command=self.save_to_file).pack(side=tk.LEFT, padx=10)

        self.canvas.bind("<Button-1>", self.start_stroke)
        self.canvas.bind("<B1-Motion>", self.record_point)
        self.canvas.bind("<ButtonRelease-1>", self.end_stroke)

        self._redraw_background()

    def _redraw_background(self):
        # 重绘遮罩与固定提示（不影响你画的线）
        self._draw_sphere_cap_outside_mask()
        self.canvas.create_text(CANVAS_WIDTH / 2, 10, text="俯视图", anchor="n", justify="center")
        self.canvas.create_text(
            CANVAS_WIDTH / 2, CANVAS_HEIGHT - 10, text="【机械臂】\n不要附近写字", anchor="s", justify="center"
        )

    def pixel_to_robot(self, px, py):
        # 像素坐标 -> 机械臂坐标（线性映射）
        rx = ROBOT_X_MIN + (float(px) / float(CANVAS_WIDTH)) * (ROBOT_X_MAX - ROBOT_X_MIN)
        ry = ROBOT_Y_MAX + (float(py) / float(CANVAS_HEIGHT)) * (ROBOT_Y_MIN - ROBOT_Y_MAX)
        return float(rx), float(ry)

    def robot_to_pixel(self, rx, ry):
        # 机械臂坐标 -> 像素坐标（pixel_to_robot 的逆）
        px = (float(rx) - ROBOT_X_MIN) / (ROBOT_X_MAX - ROBOT_X_MIN) * CANVAS_WIDTH
        py = (float(ry) - ROBOT_Y_MAX) / (ROBOT_Y_MIN - ROBOT_Y_MAX) * CANVAS_HEIGHT
        return float(px), float(py)

    def _draw_sphere_cap_outside_mask(self):
        # 曲面模式下：把投影圆盘外区域涂成“高透明红色”
        self.canvas.delete("cap_mask")
        if not get_curve_switch():
            return

        cx_px, cy_px = self.robot_to_pixel(SPHERE_CX, SPHERE_CY)
        rx_scale = CANVAS_WIDTH / (ROBOT_X_MAX - ROBOT_X_MIN)
        ry_scale = CANVAS_HEIGHT / (ROBOT_Y_MAX - ROBOT_Y_MIN)
        r_px = float(_CAP_R_XY) * min(rx_scale, ry_scale)

        self.canvas.create_rectangle(
            0,
            0,
            CANVAS_WIDTH,
            CANVAS_HEIGHT,
            fill="#ff0000",
            outline="",
            stipple="gray12",
            tags=("cap_mask",),
        )
        self.canvas.create_oval(
            cx_px - r_px,
            cy_px - r_px,
            cx_px + r_px,
            cy_px + r_px,
            fill="white",
            outline="",
            tags=("cap_mask",),
        )
        self.canvas.tag_lower("cap_mask")

    def start_stroke(self, event):
        # 开始新的一笔
        self.current_stroke = []
        rx, ry = self.pixel_to_robot(event.x, event.y)
        self.current_stroke.append((rx, ry))
        self.last_x, self.last_y = event.x, event.y

    def record_point(self, event):
        # 按住左键拖动：画线 + 采样
        self.canvas.create_line(self.last_x, self.last_y, event.x, event.y, fill="black", width=2)
        self.last_x, self.last_y = event.x, event.y

        rx, ry = self.pixel_to_robot(event.x, event.y)
        last_rx, last_ry = self.current_stroke[-1]
        dist = math.sqrt((rx - last_rx) ** 2 + (ry - last_ry) ** 2)
        if dist >= SAMPLE_DIST:
            self.current_stroke.append((rx, ry))

    def end_stroke(self, event):
        # 松开鼠标：结束一笔并入库
        rx, ry = self.pixel_to_robot(event.x, event.y)
        last_rx, last_ry = self.current_stroke[-1]
        if rx != last_rx or ry != last_ry:
            self.current_stroke.append((rx, ry))
        if len(self.current_stroke) > 1:
            self.strokes.append(self.current_stroke)
        self.current_stroke = []

    def clear_canvas(self):
        # 清空画布并重置数据
        self.canvas.delete("all")
        self.strokes = []
        self._redraw_background()

    def on_toggle_curve(self):
        # 切换曲面/平面，并重绘遮罩
        set_curve_switch(bool(self.curve_var.get()))
        self._draw_sphere_cap_outside_mask()

    def save_to_file(self):
        # 保存到 trajectory_output.txt，并询问是否启动仿真
        if not self.strokes:
            messagebox.showwarning("警告", "你还没画任何笔画")
            return

        filename = _save_trajectory_output(self.strokes)
        print(f"Saved to {filename}")

        start_sim = messagebox.askyesno(
            "开始仿真？",
            f"已保存到：{filename}\n总笔画数: {len(self.strokes)}\n\n是否现在开始仿真（运行 planning.py）？",
        )
        if start_sim:
            self._launch_planning = True
            self.root.quit()
            return

        messagebox.showinfo("保存成功", "保存成功。点击确定将自动关闭窗口。")
        self.root.quit()


######
# 八、主入口（保持 import 安全）
######
def _run_tk_app() -> bool:
    # 运行 Tk 主循环，并返回是否启动 planning
    root = tk.Tk()
    app = RoboWriterApp(root)
    root.mainloop()
    launch_planning = bool(getattr(app, "_launch_planning", False))
    try:
        root.destroy()
    except Exception:
        pass
    return launch_planning


def _launch_planning_in_process() -> None:
    #同一环境
    base_dir = _script_dir()
    if base_dir not in sys.path:
        sys.path.insert(0, base_dir)
    import planning  # noqa: F401


if __name__ == "__main__":
    if _run_tk_app():
        _launch_planning_in_process()
