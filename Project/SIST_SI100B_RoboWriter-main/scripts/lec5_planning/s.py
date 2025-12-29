import tkinter as tk
from tkinter import messagebox
import numpy as np
import math
import os
###############################
Curve_Switch=False  # False: 平面书写；True: 曲面书写
###############################

# --- 配置参数 ---
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 400

# 机械臂工作空间范围
ROBOT_X_MIN = -0.5
ROBOT_X_MAX = 0.5
ROBOT_Y_MIN = 0.1
ROBOT_Y_MAX = 0.6

# 采样距离（在机械臂坐标系下的距离阈值）
# 越小点越密，越大点越稀疏。0.05 大约对应你要求的“一笔10个点左右”
SAMPLE_DIST = 0.01

# Z轴高度
Z_UP = 0.12
Z_DOWN = 0.1

# --- 曲面书写（球面内表面）参数 ---
# (x-0)^2 + (y-0.35)^2 + (z-1.3)^2 = 1.69, 且 0.01 <= z <= 0.1
SPHERE_CX = 0.0
SPHERE_CY = 0.35
SPHERE_CZ = 1.3
SPHERE_R2 = 1.69
SPHERE_R = math.sqrt(SPHERE_R2)

# 书写区域的 z 范围（世界坐标）
# 经验修正：z 过于接近 0 时接触/数值不稳定，统一抬到 0.01 上书写
WRITE_Z_MIN = 0.01
WRITE_Z_MAX = 0.1

# 将 (x, y) 限制到球面下半球在 z=WRITE_Z_MAX 截得的圆盘内，保证 0<=z<=0.1
_CAP_DZ = SPHERE_CZ - WRITE_Z_MAX
_CAP_R_XY = math.sqrt(max(0.0, SPHERE_R2 - _CAP_DZ * _CAP_DZ))


def clamp_xy_to_sphere_cap(x, y):
    """把 (x,y) 夹到球面帽(0<=z<=WRITE_Z_MAX)的可行圆盘内。

    对应条件：dx^2 + dy^2 <= _CAP_R_XY^2。
    """
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
    """给定 (x,y)，返回球面“下支”(z = cz - sqrt(...)) 上的 z。

    为满足题目 0<=z<=0.1 的限制，这里会先把 (x,y) 夹到对应球面帽的圆盘内。
    """
    x, y = clamp_xy_to_sphere_cap(x, y)
    dx = float(x) - SPHERE_CX
    dy = float(y) - SPHERE_CY
    radicand = SPHERE_R2 - dx * dx - dy * dy
    if radicand < 0.0:
        # 超出球面投影范围时，夹到边界（理论上工作空间不会触发）
        radicand = 0.0
    z = SPHERE_CZ - math.sqrt(radicand)
    # 数值安全夹紧
    if z < WRITE_Z_MIN:
        z = WRITE_Z_MIN
    if z > WRITE_Z_MAX:
        z = WRITE_Z_MAX
    return z


def project_to_sphere_cap(x, y):
    """返回落在题目球面帽区域上的 (x,y,z)。"""
    x2, y2 = clamp_xy_to_sphere_cap(x, y)
    z2 = sphere_surface_z_lower(x2, y2)
    return float(x2), float(y2), float(z2)

def make_pen_up_point(x, y):
    """抬笔点：固定绝对高度 z = Z_UP（>0.1），用于分段与抬笔移动。"""
    return np.array([float(x), float(y), float(Z_UP)], dtype=float)

def make_pen_down_point(x, y):
    """落笔点：两种模式
    - Curve_Switch=False: 平面书写 z = Z_DOWN
    - Curve_Switch=True : 球面内表面书写 z = sphere_surface_z_lower(x,y)
    """
    if Curve_Switch:
        x2, y2, z = project_to_sphere_cap(x, y)
        return np.array([x2, y2, float(z)], dtype=float)
    else:
        z = float(Z_DOWN)
    return np.array([float(x), float(y), float(z)], dtype=float)

class RoboWriterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboWriter Trajectory Recorder")

        # 数据存储
        self.strokes = [] # 存储所有笔画，每个笔画是一个点列表 [[x,y], [x,y]...]
        self.current_stroke = []

        # UI 布局
        self.canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
        self.canvas.pack(pady=10)

        # 坐标系提示
        self.canvas.create_text(10, CANVAS_HEIGHT/2, text="X=-0.5 (Left)", anchor="w")
        self.canvas.create_text(CANVAS_WIDTH-10, CANVAS_HEIGHT/2, text="X=0.5 (Right)", anchor="e")
        self.canvas.create_text(CANVAS_WIDTH/2, 10, text="Y=0.6 (Top)", anchor="n")
        self.canvas.create_text(CANVAS_WIDTH/2, CANVAS_HEIGHT-10, text="Y=0.1 (Bottom)", anchor="s")

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)
        
        tk.Button(btn_frame, text="Clear", command=self.clear_canvas).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Save to file", command=self.save_to_file).pack(side=tk.LEFT, padx=10)

        # 事件绑定
        self.canvas.bind("<Button-1>", self.start_stroke)
        self.canvas.bind("<B1-Motion>", self.record_point)
        self.canvas.bind("<ButtonRelease-1>", self.end_stroke)

    def pixel_to_robot(self, px, py):
        """
        将屏幕像素坐标转换为机械臂坐标
        注意：屏幕坐标原点在左上角，x向右，y向下
        机械臂坐标定义：
        屏幕左侧 -> X = -0.5
        屏幕右侧 -> X = 0.5
        屏幕上方 -> Y = 0.6
        屏幕下方 -> Y = 0.1
        """
        # 线性映射
        # X: 0 -> -0.5, Width -> 0.5
        rx = ROBOT_X_MIN + (px / CANVAS_WIDTH) * (ROBOT_X_MAX - ROBOT_X_MIN)
        
        # Y: 0 -> 0.6, Height -> 0.1
        ry = ROBOT_Y_MAX + (py / CANVAS_HEIGHT) * (ROBOT_Y_MIN - ROBOT_Y_MAX)
        
        return rx, ry

    def start_stroke(self, event):
        self.current_stroke = []
        rx, ry = self.pixel_to_robot(event.x, event.y)
        self.current_stroke.append((rx, ry))
        self.last_x, self.last_y = event.x, event.y

    def record_point(self, event):
        # 画线（视觉反馈）
        self.canvas.create_line(self.last_x, self.last_y, event.x, event.y, fill="black", width=2)
        self.last_x, self.last_y = event.x, event.y

        # 采样逻辑
        rx, ry = self.pixel_to_robot(event.x, event.y)
        last_rx, last_ry = self.current_stroke[-1]
        
        # 计算欧氏距离
        dist = math.sqrt((rx - last_rx)**2 + (ry - last_ry)**2)
        
        # 只有距离超过阈值才记录，保证点分布均匀
        if dist >= SAMPLE_DIST:
            self.current_stroke.append((rx, ry))

    def end_stroke(self, event):
        # 确保最后一笔也被记录（如果移动很小可能没被采样）
        rx, ry = self.pixel_to_robot(event.x, event.y)
        last_rx, last_ry = self.current_stroke[-1]
        if (rx != last_rx or ry != last_ry):
             self.current_stroke.append((rx, ry))
             
        if len(self.current_stroke) > 1:
            self.strokes.append(self.current_stroke)
        self.current_stroke = []

    def clear_canvas(self):
        self.canvas.delete("all")
        self.strokes = []
        # 重绘提示
        self.canvas.create_text(10, CANVAS_HEIGHT/2, text="X=-0.5", anchor="w")
        self.canvas.create_text(CANVAS_WIDTH-10, CANVAS_HEIGHT/2, text="X=0.5", anchor="e")
        self.canvas.create_text(CANVAS_WIDTH/2, 10, text="Y=0.6", anchor="n")
        self.canvas.create_text(CANVAS_WIDTH/2, CANVAS_HEIGHT-10, text="Y=0.1", anchor="s")

    def save_to_file(self):
        if not self.strokes:
            messagebox.showwarning("Warning", "No strokes to save!")
            return

        output_lines = ["qn = ["]
        # Fixed starting point as requested
        output_lines.append("    np.array([0.49805356, 0.10012646, 0.19241237]),")
        
        for stroke in self.strokes:
            # 处理每一笔
            # 1. 起点：先抬笔(Z_UP)，再落笔(Z_DOWN) -> 这里简化为直接加一个抬笔点在最前
            # 2. 终点：先写完(Z_DOWN)，再抬笔(Z_UP) -> 加一个抬笔点在最后
            
            # 按照你的样例格式：
            # 一笔开始：(x, y, 0.3) -> (x, y, 0.1)
            # 中间点：(x, y, 0.1)
            # 一笔结束：(x, y, 0.1) -> (x, y, 0.3)
            
            # 起点：抬笔状态
            start_pt = stroke[0]
            p_up_start = make_pen_up_point(start_pt[0], start_pt[1])
            output_lines.append(f"    np.array([{p_up_start[0]:.4f}, {p_up_start[1]:.4f}, {p_up_start[2]:.4f}]),")
            
            # 笔画过程：落笔状态
            for pt in stroke:
                p_down = make_pen_down_point(pt[0], pt[1])
                output_lines.append(f"    np.array([{p_down[0]:.4f}, {p_down[1]:.4f}, {p_down[2]:.4f}]),")
            
            # 终点：抬笔状态
            end_pt = stroke[-1]
            p_up_end = make_pen_up_point(end_pt[0], end_pt[1])
            output_lines.append(f"    np.array([{p_up_end[0]:.4f}, {p_up_end[1]:.4f}, {p_up_end[2]:.4f}]),")
            
            output_lines.append("    # End of stroke")

        output_lines.append("]")
        
        filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectory_output.txt")
        with open(filename, "w") as f:
            f.write("\n".join(output_lines))
            
        print(f"Saved to {filename}")
        messagebox.showinfo("Success", f"Trajectory saved to {filename}\nTotal strokes: {len(self.strokes)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboWriterApp(root)
    root.mainloop()
