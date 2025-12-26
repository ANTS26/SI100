import tkinter as tk
from tkinter import messagebox
import numpy as np
import math

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
SAMPLE_DIST = 0.03

# Z轴高度
Z_UP = 0.15
Z_DOWN = 0.1

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
            output_lines.append(f"    np.array([{start_pt[0]:.4f}, {start_pt[1]:.4f}, {Z_UP}]),")
            
            # 笔画过程：落笔状态
            for pt in stroke:
                output_lines.append(f"    np.array([{pt[0]:.4f}, {pt[1]:.4f}, {Z_DOWN}]),")
            
            # 终点：抬笔状态
            end_pt = stroke[-1]
            output_lines.append(f"    np.array([{end_pt[0]:.4f}, {end_pt[1]:.4f}, {Z_UP}]),")
            
            output_lines.append("    # End of stroke")

        output_lines.append("]")
        
        filename = "trajectory_output.txt"
        with open(filename, "w") as f:
            f.write("\n".join(output_lines))
            
        print(f"Saved to {filename}")
        messagebox.showinfo("Success", f"Trajectory saved to {filename}\nTotal strokes: {len(self.strokes)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboWriterApp(root)
    root.mainloop()
