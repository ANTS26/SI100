"""render_strokes.py

把 output/final_strokes.txt（strokes 三维嵌套表格）渲染成一张图片，方便你检查：
1) 有坐标底纸（写字区域边框 + 轻网格）
2) 关键点加粗（画圆点）
3) 两点连线（相邻点连线）

依赖：Pillow（建议用和 robowriter 相同环境：pip install pillow）
用法（在本目录 output 下）：
- python render_strokes.py
输出：final_strokes_preview.png
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import List, Tuple

from PIL import Image, ImageDraw


# 写字区域边界（和 README/final.py 一致）
WRITE_X_MIN = -0.5
WRITE_X_MAX = 0.5
WRITE_Y_MIN = 0.1
WRITE_Y_MAX = 0.6


def load_strokes(path: Path) -> List[List[Tuple[float, float]]]:
    txt = path.read_text(encoding="utf-8").strip()
    raw = ast.literal_eval(txt) if txt else []

    strokes: List[List[Tuple[float, float]]] = []
    for stroke in raw:
        pts: List[Tuple[float, float]] = []
        for p in stroke:
            if p is None or len(p) < 2:
                continue
            pts.append((float(p[0]), float(p[1])))
        if len(pts) >= 2:
            strokes.append(pts[:3])
    return strokes


def xy_to_pixel(x: float, y: float, W: int, H: int, pad: int) -> Tuple[int, int]:
    # x: [-0.5,0.5] -> [pad, W-pad]
    u = (x - WRITE_X_MIN) / (WRITE_X_MAX - WRITE_X_MIN)
    px = int(round(pad + u * (W - 2 * pad)))

    # y: [0.1,0.6] -> [H-pad, pad]  (y 越大越靠上)
    v = (y - WRITE_Y_MIN) / (WRITE_Y_MAX - WRITE_Y_MIN)
    py = int(round((H - pad) - v * (H - 2 * pad)))
    return px, py


def draw_grid(draw: ImageDraw.ImageDraw, W: int, H: int, pad: int) -> None:
    # 外框
    rect = (pad, pad, W - pad, H - pad)
    draw.rectangle(rect, outline=(60, 60, 60), width=3)

    # 轻网格（10x10）
    grid_n = 10
    for i in range(1, grid_n):
        x = pad + int(round(i * (W - 2 * pad) / grid_n))
        y = pad + int(round(i * (H - 2 * pad) / grid_n))
        draw.line([(x, pad), (x, H - pad)], fill=(220, 220, 220), width=1)
        draw.line([(pad, y), (W - pad, y)], fill=(220, 220, 220), width=1)

    # 角点标注（小圆）
    corners = [
        (WRITE_X_MIN, WRITE_Y_MIN),
        (WRITE_X_MIN, WRITE_Y_MAX),
        (WRITE_X_MAX, WRITE_Y_MAX),
        (WRITE_X_MAX, WRITE_Y_MIN),
    ]
    for x, y in corners:
        cx, cy = xy_to_pixel(x, y, W, H, pad)
        r = 5
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=(80, 80, 80), width=2)


def render(strokes: List[List[Tuple[float, float]]], out_png: Path) -> None:
    W, H = 1100, 650
    pad = 60

    img = Image.new("RGB", (W, H), (255, 255, 255))
    draw = ImageDraw.Draw(img)

    # 1) 底纸
    draw_grid(draw, W, H, pad)

    # 2) 两点连线 + 3) 关键点加粗
    line_color = (30, 90, 200)
    point_color = (200, 30, 30)

    for stroke in strokes:
        pix = [xy_to_pixel(x, y, W, H, pad) for (x, y) in stroke]

        # 连线
        for i in range(1, len(pix)):
            draw.line([pix[i - 1], pix[i]], fill=line_color, width=4)

        # 关键点（加粗）
        for (px, py) in pix:
            r = 7
            draw.ellipse((px - r, py - r, px + r, py + r), fill=point_color, outline=(0, 0, 0), width=2)

    img.save(out_png)


def main() -> None:
    here = Path(__file__).resolve().parent
    strokes_path = here / "final_strokes.txt"
    if not strokes_path.exists():
        raise SystemExit(f"没找到 {strokes_path}，请先生成笔画文件")

    strokes = load_strokes(strokes_path)
    out_png = here / "final_strokes_preview.png"
    render(strokes, out_png)
    print(f"OK: strokes={len(strokes)} -> {out_png}")


if __name__ == "__main__":
    main()
