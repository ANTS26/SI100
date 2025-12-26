"""SI100 RoboWriter: image -> strokes

功能（按你的4点需求对齐）：
1) 从 input/ 读取图像并二值化（黑白两色）。
2) 基于骨架图找“起笔/收笔”端点（端点=8邻域中仅1个骨架邻居的像素）。
3) 根据 README 写字区域建立空白二维地图坐标系，并提供像素->平面坐标映射。
4) 输出三维嵌套表格：
   strokes: List[stroke]
   stroke: List[point]，长度 2~3（起点/可选中间点/终点）
   point: (x, y) 落在写字区域内

输出：output/<image_stem>_strokes.json

依赖：仅使用标准库 + Pillow + numpy
- Pillow / numpy 若未安装：pip install pillow numpy
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image


# ====== README 书写区域（固定） ======
# 平面 z=0.1 上的正方形，顶点：
# (0.5, 0.1), (0.5, 0.6), (-0.5, 0.6), (-0.5, 0.1)
WRITE_X_MIN = -0.5
WRITE_X_MAX = 0.5
WRITE_Y_MIN = 0.1
WRITE_Y_MAX = 0.6


@dataclass(frozen=True)
class WriteMap:
    """空白二维地图（只定义边界与分辨率/映射）。"""

    width: int
    height: int

    def pixel_to_xy(self, col: float, row: float) -> Tuple[float, float]:
        """像素坐标 -> (x,y)。

        col: [0, width-1] 从左到右
        row: [0, height-1] 从上到下
        """
        # x: left -> right maps to [-0.5, 0.5]
        x = WRITE_X_MIN + (col / max(1.0, (self.width - 1))) * (WRITE_X_MAX - WRITE_X_MIN)
        # y: top -> bottom maps to [0.6, 0.1] (向下是 y 变小)
        y = WRITE_Y_MAX - (row / max(1.0, (self.height - 1))) * (WRITE_Y_MAX - WRITE_Y_MIN)
        return float(x), float(y)


def load_image(path: Path) -> np.ndarray:
    """读取图像为灰度数组 uint8, shape=(H,W)。"""
    img = Image.open(path).convert("L")
    return np.array(img, dtype=np.uint8)


def binarize_bw(gray: np.ndarray, threshold: int = 128, invert: bool = False) -> np.ndarray:
    """二值化：True=黑色笔画(前景)，False=背景。"""
    if invert:
        fg = gray >= threshold
    else:
        fg = gray < threshold
    return fg.astype(bool)


# ====== 骨架化（Zhang-Suen thinning，纯 numpy 实现） ======

def _neighbors8(p: np.ndarray) -> List[np.ndarray]:
    """返回8邻域（按 Zhang-Suen 的 P2..P9 顺序）"""
    # p is binary image; neighbors computed via shifts
    P2 = np.roll(p, -1, axis=0)
    P3 = np.roll(np.roll(p, -1, axis=0), 1, axis=1)
    P4 = np.roll(p, 1, axis=1)
    P5 = np.roll(np.roll(p, 1, axis=0), 1, axis=1)
    P6 = np.roll(p, 1, axis=0)
    P7 = np.roll(np.roll(p, 1, axis=0), -1, axis=1)
    P8 = np.roll(p, -1, axis=1)
    P9 = np.roll(np.roll(p, -1, axis=0), -1, axis=1)
    return [P2, P3, P4, P5, P6, P7, P8, P9]


def zhang_suen_thinning(fg: np.ndarray, max_iters: int = 100) -> np.ndarray:
    """把前景二值图细化成1像素宽骨架。"""
    p = fg.copy().astype(bool)
    if p.ndim != 2:
        raise ValueError("fg must be 2D")

    # 为避免 roll 的边界连通问题，把边界清零
    p[[0, -1], :] = False
    p[:, [0, -1]] = False

    changed = True
    it = 0
    while changed and it < max_iters:
        changed = False
        it += 1

        for step in (0, 1):
            P2, P3, P4, P5, P6, P7, P8, P9 = _neighbors8(p)
            N = (P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9)
            # 计算 0->1 的过渡次数 A
            A = ((~P2 & P3).astype(np.uint8)
                 + (~P3 & P4).astype(np.uint8)
                 + (~P4 & P5).astype(np.uint8)
                 + (~P5 & P6).astype(np.uint8)
                 + (~P6 & P7).astype(np.uint8)
                 + (~P7 & P8).astype(np.uint8)
                 + (~P8 & P9).astype(np.uint8)
                 + (~P9 & P2).astype(np.uint8))

            if step == 0:
                m1 = P2 & P4 & P6
                m2 = P4 & P6 & P8
            else:
                m1 = P2 & P4 & P8
                m2 = P2 & P6 & P8

            cond = (
                p
                & (N >= 2) & (N <= 6)
                & (A == 1)
                & (~m1)
                & (~m2)
            )

            if np.any(cond):
                p[cond] = False
                changed = True

    return p


def skeleton_endpoints(skel: np.ndarray) -> List[Tuple[int, int]]:
    """端点：骨架像素，8邻域骨架邻居数=1。返回 (row,col)。"""
    p = skel.astype(bool)
    P2, P3, P4, P5, P6, P7, P8, P9 = _neighbors8(p)
    N = (P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9)
    ends = np.argwhere(p & (N == 1))
    return [(int(r), int(c)) for r, c in ends]


def _neighbors_coords(r: int, c: int) -> List[Tuple[int, int]]:
    return [
        (r - 1, c),
        (r - 1, c + 1),
        (r, c + 1),
        (r + 1, c + 1),
        (r + 1, c),
        (r + 1, c - 1),
        (r, c - 1),
        (r - 1, c - 1),
    ]


def trace_strokes_simple(skel: np.ndarray) -> List[List[Tuple[int, int]]]:
    """把骨架拆成若干条路径（像素坐标序列）。

    简化策略：
    - 以端点为起点，沿骨架走到下一个端点/断点
    - 对于闭环（无端点）暂不处理（会丢失），后续可扩展
    """
    H, W = skel.shape
    sk = skel.astype(bool)

    endpoints = set(skeleton_endpoints(sk))
    visited = set()
    strokes: List[List[Tuple[int, int]]] = []

    def in_bounds(rr: int, cc: int) -> bool:
        return 0 <= rr < H and 0 <= cc < W

    for ep in list(endpoints):
        if ep in visited:
            continue

        path = [ep]
        visited.add(ep)
        prev = None
        cur = ep

        while True:
            nbrs = []
            for rr, cc in _neighbors_coords(cur[0], cur[1]):
                if not in_bounds(rr, cc):
                    continue
                if not sk[rr, cc]:
                    continue
                if prev is not None and (rr, cc) == prev:
                    continue
                nbrs.append((rr, cc))

            if len(nbrs) == 0:
                break

            # 若分叉：这里选择“任意一个未访问邻居”，保证能跑通；后续可升级为图分解
            nxt = None
            for cand in nbrs:
                if cand not in visited:
                    nxt = cand
                    break
            if nxt is None:
                nxt = nbrs[0]

            prev = cur
            cur = nxt
            path.append(cur)
            visited.add(cur)

            if cur in endpoints and cur != ep:
                break

            if len(path) > H * W:
                break

        if len(path) >= 2:
            strokes.append(path)

    return strokes


def _rdp(points: Sequence[Tuple[float, float]], epsilon: float) -> List[Tuple[float, float]]:
    """Ramer–Douglas–Peucker，返回折线关键点。"""
    if len(points) <= 2:
        return list(points)

    (x1, y1) = points[0]
    (x2, y2) = points[-1]

    def dist_point_to_line(px: float, py: float) -> float:
        if x1 == x2 and y1 == y2:
            return math.hypot(px - x1, py - y1)
        num = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1)
        den = math.hypot(y2 - y1, x2 - x1)
        return num / den

    dmax = -1.0
    idx = -1
    for i in range(1, len(points) - 1):
        d = dist_point_to_line(points[i][0], points[i][1])
        if d > dmax:
            dmax = d
            idx = i

    if dmax > epsilon:
        left = _rdp(points[: idx + 1], epsilon)
        right = _rdp(points[idx:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def strokes_to_3d_table(
    pixel_strokes: List[List[Tuple[int, int]]],
    write_map: WriteMap,
    rdp_epsilon: float = 0.01,
) -> List[List[Tuple[float, float]]]:
    """像素路径 -> 三维嵌套表格（每笔 2~3 个点的 (x,y) 元组）。"""
    out: List[List[Tuple[float, float]]] = []

    for path in pixel_strokes:
        xy = [write_map.pixel_to_xy(c, r) for (r, c) in path]
        key = _rdp(xy, epsilon=rdp_epsilon)

        # 压缩到 2~3 个点：起点 + (可选中点) + 终点
        if len(key) <= 2:
            stroke = [key[0], key[-1]]
        else:
            mid = key[len(key) // 2]
            stroke = [key[0], mid, key[-1]]

        # 边界裁剪（保证在写字区域内）
        clipped = []
        for x, y in stroke:
            x = min(max(x, WRITE_X_MIN), WRITE_X_MAX)
            y = min(max(y, WRITE_Y_MIN), WRITE_Y_MAX)
            clipped.append((float(x), float(y)))

        out.append(clipped)

    return out


def process_one_image(image_path: Path, output_dir: Path, threshold: int = 128, invert: bool = False) -> Path:
    gray = load_image(image_path)
    fg = binarize_bw(gray, threshold=threshold, invert=invert)
    skel = zhang_suen_thinning(fg)

    write_map = WriteMap(width=gray.shape[1], height=gray.shape[0])

    pixel_strokes = trace_strokes_simple(skel)
    strokes = strokes_to_3d_table(pixel_strokes, write_map)

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{image_path.stem}_strokes.txt"

    # 纯文本输出：只写“三维嵌套表格”本体（Python 字面量风格，便于后续读取/复制）
    # strokes: List[List[Tuple[float,float]]]
    def fmt_point(pt: Tuple[float, float]) -> str:
        return f"({pt[0]:.6f}, {pt[1]:.6f})"

    lines: List[str] = []
    lines.append("[")
    for stroke in strokes:
        pts = ", ".join(fmt_point(p) for p in stroke)
        lines.append(f"  [{pts}],")
    lines.append("]")
    out_path.write_text("\n".join(lines), encoding="utf-8")
    return out_path


def find_first_image(input_dir: Path) -> Optional[Path]:
    exts = [".png", ".jpg", ".jpeg", ".bmp"]
    for p in sorted(input_dir.iterdir()):
        if p.suffix.lower() in exts and p.is_file():
            return p
    return None


def main() -> None:
    root = Path(__file__).resolve().parent
    input_dir = root / "input"
    output_dir = root / "output"

    img = find_first_image(input_dir)
    if img is None:
        raise SystemExit(f"input/ 里没找到图片（支持 png/jpg/jpeg/bmp）：{input_dir}")

    out = process_one_image(img, output_dir=output_dir, threshold=128, invert=False)
    print(f"OK: {img.name} -> {out}")


if __name__ == "__main__":
    main()
