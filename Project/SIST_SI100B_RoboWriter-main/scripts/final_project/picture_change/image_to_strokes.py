"""SI100 RoboWriter: image -> strokes

你指出的问题完全正确：之前把“骨架上的像素路径”当成“笔画”，会导致大量碎段/重复段，输出爆炸。

这里改成更贴近“一笔”的抽象：
- 把骨架当成图：
    - 节点 = 度 != 2 的骨架点（端点度=1 / 分叉点度>=3）
    - 笔画段 = 节点到节点之间的一段骨架
- 每段只输出 2~3 个 (x,y) 点：起点/（可选）弯折点/终点
- 额外加硬约束：总输出点数默认 <= 100（按段长度保留最长的）

输入：本目录 input/ 下的图片（png/jpg/jpeg/bmp）
输出：本目录 output/<image_stem>_strokes.txt

依赖：pip install pillow numpy
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Set, Tuple

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
            # 注意：numpy 的 bool 数组用 "+" 相加时，结果仍可能是 bool（不是计数）。
            # 这里必须显式转成 uint8 做计数。
            N = (
                P2.astype(np.uint8)
                + P3.astype(np.uint8)
                + P4.astype(np.uint8)
                + P5.astype(np.uint8)
                + P6.astype(np.uint8)
                + P7.astype(np.uint8)
                + P8.astype(np.uint8)
                + P9.astype(np.uint8)
            )
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


# ====== 骨架图分段（节点-节点） ======

Coord = Tuple[int, int]  # (row, col)


def _neighbors_coords8(r: int, c: int) -> List[Coord]:
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


def _in_bounds(h: int, w: int, r: int, c: int) -> bool:
    return 0 <= r < h and 0 <= c < w


def skeleton_degree(skel: np.ndarray) -> np.ndarray:
    p = skel.astype(bool)
    P2, P3, P4, P5, P6, P7, P8, P9 = _neighbors8(p)
    return (
        P2.astype(np.int16)
        + P3.astype(np.int16)
        + P4.astype(np.int16)
        + P5.astype(np.int16)
        + P6.astype(np.int16)
        + P7.astype(np.int16)
        + P8.astype(np.int16)
        + P9.astype(np.int16)
    )


def extract_segments(skel: np.ndarray, min_pixels: int = 10) -> List[List[Coord]]:
    """把骨架拆成若干段：节点(度!=2) 到 节点 之间的像素链。"""
    h, w = skel.shape
    p = skel.astype(bool)
    deg = skeleton_degree(p)

    nodes_mask = p & (deg != 2)
    nodes: Set[Coord] = set((int(r), int(c)) for r, c in np.argwhere(nodes_mask))

    visited_edges: Set[Tuple[Coord, Coord]] = set()
    segments: List[List[Coord]] = []

    def neighbors_of(rc: Coord) -> List[Coord]:
        r, c = rc
        out: List[Coord] = []
        for rr, cc in _neighbors_coords8(r, c):
            if _in_bounds(h, w, rr, cc) and p[rr, cc]:
                out.append((rr, cc))
        return out

    def edge_key(a: Coord, b: Coord) -> Tuple[Coord, Coord]:
        return (a, b) if a <= b else (b, a)

    def mark_edge(a: Coord, b: Coord) -> None:
        visited_edges.add(edge_key(a, b))

    def edge_seen(a: Coord, b: Coord) -> bool:
        return edge_key(a, b) in visited_edges

    # 主路径：从每个节点沿未访问边走到下一个节点
    for n in list(nodes):
        for nb in neighbors_of(n):
            if edge_seen(n, nb):
                continue

            path: List[Coord] = [n]
            prev = n
            cur = nb
            mark_edge(n, nb)

            while True:
                path.append(cur)
                if cur in nodes and cur != n:
                    break

                nbrs = neighbors_of(cur)
                nbrs = [x for x in nbrs if x != prev]
                if not nbrs:
                    break

                nxt = nbrs[0]
                mark_edge(cur, nxt)
                prev, cur = cur, nxt
                if len(path) > h * w:
                    break

            if len(path) >= min_pixels:
                segments.append(path)

    # 闭环（没有任何节点）
    if not nodes:
        pts = np.argwhere(p)
        if pts.size > 0:
            start: Coord = (int(pts[0, 0]), int(pts[0, 1]))
            path = [start]
            prev: Optional[Coord] = None
            cur = start
            for _ in range(h * w):
                nbrs = neighbors_of(cur)
                if prev is not None:
                    nbrs = [x for x in nbrs if x != prev]
                if not nbrs:
                    break
                nxt = nbrs[0]
                prev, cur = cur, nxt
                if cur == start:
                    break
                path.append(cur)
            if len(path) >= min_pixels:
                segments.append(path)

    return segments


def _max_deviation_idx(points: Sequence[Tuple[float, float]]) -> Tuple[int, float]:
    """返回离首尾连线最远的点索引和距离。"""
    if len(points) <= 2:
        return 0, 0.0
    (x1, y1) = points[0]
    (x2, y2) = points[-1]

    def dist(px: float, py: float) -> float:
        if x1 == x2 and y1 == y2:
            return math.hypot(px - x1, py - y1)
        num = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1)
        den = math.hypot(y2 - y1, x2 - x1)
        return num / den

    best_i = 0
    best_d = -1.0
    for i in range(1, len(points) - 1):
        d = dist(points[i][0], points[i][1])
        if d > best_d:
            best_d = d
            best_i = i
    return best_i, float(best_d)


def segment_to_2or3_points(
    seg_px: List[Coord],
    write_map: WriteMap,
    curvature_eps: float = 0.015,
) -> List[Tuple[float, float]]:
    """像素段 -> 2~3 个 (x,y) 点。"""
    xy = [write_map.pixel_to_xy(c, r) for (r, c) in seg_px]
    p0 = xy[0]
    p1 = xy[-1]
    if len(xy) <= 2:
        pts = [p0, p1]
    else:
        mid_idx, dev = _max_deviation_idx(xy)
        if dev > curvature_eps:
            pts = [p0, xy[mid_idx], p1]
        else:
            pts = [p0, p1]

    clipped: List[Tuple[float, float]] = []
    for x, y in pts:
        x = min(max(x, WRITE_X_MIN), WRITE_X_MAX)
        y = min(max(y, WRITE_Y_MIN), WRITE_Y_MAX)
        clipped.append((float(x), float(y)))
    return clipped


def process_one_image(image_path: Path, output_dir: Path, threshold: int = 128, invert: bool = False) -> Path:
    gray = load_image(image_path)
    fg = binarize_bw(gray, threshold=threshold, invert=invert)
    skel = zhang_suen_thinning(fg)

    write_map = WriteMap(width=gray.shape[1], height=gray.shape[0])

    # ====== 可调参数：控制“段数/点数” ======
    MIN_SEG_PIXELS = 10
    CURVATURE_EPS = 0.015
    MAX_TOTAL_POINTS = 100

    segments = extract_segments(skel, min_pixels=MIN_SEG_PIXELS)
    # 按段长度排序，优先保留主干
    segments.sort(key=len, reverse=True)

    strokes: List[List[Tuple[float, float]]] = []
    total_points = 0
    for seg in segments:
        stroke = segment_to_2or3_points(seg, write_map, curvature_eps=CURVATURE_EPS)
        if total_points + len(stroke) > MAX_TOTAL_POINTS:
            continue
        strokes.append(stroke)
        total_points += len(stroke)

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

    print(f"image: {image_path}")
    print(f"segments(all): {len(segments)}  kept_strokes: {len(strokes)}  total_points: {total_points}")
    return out_path


def find_first_image(input_dir: Path) -> Optional[Path]:
    exts = [".png", ".jpg", ".jpeg", ".bmp"]
    for p in sorted(input_dir.iterdir()):
        if p.suffix.lower() in exts and p.is_file():
            return p
    return None


def find_first_image_in_dirs(input_dirs: Sequence[Path]) -> Optional[Path]:
    for d in input_dirs:
        if not d.exists() or not d.is_dir():
            continue
        img = find_first_image(d)
        if img is not None:
            return img
    return None


def main() -> None:
    root = Path(__file__).resolve().parent
    input_dir = root / "input"
    output_dir = root / "output"

    img = find_first_image(input_dir)
    if img is None:
        raise SystemExit(f"没在 {input_dir} 找到图片（支持 png/jpg/jpeg/bmp）")

    out = process_one_image(img, output_dir=output_dir, threshold=128, invert=False)
    print(f"OK: {img} -> {out}")


if __name__ == "__main__":
    main()
