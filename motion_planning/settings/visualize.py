from __future__ import annotations

from typing import Iterable, Sequence

from .types import Box, Point3, Sphere, World


def plot_world(
    world: World,
    start: Point3 | None = None,
    goal: Point3 | None = None,
    path: Sequence[Point3] | None = None,
    visited: Sequence[Point3] | None = None,
    show: bool = True,
    ax=None,
):
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "matplotlib is required for visualization. Install with: pip install matplotlib"
        ) from exc

    if ax is None:
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection="3d")

    _plot_grid(ax, world.bounds_min, world.bounds_max, step=1.0)
    _plot_bounds(ax, world.bounds_min, world.bounds_max)
    boxes = [o for o in world.obstacles if isinstance(o, Box)]
    spheres = [o for o in world.obstacles if isinstance(o, Sphere)]
    if boxes:
        _plot_boxes(ax, boxes)
    if spheres:
        _plot_spheres(ax, spheres)

    if start is not None:
        ax.scatter([start[0]], [start[1]], [start[2]], c="green", s=50, label="start")
    if goal is not None:
        ax.scatter([goal[0]], [goal[1]], [goal[2]], c="red", s=50, label="goal")
    if path:
        xs, ys, zs = zip(*path)
        ax.plot(xs, ys, zs, c="blue", linewidth=2, label="path")
    if visited:
        xs, ys, zs = zip(*visited)
        ax.scatter(xs, ys, zs, c="purple", s=20, alpha=0.35, label="visited")

    ax.set_xlabel("X", color="black")
    ax.set_ylabel("Y", color="black")
    ax.set_zlabel("Z", color="black")
    ax.tick_params(axis="x", colors="black", labelsize=8)
    ax.tick_params(axis="y", colors="black", labelsize=8)
    ax.tick_params(axis="z", colors="black", labelsize=8)
    ax.xaxis.set_tick_params(labelsize=8)
    ax.yaxis.set_tick_params(labelsize=8)
    ax.zaxis.set_tick_params(labelsize=8)
    ax.set_xlim(world.bounds_min[0], world.bounds_max[0])
    ax.set_ylim(world.bounds_min[1], world.bounds_max[1])
    ax.set_zlim(world.bounds_min[2], world.bounds_max[2])
    ax.legend(loc="upper right")

    if show:
        plt.show()
    return ax


def _plot_bounds(ax, mn: Point3, mx: Point3) -> None:
    # Draw a faint bounding box.
    lines = [
        ((mn[0], mn[1], mn[2]), (mx[0], mn[1], mn[2])),
        ((mn[0], mn[1], mn[2]), (mn[0], mx[1], mn[2])),
        ((mn[0], mn[1], mn[2]), (mn[0], mn[1], mx[2])),
        ((mx[0], mx[1], mx[2]), (mn[0], mx[1], mx[2])),
        ((mx[0], mx[1], mx[2]), (mx[0], mn[1], mx[2])),
        ((mx[0], mx[1], mx[2]), (mx[0], mx[1], mn[2])),
        ((mn[0], mx[1], mn[2]), (mx[0], mx[1], mn[2])),
        ((mn[0], mx[1], mn[2]), (mn[0], mx[1], mx[2])),
        ((mx[0], mn[1], mn[2]), (mx[0], mx[1], mn[2])),
        ((mx[0], mn[1], mn[2]), (mx[0], mn[1], mx[2])),
        ((mn[0], mn[1], mx[2]), (mx[0], mn[1], mx[2])),
        ((mn[0], mn[1], mx[2]), (mn[0], mx[1], mx[2])),
    ]
    for a, b in lines:
        ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], c="gray", alpha=0.3)


def _plot_grid(ax, mn: Point3, mx: Point3, step: float = 1.0) -> None:
    if step <= 0:
        return
    xs = _frange(mn[0], mx[0], step)
    ys = _frange(mn[1], mx[1], step)
    zs = _frange(mn[2], mx[2], step)

    for y in ys:
        for z in zs:
            ax.plot([mn[0], mx[0]], [y, y], [z, z], c="gray", alpha=0.05, linewidth=0.5)
    for x in xs:
        for z in zs:
            ax.plot([x, x], [mn[1], mx[1]], [z, z], c="gray", alpha=0.05, linewidth=0.5)
    for x in xs:
        for y in ys:
            ax.plot([x, x], [y, y], [mn[2], mx[2]], c="gray", alpha=0.05, linewidth=0.5)


def _frange(start: float, stop: float, step: float) -> list[float]:
    vals: list[float] = []
    v = start
    while v <= stop + 1e-9:
        vals.append(v)
        v += step
    return vals


def _plot_boxes(ax, boxes: Iterable[Box]) -> None:
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    faces = []
    for box in boxes:
        mn = box.min_corner
        mx = box.max_corner
        v = [
            (mn[0], mn[1], mn[2]),
            (mx[0], mn[1], mn[2]),
            (mx[0], mx[1], mn[2]),
            (mn[0], mx[1], mn[2]),
            (mn[0], mn[1], mx[2]),
            (mx[0], mn[1], mx[2]),
            (mx[0], mx[1], mx[2]),
            (mn[0], mx[1], mx[2]),
        ]
        faces.extend(
            [
                [v[0], v[1], v[2], v[3]],
                [v[4], v[5], v[6], v[7]],
                [v[0], v[1], v[5], v[4]],
                [v[2], v[3], v[7], v[6]],
                [v[1], v[2], v[6], v[5]],
                [v[0], v[3], v[7], v[4]],
            ]
        )
    poly = Poly3DCollection(faces, alpha=0.2, facecolor="orange", edgecolor="k", linewidths=0.3)
    ax.add_collection3d(poly)


def _plot_spheres(ax, spheres: Iterable[Sphere]) -> None:
    import numpy as np

    u_vals = np.linspace(0, 2 * np.pi, 18)
    v_vals = np.linspace(0, np.pi, 14)
    for sphere in spheres:
        cx, cy, cz = sphere.center
        r = sphere.radius
        x = cx + r * np.outer(np.cos(u_vals), np.sin(v_vals))
        y = cy + r * np.outer(np.sin(u_vals), np.sin(v_vals))
        z = cz + r * np.outer(np.ones_like(u_vals), np.cos(v_vals))
        ax.plot_surface(x, y, z, color="orange", alpha=0.2, linewidth=0.2, edgecolor="k")
