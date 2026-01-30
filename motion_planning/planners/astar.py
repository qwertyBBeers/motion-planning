from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Tuple

from ..types import Point3, World, clamp_point, distance
from .base import PlanResult


GridIndex = Tuple[int, int, int]


@dataclass
class AStarPlanner:
    resolution: float = 1.0
    allow_diagonal: bool = False

    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        if self.resolution <= 0:
            return PlanResult([], False, 0, [])
        if not world.in_bounds(start) or not world.in_bounds(goal):
            return PlanResult([], False, 0, [])

        start_p = self._snap(world, start)
        goal_p = self._snap(world, goal)
        if world.collides(start_p) or world.collides(goal_p):
            return PlanResult([], False, 0, [])

        dims = self._grid_dims(world)
        start_idx = self._to_index(world, start_p)
        goal_idx = self._to_index(world, goal_p)

        open_heap: List[Tuple[float, float, GridIndex]] = []
        heapq.heappush(open_heap, (0.0, 0.0, start_idx))
        came_from: Dict[GridIndex, GridIndex] = {}
        g_score: Dict[GridIndex, float] = {start_idx: 0.0}

        iterations = 0
        visited = []
        while open_heap:
            iterations += 1
            _, curr_g, curr = heapq.heappop(open_heap)
            visited.append(self._to_point(world, curr))
            if curr == goal_idx:
                path = self._reconstruct(world, came_from, curr)
                return PlanResult(path, True, iterations, visited)

            curr_p = self._to_point(world, curr)
            for nxt in self._neighbors(curr, dims):
                nxt_p = self._to_point(world, nxt)
                if world.collides(nxt_p):
                    continue
                if world.path_collides(curr_p, nxt_p):
                    continue
                tentative_g = curr_g + distance(curr_p, nxt_p)
                if tentative_g < g_score.get(nxt, float("inf")):
                    came_from[nxt] = curr
                    g_score[nxt] = tentative_g
                    f = tentative_g + distance(nxt_p, self._to_point(world, goal_idx))
                    heapq.heappush(open_heap, (f, tentative_g, nxt))

        return PlanResult([], False, iterations, visited)

    def _grid_dims(self, world: World) -> GridIndex:
        dx = int((world.bounds_max[0] - world.bounds_min[0]) / self.resolution)
        dy = int((world.bounds_max[1] - world.bounds_min[1]) / self.resolution)
        dz = int((world.bounds_max[2] - world.bounds_min[2]) / self.resolution)
        return (dx + 1, dy + 1, dz + 1)

    def _snap(self, world: World, p: Point3) -> Point3:
        bx, by, bz = world.bounds_min
        rx = round((p[0] - bx) / self.resolution) * self.resolution + bx
        ry = round((p[1] - by) / self.resolution) * self.resolution + by
        rz = round((p[2] - bz) / self.resolution) * self.resolution + bz
        return clamp_point((rx, ry, rz), world.bounds_min, world.bounds_max)

    def _to_index(self, world: World, p: Point3) -> GridIndex:
        return (
            int(round((p[0] - world.bounds_min[0]) / self.resolution)),
            int(round((p[1] - world.bounds_min[1]) / self.resolution)),
            int(round((p[2] - world.bounds_min[2]) / self.resolution)),
        )

    def _to_point(self, world: World, idx: GridIndex) -> Point3:
        return (
            world.bounds_min[0] + idx[0] * self.resolution,
            world.bounds_min[1] + idx[1] * self.resolution,
            world.bounds_min[2] + idx[2] * self.resolution,
        )

    def _neighbors(self, idx: GridIndex, dims: GridIndex) -> List[GridIndex]:
        neighbors: List[GridIndex] = []
        if self.allow_diagonal:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == dy == dz == 0:
                            continue
                        nx, ny, nz = idx[0] + dx, idx[1] + dy, idx[2] + dz
                        if 0 <= nx < dims[0] and 0 <= ny < dims[1] and 0 <= nz < dims[2]:
                            neighbors.append((nx, ny, nz))
            return neighbors

        for dx, dy, dz in (
            (-1, 0, 0),
            (1, 0, 0),
            (0, -1, 0),
            (0, 1, 0),
            (0, 0, -1),
            (0, 0, 1),
        ):
            nx, ny, nz = idx[0] + dx, idx[1] + dy, idx[2] + dz
            if 0 <= nx < dims[0] and 0 <= ny < dims[1] and 0 <= nz < dims[2]:
                neighbors.append((nx, ny, nz))
        return neighbors

    def _reconstruct(
        self, world: World, came_from: Dict[GridIndex, GridIndex], current: GridIndex
    ) -> List[Point3]:
        path = [self._to_point(world, current)]
        while current in came_from:
            current = came_from[current]
            path.append(self._to_point(world, current))
        path.reverse()
        return path
