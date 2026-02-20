from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Tuple

from ..settings.types import Point3, World, distance
from .base import PlanResult
from ..modules.grid import Grid3D, GridIndex

@dataclass
class AStarPlanner:
    resolution: float = 0.5
    allow_diagonal: bool = False

    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        # Validate basic parameters and start/goal feasibility.
        if self.resolution <= 0:
            return PlanResult([], False, 0, [])
        if not world.in_bounds(start) or not world.in_bounds(goal):
            return PlanResult([], False, 0, [])

        # Build a grid representation of the continuous world.
        grid = Grid3D(world, self.resolution)

        # Snap start/goal to the grid.
        start_p = grid.snap(start)
        goal_p = grid.snap(goal)
        if world.collides(start_p) or world.collides(goal_p):
            return PlanResult([], False, 0, [])

        # Convert positions to grid indices for the search.
        dims = grid.dims()
        start_idx = grid.to_index(start_p)
        goal_idx = grid.to_index(goal_p)

        # Priority queue of (f, g, node).
        open_heap: List[Tuple[float, float, GridIndex]] = []
        # Push the start node into the frontier.
        heapq.heappush(open_heap, (0.0, 0.0, start_idx))
        came_from: Dict[GridIndex, GridIndex] = {}
        g_score: Dict[GridIndex, float] = {start_idx: 0.0}

        iterations = 0
        visited = []
        while open_heap:
            iterations += 1
            _, curr_g, curr = heapq.heappop(open_heap)
            visited.append(grid.to_point(curr))
            if curr == goal_idx:
                # Reconstruct path by following parent links.
                path = self._reconstruct(grid, came_from, curr)
                return PlanResult(path, True, iterations, visited)

            curr_p = grid.to_point(curr)
            for nxt in self._neighbors(curr, dims):
                nxt_p = grid.to_point(nxt)
                if world.collides(nxt_p):
                    continue
                if world.path_collides(curr_p, nxt_p):
                    continue
                # g = cost so far + step distance.
                tentative_g = curr_g + distance(curr_p, nxt_p)
                if tentative_g < g_score.get(nxt, float("inf")):
                    came_from[nxt] = curr
                    g_score[nxt] = tentative_g
                    # f = g + heuristic (Euclidean distance).
                    f = tentative_g + distance(nxt_p, grid.to_point(goal_idx))
                    heapq.heappush(open_heap, (f, tentative_g, nxt))

        return PlanResult([], False, iterations, visited)

    def _neighbors(self, idx: GridIndex, dims: GridIndex) -> List[GridIndex]:
        # Generate neighbor indices (6-connected or 26-connected).
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
        self, grid: Grid3D, came_from: Dict[GridIndex, GridIndex], current: GridIndex
    ) -> List[Point3]:
        # Follow the came_from map backwards to build the path.
        path = [grid.to_point(current)]
        while current in came_from:
            current = came_from[current]
            path.append(grid.to_point(current))
        path.reverse()
        return path
