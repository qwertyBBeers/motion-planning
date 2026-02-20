from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List, Optional

from ..settings.types import Point3, World, distance
from .base import PlanResult


@dataclass
class RRTStarPlanner:
    # Maximum distance to extend the tree in one step.
    step_size: float = 0.5
    # Maximum number of sampling iterations.
    max_iters: int = 2000
    # Probability of sampling the goal directly (goal bias).
    goal_sample_rate: float = 0.05
    # Radius used to select nearby nodes for rewire; if None, a default is used.
    neighbor_radius: Optional[float] = None

    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        # Basic input validation.
        if self.step_size <= 0.0 or self.max_iters <= 0:
            return PlanResult([], False, 0, [])
        if not world.in_bounds(start) or not world.in_bounds(goal):
            return PlanResult([], False, 0, [])
        if world.collides(start) or world.collides(goal):
            return PlanResult([], False, 0, [])

        # Tree data structures: nodes list, parent indices, and cost-to-come.
        rng = random.Random()
        nodes: List[Point3] = [start]
        parents: List[int] = [-1]
        costs: List[float] = [0.0]
        visited: List[Point3] = []

        # Fixed neighbor radius for simplicity in this toy implementation.
        radius = self.neighbor_radius
        if radius is None:
            radius = self.step_size * 2.5

        for i in range(self.max_iters):
            # Sample a random point with goal bias.
            sample = self._sample(world, goal, rng)
            # Find nearest existing node and extend toward the sample.
            nearest_idx = self._nearest(nodes, sample)
            nearest = nodes[nearest_idx]
            new_pt = self._steer(nearest, sample, self.step_size)

            # Skip if the new point does not move or collides.
            if distance(new_pt, nearest) < 1e-9:
                continue
            if world.collides(new_pt) or world.path_collides(nearest, new_pt):
                continue

            # Choose the best parent among neighbors to minimize cost.
            neighbor_indices = self._neighbors(nodes, new_pt, radius)
            best_parent = nearest_idx
            best_cost = costs[nearest_idx] + distance(nearest, new_pt)
            for n_idx in neighbor_indices:
                n_pt = nodes[n_idx]
                if world.path_collides(n_pt, new_pt):
                    continue
                c = costs[n_idx] + distance(n_pt, new_pt)
                if c < best_cost:
                    best_cost = c
                    best_parent = n_idx

            # Add the new node to the tree.
            nodes.append(new_pt)
            parents.append(best_parent)
            costs.append(best_cost)
            visited.append(new_pt)
            new_idx = len(nodes) - 1

            # Rewire: try to improve nearby nodes by going through new_pt.
            for n_idx in neighbor_indices:
                n_pt = nodes[n_idx]
                alt_cost = best_cost + distance(new_pt, n_pt)
                if alt_cost + 1e-9 < costs[n_idx]:
                    if not world.path_collides(new_pt, n_pt):
                        parents[n_idx] = new_idx
                        costs[n_idx] = alt_cost

            # If close enough to goal, attempt final connection.
            if distance(new_pt, goal) <= self.step_size:
                if not world.path_collides(new_pt, goal) and not world.collides(goal):
                    nodes.append(goal)
                    parents.append(new_idx)
                    costs.append(best_cost + distance(new_pt, goal))
                    visited.append(goal)
                    path = self._reconstruct(nodes, parents, len(nodes) - 1)
                    return PlanResult(path, True, i + 1, visited)

        # Failed to find a path within max_iters.
        return PlanResult([], False, self.max_iters, visited)

    def _sample(self, world: World, goal: Point3, rng: random.Random) -> Point3:
        # Randomly choose the goal or a uniform sample in the world bounds.
        if rng.random() < self.goal_sample_rate:
            return goal
        return (
            rng.uniform(world.bounds_min[0], world.bounds_max[0]),
            rng.uniform(world.bounds_min[1], world.bounds_max[1]),
            rng.uniform(world.bounds_min[2], world.bounds_max[2]),
        )

    def _nearest(self, nodes: List[Point3], target: Point3) -> int:
        # Linear nearest-neighbor search for simplicity.
        best_idx = 0
        best_dist = float("inf")
        for i, p in enumerate(nodes):
            d = distance(p, target)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _neighbors(self, nodes: List[Point3], target: Point3, radius: float) -> List[int]:
        # Collect neighbors within the fixed radius.
        neighbors: List[int] = []
        for i, p in enumerate(nodes):
            if distance(p, target) <= radius:
                neighbors.append(i)
        if not neighbors:
            neighbors.append(self._nearest(nodes, target))
        return neighbors

    def _steer(self, src: Point3, dst: Point3, step: float) -> Point3:
        # Move from src toward dst by at most step.
        d = distance(src, dst)
        if d <= step:
            return dst
        scale = step / d
        return (
            src[0] + (dst[0] - src[0]) * scale,
            src[1] + (dst[1] - src[1]) * scale,
            src[2] + (dst[2] - src[2]) * scale,
        )

    def _reconstruct(self, nodes: List[Point3], parents: List[int], idx: int) -> List[Point3]:
        # Follow parent links back to the root to build the path.
        path: List[Point3] = []
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        return path
