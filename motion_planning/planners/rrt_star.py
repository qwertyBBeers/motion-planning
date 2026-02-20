from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List, Optional

from ..settings.types import Point3, World, distance
from .base import PlanResult


@dataclass
class RRTStarPlanner:
    # 한 번에 확장할 최대 거리.
    step_size: float = 0.5
    # 샘플링 반복 횟수 상한.
    max_iters: int = 2000
    # 목표를 직접 샘플링할 확률(골 바이어스).
    goal_sample_rate: float = 0.05
    # 리와이어에 사용할 이웃 반경(없으면 기본값 사용).
    neighbor_radius: Optional[float] = None

    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        # 기본 입력 유효성 검사.
        if self.step_size <= 0.0 or self.max_iters <= 0:
            return PlanResult([], False, 0, [])
        if not world.in_bounds(start) or not world.in_bounds(goal):
            return PlanResult([], False, 0, [])
        if world.collides(start) or world.collides(goal):
            return PlanResult([], False, 0, [])

        # 트리 저장 구조: 노드, 부모 인덱스, 비용.
        rng = random.Random()
        nodes: List[Point3] = [start]
        parents: List[int] = [-1]
        costs: List[float] = [0.0]
        visited: List[Point3] = []

        # 간단한 구현을 위해 고정 반경 사용.
        radius = self.neighbor_radius
        if radius is None:
            radius = self.step_size * 2.5

        for i in range(self.max_iters):
            # 골 바이어스를 포함한 랜덤 샘플링.
            sample = self._sample(world, goal, rng)
            # 최근접 노드를 찾고 샘플 방향으로 확장.
            nearest_idx = self._nearest(nodes, sample)
            nearest = nodes[nearest_idx]
            new_pt = self._steer(nearest, sample, self.step_size)

            # 이동이 없거나 충돌이면 건너뛴다.
            if distance(new_pt, nearest) < 1e-9:
                continue
            if world.collides(new_pt) or world.path_collides(nearest, new_pt):
                continue

            # 이웃 중 비용이 최소가 되는 부모를 선택.
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

            # 새 노드를 트리에 추가.
            nodes.append(new_pt)
            parents.append(best_parent)
            costs.append(best_cost)
            visited.append(new_pt)
            new_idx = len(nodes) - 1

            # 리와이어: new_pt를 통해 비용을 줄일 수 있으면 갱신.
            for n_idx in neighbor_indices:
                n_pt = nodes[n_idx]
                alt_cost = best_cost + distance(new_pt, n_pt)
                if alt_cost + 1e-9 < costs[n_idx]:
                    if not world.path_collides(new_pt, n_pt):
                        parents[n_idx] = new_idx
                        costs[n_idx] = alt_cost

            # 충분히 가까우면 목표로 연결을 시도.
            if distance(new_pt, goal) <= self.step_size:
                if not world.path_collides(new_pt, goal) and not world.collides(goal):
                    nodes.append(goal)
                    parents.append(new_idx)
                    costs.append(best_cost + distance(new_pt, goal))
                    visited.append(goal)
                    path = self._reconstruct(nodes, parents, len(nodes) - 1)
                    return PlanResult(path, True, i + 1, visited)

        # max_iters 안에 경로를 찾지 못함.
        return PlanResult([], False, self.max_iters, visited)

    def _sample(self, world: World, goal: Point3, rng: random.Random) -> Point3:
        # 목표 또는 월드 범위 내 균일 샘플 중 하나를 선택.
        if rng.random() < self.goal_sample_rate:
            return goal
        return (
            rng.uniform(world.bounds_min[0], world.bounds_max[0]),
            rng.uniform(world.bounds_min[1], world.bounds_max[1]),
            rng.uniform(world.bounds_min[2], world.bounds_max[2]),
        )

    def _nearest(self, nodes: List[Point3], target: Point3) -> int:
        # 단순 선형 최근접 탐색.
        best_idx = 0
        best_dist = float("inf")
        for i, p in enumerate(nodes):
            d = distance(p, target)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _neighbors(self, nodes: List[Point3], target: Point3, radius: float) -> List[int]:
        # 고정 반경 안의 이웃을 수집.
        neighbors: List[int] = []
        for i, p in enumerate(nodes):
            if distance(p, target) <= radius:
                neighbors.append(i)
        if not neighbors:
            neighbors.append(self._nearest(nodes, target))
        return neighbors

    def _steer(self, src: Point3, dst: Point3, step: float) -> Point3:
        # src에서 dst 방향으로 최대 step만큼 이동.
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
        # 부모 링크를 따라가며 경로를 구성.
        path: List[Point3] = []
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        return path
