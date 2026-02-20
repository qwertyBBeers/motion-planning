from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List

from ..settings.types import Point3, World, distance
from .base import PlanResult


@dataclass
class RRTPlanner:
    # 한 번에 확장할 최대 거리.
    step_size: float = 0.5
    # 샘플링 반복 횟수 상한.
    max_iters: int = 1000
    # 목표를 직접 샘플링할 확률(골 바이어스).
    goal_sample_rate: float = 0.05

    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        # 기본 입력 유효성 검사.
        if self.step_size <= 0.0 or self.max_iters <= 0:
            return PlanResult([], False, 0, [])
        if not world.in_bounds(start) or not world.in_bounds(goal):
            return PlanResult([], False, 0, [])
        if world.collides(start) or world.collides(goal):
            return PlanResult([], False, 0, [])

        # 트리 저장 구조: 노드와 부모 인덱스.
        rng = random.Random()
        nodes: List[Point3] = [start]
        parents: List[int] = [-1]
        visited: List[Point3] = []

        for i in range(self.max_iters):
            # 골 바이어스를 포함한 랜덤 샘플링.
            sample = self._sample(world, goal, rng)
            # 샘플 방향으로 트리를 확장.
            nearest_idx = self._nearest(nodes, sample)
            nearest = nodes[nearest_idx]
            new_pt = self._steer(nearest, sample, self.step_size)
            if distance(new_pt, nearest) < 1e-9:
                continue
            if world.collides(new_pt) or world.path_collides(nearest, new_pt):
                continue

            # 새 노드를 트리에 추가.
            nodes.append(new_pt)
            parents.append(nearest_idx)
            visited.append(new_pt)

            # 충분히 가까우면 목표로 직접 연결을 시도.
            if distance(new_pt, goal) <= self.step_size:
                if not world.path_collides(new_pt, goal) and not world.collides(goal):
                    nodes.append(goal)
                    parents.append(len(nodes) - 2)
                    visited.append(goal)
                    path = self._reconstruct(nodes, parents, len(nodes) - 1)
                    return PlanResult(path, True, i + 1, visited)

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
