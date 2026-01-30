from __future__ import annotations

from dataclasses import dataclass

from ..settings.types import Point3, World
from .base import PlanResult


@dataclass
class RRTPlanner:
    step_size: float = 0.5
    max_iters: int = 1000
    goal_sample_rate: float = 0.05

    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        return PlanResult([], False, 0, [])
