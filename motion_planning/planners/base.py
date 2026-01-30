from __future__ import annotations

from dataclasses import dataclass
from typing import List, Protocol

from ..types import Point3, World


@dataclass(frozen=True)
class PlanResult:
    path: List[Point3]
    success: bool
    iterations: int
    visited: List[Point3] | None = None


class Planner(Protocol):
    def plan(self, world: World, start: Point3, goal: Point3) -> PlanResult:
        ...
