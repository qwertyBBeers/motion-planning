# 격자 인덱스로 변환해주는 module
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from ..settings.types import Point3, World, clamp_point

GridIndex = Tuple[int, int, int]


@dataclass(frozen=True)
class Grid3D:
    world: World
    resolution: float
    
    # 격자 생성.
    def dims(self) -> GridIndex:
        dx = int((self.world.bounds_max[0] - self.world.bounds_min[0]) / self.resolution)
        dy = int((self.world.bounds_max[1] - self.world.bounds_min[1]) / self.resolution)
        dz = int((self.world.bounds_max[2] - self.world.bounds_min[2]) / self.resolution)
        return (dx + 1, dy + 1, dz + 1)

    # 임의의 좌표 p를 가장 가까운 격좌 좌표로 반올림함
    def snap(self, p: Point3) -> Point3:
        bx, by, bz = self.world.bounds_min
        rx = round((p[0] - bx) / self.resolution) * self.resolution + bx
        ry = round((p[1] - by) / self.resolution) * self.resolution + by
        rz = round((p[2] - bz) / self.resolution) * self.resolution + bz
        return clamp_point((rx, ry, rz), self.world.bounds_min, self.world.bounds_max)

    # 실제 좌표를 인덱스로 변환
    def to_index(self, p: Point3) -> GridIndex:
        return (
            int(round((p[0] - self.world.bounds_min[0]) / self.resolution)),
            int(round((p[1] - self.world.bounds_min[1]) / self.resolution)),
            int(round((p[2] - self.world.bounds_min[2]) / self.resolution)),
        )

    # 격자 인덱스 실제 좌표 복원
    def to_point(self, idx: GridIndex) -> Point3:
        return (
            self.world.bounds_min[0] + idx[0] * self.resolution,
            self.world.bounds_min[1] + idx[1] * self.resolution,
            self.world.bounds_min[2] + idx[2] * self.resolution,
        )
