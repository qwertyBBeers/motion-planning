import random
from dataclasses import dataclass
from typing import List, Tuple

from .types import Box, Point3, Sphere, World


@dataclass(frozen=True)
class RandomWorld:
    bounds_min: Point3 = (0.0, 0.0, 0.0)
    bounds_max: Point3 = (10.0, 10.0, 10.0)
    obstacle_count: int = 15
    obstacle_size_range: Tuple[float, float] = (0.5, 2.0)
    sphere_ratio: float = 0.3
    max_tries: int = 200

    def generate(self, rng: random.Random | None = None) -> World:
        rng = rng or random.Random()
        obstacles: List[Box | Sphere] = []
        for _ in range(self.obstacle_count):
            for _ in range(self.max_tries):
                size = rng.uniform(*self.obstacle_size_range)
                if rng.random() < self.sphere_ratio:
                    radius = size * 0.5
                    x = rng.uniform(self.bounds_min[0] + radius, self.bounds_max[0] - radius)
                    y = rng.uniform(self.bounds_min[1] + radius, self.bounds_max[1] - radius)
                    z = rng.uniform(self.bounds_min[2] + radius, self.bounds_max[2] - radius)
                    obstacles.append(Sphere((x, y, z), radius))
                else:
                    x = rng.uniform(self.bounds_min[0], self.bounds_max[0] - size)
                    y = rng.uniform(self.bounds_min[1], self.bounds_max[1] - size)
                    z = rng.uniform(self.bounds_min[2], self.bounds_max[2] - size)
                    obstacles.append(Box((x, y, z), (x + size, y + size, z + size)))
                break
        return World(self.bounds_min, self.bounds_max, obstacles)

    def sample_free_point(self, world: World, rng: random.Random | None = None) -> Point3:
        rng = rng or random.Random()
        for _ in range(self.max_tries):
            p = (
                rng.uniform(world.bounds_min[0], world.bounds_max[0]),
                rng.uniform(world.bounds_min[1], world.bounds_max[1]),
                rng.uniform(world.bounds_min[2], world.bounds_max[2]),
            )
            if not world.collides(p):
                return p
        # Fallback: return a corner if random sampling fails.
        return world.bounds_min

    def sample_line_segment(
        self, world: World, rng: random.Random | None = None
    ) -> tuple[Point3, Point3]:
        rng = rng or random.Random()
        return self.sample_free_point(world, rng), self.sample_free_point(world, rng)
