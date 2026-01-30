from dataclasses import dataclass
from typing import Iterable, List, Tuple


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Box:
    min_corner: Point3
    max_corner: Point3

    def contains(self, p: Point3) -> bool:
        return (
            self.min_corner[0] <= p[0] <= self.max_corner[0]
            and self.min_corner[1] <= p[1] <= self.max_corner[1]
            and self.min_corner[2] <= p[2] <= self.max_corner[2]
        )


@dataclass(frozen=True)
class Sphere:
    center: Point3
    radius: float

    def contains(self, p: Point3) -> bool:
        return distance(self.center, p) <= self.radius


Obstacle = Box | Sphere


@dataclass(frozen=True)
class World:
    bounds_min: Point3
    bounds_max: Point3
    obstacles: List[Obstacle]

    def in_bounds(self, p: Point3) -> bool:
        return (
            self.bounds_min[0] <= p[0] <= self.bounds_max[0]
            and self.bounds_min[1] <= p[1] <= self.bounds_max[1]
            and self.bounds_min[2] <= p[2] <= self.bounds_max[2]
        )

    def collides(self, p: Point3) -> bool:
        for obs in self.obstacles:
            if obs.contains(p):
                return True
        return False

    def path_collides(self, a: Point3, b: Point3) -> bool:
        for obs in self.obstacles:
            if isinstance(obs, Box):
                if segment_intersects_aabb(a, b, obs):
                    return True
            else:
                if segment_intersects_sphere(a, b, obs):
                    return True
        return False


def segment_intersects_aabb(a: Point3, b: Point3, box: Box) -> bool:
    # Slab method for segment vs axis-aligned bounding box.
    tmin = 0.0
    tmax = 1.0
    for i in range(3):
        da = b[i] - a[i]
        if abs(da) < 1e-9:
            if a[i] < box.min_corner[i] or a[i] > box.max_corner[i]:
                return False
            continue
        inv = 1.0 / da
        t1 = (box.min_corner[i] - a[i]) * inv
        t2 = (box.max_corner[i] - a[i]) * inv
        t_low = min(t1, t2)
        t_high = max(t1, t2)
        tmin = max(tmin, t_low)
        tmax = min(tmax, t_high)
        if tmin > tmax:
            return False
    return True


def segment_intersects_sphere(a: Point3, b: Point3, sphere: Sphere) -> bool:
    d = sub(b, a)
    f = sub(a, sphere.center)
    a_coeff = dot(d, d)
    if a_coeff < 1e-12:
        return distance(a, sphere.center) <= sphere.radius
    b_coeff = 2.0 * dot(f, d)
    c_coeff = dot(f, f) - sphere.radius * sphere.radius
    disc = b_coeff * b_coeff - 4.0 * a_coeff * c_coeff
    if disc < 0.0:
        return False
    sqrt_disc = disc ** 0.5
    t1 = (-b_coeff - sqrt_disc) / (2.0 * a_coeff)
    t2 = (-b_coeff + sqrt_disc) / (2.0 * a_coeff)
    return (0.0 <= t1 <= 1.0) or (0.0 <= t2 <= 1.0)


def distance(a: Point3, b: Point3) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def clamp_point(p: Point3, bounds_min: Point3, bounds_max: Point3) -> Point3:
    return (
        min(max(p[0], bounds_min[0]), bounds_max[0]),
        min(max(p[1], bounds_min[1]), bounds_max[1]),
        min(max(p[2], bounds_min[2]), bounds_max[2]),
    )


def add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Point3, s: float) -> Point3:
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def normalize(a: Point3) -> Point3:
    length = distance(a, (0.0, 0.0, 0.0))
    if length < 1e-9:
        return (0.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)
