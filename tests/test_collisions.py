from motion_planning import Box, Sphere, World


def test_path_collides_with_box():
    world = World(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(4.0, 4.0, 4.0),
        obstacles=[Box((1.0, 1.0, 1.0), (2.0, 2.0, 2.0))],
    )
    assert world.path_collides((0.0, 0.0, 0.0), (3.0, 3.0, 3.0)) is True
    assert world.path_collides((0.0, 0.0, 0.0), (0.0, 3.0, 0.0)) is False


def test_path_collides_with_sphere():
    world = World(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(4.0, 4.0, 4.0),
        obstacles=[Sphere((1.5, 0.0, 0.0), 0.5)],
    )
    assert world.path_collides((0.0, 0.0, 0.0), (3.0, 0.0, 0.0)) is True
    assert world.path_collides((0.0, 1.0, 0.0), (3.0, 1.0, 0.0)) is False
