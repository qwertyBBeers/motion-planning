from motion_planning import AStarPlanner, Box, Sphere, World


def test_astar_empty_world_path_found():
    world = World(bounds_min=(0.0, 0.0, 0.0), bounds_max=(4.0, 4.0, 4.0), obstacles=[])
    planner = AStarPlanner(resolution=1.0, allow_diagonal=False)
    result = planner.plan(world, (1.0, 1.0, 1.0), (3.0, 3.0, 3.0))

    assert result.success is True
    assert result.path[0] == (1.0, 1.0, 1.0)
    assert result.path[-1] == (3.0, 3.0, 3.0)
    assert result.iterations > 0


def test_astar_start_or_goal_collides():
    world = World(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(3.0, 3.0, 3.0),
        obstacles=[Box((0.0, 0.0, 0.0), (3.0, 3.0, 3.0))],
    )
    planner = AStarPlanner(resolution=1.0)
    result = planner.plan(world, (1.0, 1.0, 1.0), (2.0, 2.0, 2.0))

    assert result.success is False
    assert result.path == []


def test_astar_avoids_obstacle():
    world = World(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(4.0, 4.0, 4.0),
        obstacles=[Sphere((2.0, 2.0, 2.0), 0.6)],
    )
    planner = AStarPlanner(resolution=1.0, allow_diagonal=False)
    result = planner.plan(world, (1.0, 1.0, 1.0), (3.0, 3.0, 3.0))

    assert result.success is True
    assert result.path[0] == (1.0, 1.0, 1.0)
    assert result.path[-1] == (3.0, 3.0, 3.0)
