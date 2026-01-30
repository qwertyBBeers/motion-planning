from motion_planning import AStarPlanner, RandomWorld, plot_world


def main() -> None:
    world_gen = RandomWorld(
        bounds_min=(0.0, 0.0, 0.0),
        bounds_max=(12.0, 12.0, 12.0),
        obstacle_count=18,
        obstacle_size_range=(0.8, 2.2),
    )
    world = world_gen.generate()
    start = (1.0, 1.0, 1.0)
    goal = (
        world.bounds_max[0] - 1.0,
        world.bounds_max[1] - 1.0,
        world.bounds_max[2] - 1.0,
    )

    print("start:", start)
    print("goal:", goal)
    print("obstacles:", len(world.obstacles))

    planner = AStarPlanner(resolution=0.5, allow_diagonal=True)
    result = planner.plan(world, start, goal)

    print("success:", result.success, "iters:", result.iterations)
    print("path length:", len(result.path))

    plot_world(world, start=start, goal=goal, path=result.path, visited=result.visited)


if __name__ == "__main__":
    main()
