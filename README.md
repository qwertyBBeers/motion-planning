# motion-toyproject

Minimal 3D motion-planning playground in Python. The goal is a clean, reusable
base for experimenting with algorithms (A*, RRT, RRG, MPC, etc.), collision
checking, and visualization.

## Goals
- Random 3D world generation with box/sphere obstacles
- Deterministic start/goal setup for reproducible demos
- 3D visualization utilities for debugging and demos
- Simple, reusable core you can extend in other projects

## Example output
Place a demo screenshot here:
```
docs/world_demo.png
```
Then it will show up below:
![World demo](docs/world_demo.png)

## Quick start
Run the world demo (A* path + visited nodes):
```bash
python examples/world_demo.py
```

## Project layout
- `motion_planning/` core library (world, collision, planners, visualization)
- `examples/` runnable demos
- `tests/` pytest-based tests
- `docs/` images and notes

## Notes
- Visualization requires `matplotlib` (and `numpy` for sphere rendering).
- The world is intentionally minimal; extend as needed.
