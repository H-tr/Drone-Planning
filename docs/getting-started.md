# Getting Started

## Prerequisites

- **Linux** (x86_64, glibc 2.28+) on a CPU with AVX2 (Intel Haswell / AMD Zen or newer)
- **Python** 3.10–3.14

## Installation

Pre-built wheels are available for Python 3.10–3.14 on Linux x86_64. No local compilation required:

```bash
pip install drone-planning
```

## Verify installation

```python
from drone_planning.planning import create_planner
from drone_planning.x500 import HOME_JOINTS

planner = create_planner("x500")
goal = planner.sample_valid()
result = planner.plan(HOME_JOINTS.copy(), goal)
print(f"Planning {'succeeded' if result.success else 'failed'}")
```

## Building Wheels from Source

Wheels are built with [cibuildwheel](https://cibuildwheel.pypa.io) inside
the manylinux Docker images (configured under `[tool.cibuildwheel]` in
`pyproject.toml`). To build one locally:

```bash
pipx run cibuildwheel --platform linux --only cp312-manylinux_x86_64
```

The output goes to `wheelhouse/`. Requirements: Docker must be installed and running.

## Releasing

Pushing a `v*` tag runs `.github/workflows/release.yml`, which builds
manylinux_2_34 and manylinux_2_28 wheels for every supported Python and
publishes them to PyPI via Trusted Publishing:

```bash
# after bumping `version` in pyproject.toml on main
git tag v0.1.0
git push origin v0.1.0
```

Trusted Publishing needs a (pending) publisher registered once on pypi.org
for project `drone-planning`: owner `H-tr`, repository `Drone-Planning`,
workflow `release.yml`, environment `pypi`.

## Development Setup

For contributing or rebuilding C++ dependencies from source, use [pixi](https://pixi.sh):

```bash
git clone --recursive https://github.com/H-tr/Drone-Planning.git
cd Drone-Planning
bash scripts/setup.sh
```

Or manually:

```bash
git clone --recursive https://github.com/H-tr/Drone-Planning.git
cd Drone-Planning
pixi install
pixi run cricket-build
pixi run foam-build
```

`pixi run test` runs the test suite; `pixi run generate-fk` rebuilds the
robot model end to end (see [Robot Model](robot.md)).
