import sys
from pathlib import Path

import numpy as np

# The demo scene lives with the examples (examples/planning/_shared.py).
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples" / "planning"))

# VAMP checks motions every 1/64 of configuration distance
# (scripts/cricket_configs/x500.json, "resolution": 64).
CHECK_STEP = 1.0 / 64


def assert_collision_free(planner, configs: np.ndarray) -> None:
    """``configs`` is a densely sampled motion; it must be collision-free up
    to VAMP's motion resolution: any colliding stretch lies between two
    checked configurations, so it is shorter than one check step."""
    valid = planner.validate_batch(configs)
    assert valid[0] and valid[-1]
    bad = np.flatnonzero(~valid)
    for run in np.split(bad, np.flatnonzero(np.diff(bad) > 1) + 1) if bad.size else []:
        gap = configs[run[0] - 1 : run[-1] + 2]
        assert np.linalg.norm(np.diff(gap, axis=0), axis=1).sum() < CHECK_STEP
