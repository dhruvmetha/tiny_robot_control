"""robot_control must not override namo_cpp's canonical opening-bar sample size.

The "region opened" test is a *fraction* of the sampled goal points, so the
sample size is half the criterion. namo_cpp grades at 20% of 100 points.
robot_control used to pass its own default of 10 at three separate layers
(the CLI flag, NAMOPlanner, and NAMOPlanBridge), which silently turned that
into "2 of 10 points" — the same fraction over a sample small enough that two
lucky cells call a boundary open.

All three now default to None, which the service reads as "use the canonical
value". These tests pin that, because the failure is invisible: nothing errors,
the run just grades itself more loosely than the numbers it is compared against.
"""

import inspect

from robot_control.planner.namo_bridge import NAMOPlanBridge
from robot_control.planner.namo_planner import NAMOPlanner


def _default(func, name):
    return inspect.signature(func).parameters[name].default


def test_planner_defers_the_sample_size():
    assert _default(NAMOPlanner.__init__, "goals_per_region") is None


def test_bridge_defers_the_sample_size():
    assert _default(NAMOPlanBridge.plan, "goals_per_region") is None
