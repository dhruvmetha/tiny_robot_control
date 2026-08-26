# Agent guide for robot_control

This file is a pointer, not a manual. The manual is [CLAUDE.md](CLAUDE.md)
(conventions, architecture, coding rules) and the doc index in
[README.md](README.md). Do not duplicate their content here; two copies
drift and the stale one wins.

Read before doing anything physical: [docs/REAL_ROBOT_TRIALS.md](docs/REAL_ROBOT_TRIALS.md).
It is the table runbook and every step in it was paid for.

Facts an agent needs on turn one, and nowhere else states this compactly:

- Python is `~/miniconda3/envs/namo312/bin/python`. The base conda python is
  3.8 and cannot import this package.
- Anything that touches the NAMO planner needs the environment sourced from
  the namo_cpp ROOT first: `cd ../namo_cpp && set -a && . env.robotlearning.sh
  && set +a`, then `export NAMO_REPO=<namo_cpp path>`. Unsourced, planning
  fails instantly with the real error hidden.
- The serial port `/dev/ttyACM0` has one owner. Check `fuser` before
  launching anything that drives the robot; a zombie run_namo garbles the
  radio silently.
- The camera service publishes multipart `[b"obs", json]` on tcp://5556;
  object theta comes from the tag, and tag-to-long-axis mapping goes through
  objects.yaml width/depth (bearing = theta + 90 when width > depth).
  Comparing raw tag yaw to a sheet bearing is 90 degrees wrong.
- Sim physics is frozen for the ICRA study. Never propose fitting sim mass
  or friction to hardware measurements; the divergence is a finding.
- Answer from the code, never from recall. Same rule as namo_cpp/AGENTS.md,
  same reason.
