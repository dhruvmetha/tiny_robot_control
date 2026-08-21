"""Loading captured scenes on a machine that did not capture them.

The scenes under real_test_envs bake an absolute include:

    <include file="/home/dhruv/.../namo_cpp/test_xml/.../little_car.xml"/>

which is the path on the box that captured them. Anywhere else MuJoCo fails to
open the scene, and the checkout is not even called namo_cpp everywhere. Rather
than rewrite the committed scenes and re-break them the next time someone
captures on a different box, load a copy with the include pointed at this
machine's checkout, resolved through the same loader the runtime uses.
"""

from __future__ import annotations

from contextlib import contextmanager
import re
import tempfile
from pathlib import Path
from typing import Iterator

from robot_control.planner.namo_binding_loader import resolve_namo_cpp_dir

# Matches the include whatever absolute prefix it carries. The tail from
# test_xml/ onward is the part that is stable across checkouts.
_ABSOLUTE_INCLUDE = re.compile(r'(<include\s+file=")([^"]*?)(test_xml/[^"]+)(")')


def portable_scene(scene_path: Path, out_dir: Path) -> Path:
    """Copy a captured scene with its include repointed at this checkout.

    Returns the copy's path. A scene with no absolute include is copied
    unchanged, so callers do not have to know which kind they hold.
    """
    namo_cpp_dir = resolve_namo_cpp_dir(scene_path)
    text = Path(scene_path).read_text()

    def _repoint(match: re.Match) -> str:
        head, _stale_prefix, tail, quote = match.groups()
        return f"{head}{namo_cpp_dir}/{tail}{quote}"

    out = Path(out_dir) / Path(scene_path).name
    out.write_text(_ABSOLUTE_INCLUDE.sub(_repoint, text))
    return out


@contextmanager
def portable_scene_path(scene_path: Path) -> Iterator[Path]:
    """Yield a loadable temporary copy of a captured scene.

    Scenes without the captured-car include stay at their original path, which
    preserves any unrelated relative assets. Keep the context open for as long
    as a consumer may reopen the XML; replay does so after running the push.
    """
    scene_path = Path(scene_path)
    if _ABSOLUTE_INCLUDE.search(scene_path.read_text()) is None:
        yield scene_path
        return

    with tempfile.TemporaryDirectory(prefix="robot_control_scene_") as tmp_dir:
        yield portable_scene(scene_path, Path(tmp_dir))
