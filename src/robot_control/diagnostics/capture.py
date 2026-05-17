"""Capture helper that fetches a JPEG frame from a running camera_service.

The camera_service publishes Observations over a PUB socket on port N
(default 5556). When extended with the diagnostics REP socket (port N+1),
it can also serve JPEG-encoded frames on demand. This helper is the client
side of that REP socket — used by run_namo.py to grab scene_before /
scene_after snapshots without disrupting the live observation stream.
"""

from __future__ import annotations

from typing import Optional

import zmq

# Default kind matches what most callers want — annotated frame with markers
# already drawn, which is more useful for human inspection than raw.
_DEFAULT_KIND = b"vis"


def _derive_frame_addr(pub_addr: str) -> str:
    """Translate a PUB address (tcp://host:N) to its REP companion (port N+1).

    Accepts:
      tcp://host:5556       → tcp://host:5557
      tcp://*:5556          → tcp://localhost:5557 (REQ side connects, not binds)
    """
    if not pub_addr.startswith("tcp://"):
        raise ValueError(f"only tcp:// URLs supported, got {pub_addr!r}")
    rest = pub_addr[len("tcp://"):]
    host, _, port = rest.rpartition(":")
    if not host or not port:
        raise ValueError(f"could not parse host:port from {pub_addr!r}")
    # PUB binds on "tcp://*:...", but the REQ side needs a connectable host.
    if host == "*":
        host = "localhost"
    return f"tcp://{host}:{int(port) + 1}"


def request_camera_frame(
    camera_service_addr: str,
    kind: str = "vis",
    timeout_sec: float = 2.0,
) -> Optional[bytes]:
    """Ask the camera_service for a JPEG-encoded frame.

    Args:
        camera_service_addr: PUB address used by the camera service
            (e.g. "tcp://localhost:5556"). The REP socket is assumed to live
            at port+1.
        kind: "vis" for an ArUco-annotated frame, "raw" for the underlying
            BGR camera frame.
        timeout_sec: Maximum time to wait for a reply.

    Returns:
        JPEG bytes on success, or None if the camera_service is unavailable,
        does not implement the REP socket, or times out. Callers must handle
        None — the runtime should never fail because diagnostics is best-
        effort.
    """
    try:
        req_addr = _derive_frame_addr(camera_service_addr)
    except ValueError as exc:
        print(f"[Diagnostics] capture: bad camera_service address: {exc}", flush=True)
        return None

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SNDTIMEO, int(timeout_sec * 1000))
    sock.setsockopt(zmq.RCVTIMEO, int(timeout_sec * 1000))
    try:
        sock.connect(req_addr)
        sock.send(kind.encode() if isinstance(kind, str) else kind)
        try:
            data = sock.recv()
        except zmq.Again:
            return None
        if not data:
            return None
        return bytes(data)
    except zmq.ZMQError as exc:
        print(f"[Diagnostics] capture: ZMQ error talking to {req_addr}: {exc}",
              flush=True)
        return None
    finally:
        try:
            sock.close(linger=0)
        except Exception:
            pass
