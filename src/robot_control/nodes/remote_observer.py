"""Remote observer node that receives Observations via ZeroMQ.

Connects to a camera_service publishing observations on a ZMQ PUB socket.
Deserializes and republishes into the local PyPubSub bus (Topics.SENSOR_VISION)
so that WorldState and the rest of the pipeline work unchanged.

Optionally enriches observations with object size/type info from objects.yaml,
since the camera service only provides pose data (marker detection).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, Optional

import zmq
from pubsub import pub

from robot_control.core.serialization import bytes_to_obs
from robot_control.core.topics import Topics
from robot_control.core.types import Observation


@dataclass
class ObjectSizeInfo:
    """Object physical properties for enriching camera-only observations."""
    width: float = 0.0    # cm
    depth: float = 0.0    # cm
    height: float = 0.0   # cm
    is_static: bool = False


class RemoteObserverNode:
    """
    ZMQ SUB client that receives serialized Observations from camera_service.

    The camera service publishes pose-only observations (marker detection).
    If object_sizes is provided, each received observation is enriched with
    width/depth/height/is_static before publishing to PyPubSub.

    Usage:
        node = RemoteObserverNode("tcp://localhost:5556", object_sizes={...})
        if node.start():
            obs = node.get()   # latest cached observation (enriched)
            # or subscribe to Topics.SENSOR_VISION via PyPubSub
        node.stop()
    """

    def __init__(
        self,
        address: str = "tcp://localhost:5556",
        object_sizes: Optional[Dict[str, ObjectSizeInfo]] = None,
    ) -> None:
        self._address = address
        self._object_sizes = object_sizes or {}
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._ctx: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._latest_obs: Optional[Observation] = None
        self._lock = threading.Lock()

    def start(self) -> bool:
        """Connect to camera service and start receiving. Returns True on success."""
        if self._running:
            return True

        try:
            self._ctx = zmq.Context()
            self._socket = self._ctx.socket(zmq.SUB)
            self._socket.connect(self._address)
            self._socket.setsockopt(zmq.SUBSCRIBE, b"obs")
            # DO NOT MOVE THIS ABOVE connect(). ZMQ only honours CONFLATE when
            # it is set first, so setting it here makes it a no-op, and the
            # no-op is what keeps this stream alive. CONFLATE drops all but the
            # last message and does not work with multipart at all: the camera
            # service sends [b"obs", payload], and a conflating subscriber
            # receives NOTHING. Measured 2026-08-22 on pyzmq 27.1.0 / libzmq
            # 4.3.5, same publisher both ways: set before connect delivered 0
            # of 10 messages, set after delivered all 10.
            #
            # So this line reads as an optimisation and is really an accident
            # that happens to be harmless. Tidying it into the documented order
            # kills every observation on the real robot, silently, because the
            # except Exception in _receive_loop swallows the starvation.
            #
            # The lag it was meant to fix is real. Fix it by draining to the
            # newest message with a non-blocking poll, not with CONFLATE.
            self._socket.setsockopt(zmq.CONFLATE, 1)
        except zmq.ZMQError as e:
            print(f"[RemoteObserver] Failed to connect to {self._address}: {e}")
            self._cleanup_socket()
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="RemoteObserver-Recv"
        )
        self._thread.start()
        print(f"[RemoteObserver] Connected to {self._address}")
        return True

    def stop(self) -> None:
        """Stop receiving and close socket."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._cleanup_socket()
        print("[RemoteObserver] Stopped")

    def get(self) -> Optional[Observation]:
        """Get latest cached observation (thread-safe)."""
        with self._lock:
            return self._latest_obs

    def _enrich(self, obs: Observation) -> Observation:
        """Stamp object size/type info onto pose-only observations."""
        if not self._object_sizes:
            return obs
        for name, obj in obs.objects.items():
            info = self._object_sizes.get(name)
            if info:
                obj.width = info.width
                obj.depth = info.depth
                obj.height = info.height
                obj.is_static = info.is_static
        return obs

    def _recv_loop(self) -> None:
        """Background thread: receive and republish observations."""
        poller = zmq.Poller()
        poller.register(self._socket, zmq.POLLIN)

        while self._running:
            events = dict(poller.poll(timeout=100))  # 100ms timeout
            if self._socket in events:
                try:
                    topic, data = self._socket.recv_multipart(zmq.NOBLOCK)
                    obs = self._enrich(bytes_to_obs(data))
                    with self._lock:
                        self._latest_obs = obs
                    pub.sendMessage(Topics.SENSOR_VISION, obs=obs)
                except zmq.ZMQError:
                    pass
                except Exception as e:
                    print(f"[RemoteObserver] Error deserializing: {e}")

    def _cleanup_socket(self) -> None:
        """Close ZMQ socket and context."""
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:
                pass
            self._socket = None
        if self._ctx is not None:
            try:
                self._ctx.term()
            except Exception:
                pass
            self._ctx = None

    @property
    def is_running(self) -> bool:
        """Check if receiver is running."""
        return self._running
