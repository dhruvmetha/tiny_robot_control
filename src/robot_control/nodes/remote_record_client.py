"""Client that sends record_start / record_stop commands to a remote camera_service.

The camera_service publishes observations on tcp://*:PORT (typically 5556) and
exposes a separate REP socket on tcp://*:(PORT+1) for on-demand frame requests
and recording control. This client uses that REP socket to start/stop recording
on the service side; the service writes the MP4 itself (it has the frames).

Use case: bracket a real-robot execution with recording so the resulting MP4
exactly matches the time window the runtime was active.
"""

from __future__ import annotations

from typing import Optional
from urllib.parse import urlparse

import zmq


class RemoteRecordClient:
    """Thin client for record_start / record_stop on camera_service REP socket."""

    def __init__(self, service_address: str, recv_timeout_ms: int = 2000):
        """
        Args:
            service_address: Address of the camera_service PUB endpoint, e.g.
                'tcp://localhost:5556'. The REP control socket is assumed to
                be on (PORT + 1), per the camera_service convention.
            recv_timeout_ms: How long to wait for a reply before giving up.
        """
        self._service_address = service_address
        self._control_address = self._derive_control_address(service_address)
        self._recv_timeout_ms = recv_timeout_ms
        self._ctx: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None

    @staticmethod
    def _derive_control_address(pub_address: str) -> str:
        """Translate the PUB address (port N) into the REP control address (port N+1)."""
        parsed = urlparse(pub_address)
        if parsed.scheme != "tcp" or parsed.hostname is None or parsed.port is None:
            raise ValueError(
                f"RemoteRecordClient expects a tcp://host:port address, got: {pub_address!r}"
            )
        return f"tcp://{parsed.hostname}:{parsed.port + 1}"

    def _ensure_socket(self) -> None:
        if self._socket is not None:
            return
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.REQ)
        # Don't block forever if the server is gone — bound recv attempts.
        self._socket.setsockopt(zmq.RCVTIMEO, self._recv_timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(self._control_address)

    def _request(self, payload: bytes) -> Optional[bytes]:
        """Send a one-shot REQ/REP. Returns None on timeout/error."""
        self._ensure_socket()
        try:
            assert self._socket is not None
            self._socket.send(payload)
            return self._socket.recv()
        except zmq.Again:
            print(f"[RemoteRecordClient] Timeout waiting for {payload!r}", flush=True)
            self._reset_socket()
            return None
        except Exception as exc:
            print(f"[RemoteRecordClient] Error sending {payload!r}: {exc!r}", flush=True)
            self._reset_socket()
            return None

    def _reset_socket(self) -> None:
        # REQ socket state machine breaks after a failed recv; close and rebuild.
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:
                pass
        self._socket = None

    def start(self, output_dir: str, filename: Optional[str] = None) -> Optional[str]:
        """Ask camera_service to begin recording into output_dir.

        Args:
            output_dir: Directory to save the MP4 in.
            filename: Optional filename (without extension). If None, the
                service uses a timestamp.

        Returns the MP4 path the service is writing to, or None on failure.
        """
        if filename:
            payload = f"record_start:{output_dir}:{filename}".encode("utf-8")
        else:
            payload = f"record_start:{output_dir}".encode("utf-8")
        reply = self._request(payload)
        if reply is None or reply == b"":
            print(f"[RemoteRecordClient] record_start failed (no reply)", flush=True)
            return None
        path = reply.decode("utf-8", errors="ignore")
        print(f"[RemoteRecordClient] Recording started → {path}", flush=True)
        return path

    def stop(self) -> Optional[str]:
        """Ask camera_service to stop recording and finalize the MP4.

        Returns the saved file path, or None if nothing was recording.
        """
        reply = self._request(b"record_stop")
        if reply is None:
            return None
        path = reply.decode("utf-8", errors="ignore")
        if not path:
            return None
        print(f"[RemoteRecordClient] Recording stopped → {path}", flush=True)
        return path

    def close(self) -> None:
        self._reset_socket()
