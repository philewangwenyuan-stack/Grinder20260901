"""Separate receive polling from bounded writes on a nonblocking TCP socket."""
import select
import socket
import time


def recv_poll(peer, size, poll_sec=0.1):
    try:
        readable, _, _ = select.select([peer], [], [], poll_sec)
        if not readable:
            return None
        return peer.recv(size)
    except (BlockingIOError, InterruptedError):
        return None
    except ValueError as error:
        raise ConnectionError("socket closed while receiving") from error


def send_with_deadline(peer, payload, timeout_sec, is_open):
    deadline = time.monotonic() + timeout_sec
    remaining = memoryview(payload)
    while remaining:
        if not is_open():
            raise ConnectionError("SL-LinkA session closed")
        wait = deadline - time.monotonic()
        if wait <= 0:
            raise socket.timeout("SL-LinkA send deadline exceeded")
        try:
            _, writable, _ = select.select([], [peer], [], min(wait, 0.1))
            if not writable:
                continue
            count = peer.send(remaining)
            if count == 0:
                raise ConnectionError("socket closed while sending")
            remaining = remaining[count:]
        except (BlockingIOError, InterruptedError):
            continue
        except ValueError as error:
            raise ConnectionError("socket closed while sending") from error
