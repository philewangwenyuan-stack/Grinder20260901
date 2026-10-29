"""Local socket backpressure only; never contacts robot hardware."""
from pathlib import Path
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from grinder_scheduler.socket_io import recv_poll, send_with_deadline


class SocketIOTest(unittest.TestCase):
    def pair(self):
        # Set the TCP receive window before the handshake (Windows socketpair uses
        # loopback fast-path buffering and cannot reliably simulate backpressure).
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
            listener.bind(('127.0.0.1', 0))
            listener.listen(1)
            sender = socket.create_connection(listener.getsockname(), timeout=2)
            receiver, _ = listener.accept()
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)
        sender.setblocking(False)
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        receiver.settimeout(2)
        return sender, receiver

    @unittest.skipIf(sys.platform == 'win32', 'Validate TCP backpressure on target Linux; Windows loopback buffers full writes')
    def test_receive_poll_does_not_abort_a_backpressured_map_after_100ms(self):
        sender, receiver = self.pair()
        payload = bytes(range(256)) * 4096
        failures = []
        def write():
            try: send_with_deadline(sender, payload, 2., lambda: True)
            except BaseException as e: failures.append(e)
        worker = threading.Thread(target=write)
        worker.start()
        # Receive polling on this same socket must not set its write deadline.
        self.assertIsNone(recv_poll(sender, 128, .15))
        self.assertTrue(worker.is_alive())
        self.assertEqual(0., sender.gettimeout())
        received = bytearray()
        while len(received) < len(payload):
            received.extend(receiver.recv(65536))
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual([], failures)
        self.assertEqual(payload, received)

    @unittest.skipIf(sys.platform == 'win32', 'Validate TCP backpressure on target Linux')
    def test_write_deadline_is_bounded_under_permanent_backpressure(self):
        sender, _ = self.pair()
        start = time.monotonic()
        with self.assertRaises(socket.timeout):
            send_with_deadline(sender, b'x' * (8 * 1024 * 1024), .12, lambda: True)
        self.assertLess(time.monotonic() - start, .6)

    @unittest.skipIf(sys.platform == 'win32', 'Validate TCP backpressure on target Linux')
    def test_session_cancellation_interrupts_backpressured_write(self):
        sender, _ = self.pair()
        cancelled = threading.Event()
        failures = []
        def write():
            try: send_with_deadline(sender, b'x' * (8 * 1024 * 1024), 2., lambda: not cancelled.is_set())
            except BaseException as e: failures.append(e)
        worker = threading.Thread(target=write)
        worker.start()
        time.sleep(.05)
        cancelled.set()
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(1, len(failures))
        self.assertIsInstance(failures[0], ConnectionError)

    def test_readable_and_disconnected_socket(self):
        sender, receiver = self.pair()
        receiver.sendall(b'hello')
        self.assertEqual(b'hello', recv_poll(sender, 10))
        receiver.close()
        self.assertEqual(b'', recv_poll(sender, 10))


if __name__ == '__main__':
    unittest.main()
