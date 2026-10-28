"""Real paired TCP admission and safety tests; never connects to ROS or hardware."""
import socket
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'catkin_ws/src/grinder_scheduler/src'))
from grinder_scheduler.channel_sessions import ChannelSessions, BulkBudget
from grinder_scheduler.manual_delivery import ManualDelivery, is_manual_stop
from grinder_scheduler.sl_linka_adapter import SlLinkAServer


class SessionTest(unittest.TestCase):
    def test_exclusive_recovery_and_stale_finish(self):
        clock = [0.0]
        sessions = ChannelSessions(clock=lambda: clock[0])
        a, b, control, replacement = [object() for _ in range(4)]
        okay, token, _ = sessions.attach('data', a)
        self.assertTrue(okay)
        self.assertFalse(sessions.attach('data', b)[0])
        self.assertFalse(sessions.attach('control', b, 'wrong')[0])
        self.assertTrue(sessions.attach('control', control, token)[0])
        self.assertTrue(sessions.attach('data', replacement, token)[0])
        sessions.detach(a)
        self.assertTrue(sessions.valid('data', replacement))
        self.assertTrue(sessions.valid('control', control))
        clock[0] = 3.01
        self.assertCountEqual([replacement, control], sessions.expire())
        self.assertFalse(sessions.attach('data', a, token)[0])
        self.assertTrue(sessions.attach('data', b)[0])

    def test_legacy_readonly_cannot_bind_control(self):
        sessions = ChannelSessions()
        accepted, token, _ = sessions.attach('data', object(), legacy=True)
        self.assertTrue(accepted)
        self.assertFalse(sessions.attach('control', object(), token)[0])

    def test_bulk_is_bounded_and_cancelable(self):
        budget = BulkBudget(rate=8192, manual_rate=4096, burst=4096)
        start = time.monotonic()
        budget.consume(8192, lambda: True)
        self.assertGreaterEqual(time.monotonic() - start, .45)
        with self.assertRaises(ConnectionError):
            budget.consume(1, lambda: False)


class PairedTcpTest(unittest.TestCase):
    def setUp(self):
        self.addCleanup(patch.stopall)
        patch('grinder_scheduler.sl_linka_adapter.rospy', None).start()
        owner = self
        self.outputs = []
        self.commands = []
        self.generation = 0
        self.captured = threading.Event()
        self.entered, self.release = threading.Event(), threading.Event()
        self.addCleanup(self.release.set)
        class Handler:
            delivery = ManualDelivery()
            def begin_manual_session(self, session): self.delivery.begin(session)
            def cancel_manual_session(self, session):
                if self.delivery.invalidate(session): owner.outputs.append(0)
            def capture_control_generation(self):
                owner.captured.set()
                return owner.generation
            def capture_stop_generation(self, session):
                self.cancel_manual_session(session)
                owner.generation += 1
                return owner.generation
            def is_control_session_current(self, session): return self.delivery.session is session
            def capture_manual_ticket(self, session, request, received):
                stop = is_manual_stop(request.manual_drive, owner.server.pb)
                return self.delivery.capture(session, received, .5, stop), stop
            def handle_control_command(self, payload, parsed_request=None, manual_ticket=None, connection_valid=None, **kw):
                pb = owner.server.pb
                if connection_valid() and (manual_ticket is None or self.delivery.valid(manual_ticket)):
                    if parsed_request.HasField('manual_drive'):
                        owner.outputs.append(parsed_request.manual_drive.speed_ratio)
                response = pb.ControlCommandResponse(result=pb.RESULT_SUCCESS)
                return response.SerializeToString(), pb.MSG_ID_CONTROL_COMMAND_RESPONSE, pb.COMP_CONTROL
            def handle_task_command(self, payload, **kw):
                pb = owner.server.pb
                valid = kw.get('control_generation') in (None, owner.generation)
                if valid: owner.commands.append(pb.TaskCommand.FromString(payload).command)
                return pb.TaskCommandResponse(result=pb.RESULT_SUCCESS if valid else pb.RESULT_FAILED).SerializeToString(), pb.MSG_ID_TASK_COMMAND_RESPONSE, pb.COMP_SCHEDULER
            def handle_settings_read_request(self, payload):
                owner.entered.set()
                owner.release.wait(2)
                pb = owner.server.pb
                return b'', pb.MSG_ID_SETTINGS_READ_RESPONSE, pb.COMP_SYSTEM
            def build_device_status_report(self): raise RuntimeError('No periodic status in fixture')
        self.server = SlLinkAServer(str(ROOT/'third_party/sl_linka/sdk/python'), '127.0.0.1', 0,
                                   Handler(), control_port=0)
        self.server.start()
        self.addCleanup(self.server.stop)
        self.pb = self.server.pb
        self.parsers = {}
        self.sequence = 0

    def connect(self, control=False):
        peer = socket.create_connection(self.server._listeners[int(control)].server_address, timeout=1)
        self.addCleanup(peer.close)
        self.parsers[peer] = self.server.SlFrameParser()
        return peer

    def send(self, peer, msg_id, message):
        self.sequence += 1
        frame = self.server.SlFrame(version=self.server.SL_PROTOCOL_VERSION, flags=0,
            seq=self.sequence, ack_seq=0, src_id=self.pb.DEVICE_APP, dst_id=self.pb.DEVICE_LOWER,
            comp_id=self.pb.COMP_CONTROL, msg_id=msg_id, payload=message.SerializeToString())
        peer.sendall(frame.pack())
        return self.sequence

    def receive(self, peer, ack):
        until = time.monotonic() + 1
        while time.monotonic() < until:
            for frame in self.parsers[peer].parse(peer.recv(8192)):
                if frame.ack_seq == ack: return frame
        self.fail('No response')

    def handshake(self, peer, control=False, token=''):
        pb = self.pb
        request = pb.ChannelSessionRequest(action=pb.CHANNEL_SESSION_BIND if control else pb.CHANNEL_SESSION_OPEN,
            role=pb.CHANNEL_ROLE_CONTROL if control else pb.CHANNEL_ROLE_DATA, token=token)
        ack = self.send(peer, pb.MSG_ID_CHANNEL_SESSION_REQUEST, request)
        return pb.ChannelSessionResponse.FromString(self.receive(peer, ack).payload)

    def pair(self):
        data = self.connect()
        response = self.handshake(data)
        self.assertEqual(self.pb.RESULT_SUCCESS, response.result)
        control = self.connect(True)
        self.assertEqual(self.pb.RESULT_SUCCESS, self.handshake(control, True, response.token).result)
        return data, control, response.token

    def moving(self):
        request = self.pb.ControlCommand()
        request.manual_drive.speed_ratio = .5
        request.manual_drive.motion = self.pb.MANUAL_MOTION_FORWARD
        return request

    def test_blocked_data_and_intruder_do_not_interrupt_control(self):
        data, control, token = self.pair()
        self.send(data, self.pb.MSG_ID_SETTINGS_READ_REQUEST, self.pb.SettingsReadRequest())
        self.assertTrue(self.entered.wait(1))
        foreign = self.connect()
        self.assertEqual(self.pb.RESULT_BUSY, self.handshake(foreign).result)
        probe = self.connect()
        probe.close()
        ack = self.send(control, self.pb.MSG_ID_CONTROL_COMMAND, self.moving())
        self.assertEqual(self.pb.MSG_ID_CONTROL_COMMAND_RESPONSE, self.receive(control, ack).msg_id)
        self.assertEqual(.5, self.outputs[-1])
        self.assertTrue(self.server.sessions.control_ready())

    def test_disconnect_stops_manual_and_recovery_rejects_old_finish(self):
        data, control, token = self.pair()
        ack = self.send(control, self.pb.MSG_ID_CONTROL_COMMAND, self.moving())
        self.receive(control, ack)
        replacement = self.connect(True)
        self.handshake(replacement, True, token)
        control.close()
        ack = self.send(replacement, self.pb.MSG_ID_CONTROL_COMMAND, self.moving())
        self.receive(replacement, ack)
        self.assertEqual(.5, self.outputs[-1])
        replacement.shutdown(socket.SHUT_RDWR)
        until = time.monotonic() + 1
        while self.outputs[-1] != 0 and time.monotonic() < until: time.sleep(.01)
        self.assertEqual(0, self.outputs[-1])

    def test_manual_timeout_does_not_release_data_owner(self):
        data, control, token = self.pair()
        ack = self.send(control, self.pb.MSG_ID_CONTROL_COMMAND, self.moving())
        self.receive(control, ack)
        time.sleep(.62)
        self.assertEqual(0, self.outputs[-1])
        self.assertFalse(self.server.sessions.control_ready())
        self.assertEqual(self.pb.RESULT_BUSY, self.handshake(self.connect()).result)

    def test_data_control_and_control_bulk_are_rejected(self):
        data, control, token = self.pair()
        ack = self.send(data, self.pb.MSG_ID_CONTROL_COMMAND, self.moving())
        response = self.pb.ControlCommandResponse.FromString(self.receive(data, ack).payload)
        self.assertEqual(self.pb.RESULT_UNSUPPORTED, response.result)
        ack = self.send(control, self.pb.MSG_ID_MAP_REQUEST, self.pb.MapRequest())
        response = self.pb.ChannelSessionResponse.FromString(self.receive(control, ack).payload)
        self.assertEqual(self.pb.RESULT_UNSUPPORTED, response.result)
        self.assertNotIn(.5, self.outputs)

    def test_stop_cancels_start_already_waiting_in_data_queue(self):
        data, control, _ = self.pair()
        self.send(data, self.pb.MSG_ID_SETTINGS_READ_REQUEST, self.pb.SettingsReadRequest())
        self.assertTrue(self.entered.wait(1))
        old_start = self.send(data, self.pb.MSG_ID_TASK_COMMAND, self.pb.TaskCommand(command=self.pb.TASK_CMD_START))
        # Let the data receive loop capture the old generation before STOP.
        self.assertTrue(self.captured.wait(1))
        stop = self.send(control, self.pb.MSG_ID_TASK_COMMAND, self.pb.TaskCommand(command=self.pb.TASK_CMD_STOP))
        self.receive(control, stop)
        self.release.set()
        response = self.pb.TaskCommandResponse.FromString(self.receive(data, old_start).payload)
        self.assertEqual(self.pb.RESULT_FAILED, response.result)
        self.assertEqual([self.pb.TASK_CMD_STOP], self.commands)


if __name__ == '__main__': unittest.main()
