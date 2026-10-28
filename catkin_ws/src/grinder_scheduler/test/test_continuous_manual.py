"""Real localhost TCP/Protobuf dispatch under blocked business; no ROS/motors."""
from pathlib import Path
import socket
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'catkin_ws/src/grinder_scheduler/src'))
from grinder_scheduler.manual_delivery import ManualDelivery, is_manual_stop
from grinder_scheduler.sl_linka_adapter import SlLinkAServer, _BoundedFifoWorker


class ContinuousManualTest(unittest.TestCase):
    packet_count=60
    send_interval=.02

    def test_sustained_manual_rx_bypasses_blocked_control_business(self):
        ros_patch = patch('grinder_scheduler.sl_linka_adapter.rospy', None)
        ros_patch.start()
        self.addCleanup(ros_patch.stop)
        entered,release=threading.Event(),threading.Event()
        output=[]
        blocked_seconds=self.packet_count*self.send_interval+2
        class Handler:
            delivery=ManualDelivery()
            def begin_manual_session(self,session):self.delivery.begin(session)
            def capture_control_generation(self):return 0
            def is_control_session_current(self,session):return self.delivery.session is session
            def capture_manual_ticket(self,session,request,received_at):
                stop=is_manual_stop(request.manual_drive,server.pb)
                return self.delivery.capture(session,received_at,.5,stop),stop
            def cancel_manual_session(self,session):
                if self.delivery.invalidate(session):output.append((time.monotonic(),0.))
            def handle_control_command(self,payload,parsed_request=None,control_generation=None,manual_ticket=None,connection_valid=None):
                request=parsed_request
                if request.HasField('lighting'):
                    entered.set();release.wait(blocked_seconds)
                elif request.HasField('manual_drive') and self.delivery.valid(manual_ticket):
                    output.append((time.monotonic(),request.manual_drive.speed_ratio))
                response=server.pb.ControlCommandResponse(result=server.pb.RESULT_SUCCESS,message='applied')
                return response.SerializeToString(),server.pb.MSG_ID_CONTROL_COMMAND_RESPONSE,server.pb.COMP_CONTROL
            def build_device_status_report(self):raise RuntimeError('periodic reports excluded')
        server=SlLinkAServer(str(ROOT/'third_party/sl_linka/sdk/python'),'127.0.0.1',0,Handler(), control_port=0)
        server.start()
        self.addCleanup(server.stop)
        self.addCleanup(release.set)
        data_connection=socket.create_connection(server._server.server_address,timeout=1)
        self.addCleanup(data_connection.close)
        def handshake(peer, control=False, token=''):
            pb=server.pb
            request=pb.ChannelSessionRequest(action=pb.CHANNEL_SESSION_BIND if control else pb.CHANNEL_SESSION_OPEN,
                role=pb.CHANNEL_ROLE_CONTROL if control else pb.CHANNEL_ROLE_DATA,token=token)
            frame=server.SlFrame(version=server.SL_PROTOCOL_VERSION,flags=0,seq=60000,ack_seq=0,
                src_id=pb.DEVICE_APP,dst_id=pb.DEVICE_LOWER,comp_id=pb.COMP_CONTROL,
                msg_id=pb.MSG_ID_CHANNEL_SESSION_REQUEST,payload=request.SerializeToString())
            peer.sendall(frame.pack())
            parser=server.SlFrameParser()
            while True:
                for reply in parser.parse(peer.recv(4096)):
                    if reply.msg_id==pb.MSG_ID_CHANNEL_SESSION_RESPONSE:
                        result=pb.ChannelSessionResponse.FromString(reply.payload)
                        self.assertEqual(pb.RESULT_SUCCESS,result.result)
                        return result.token
        token=handshake(data_connection)
        connection=socket.create_connection(server._listeners[1].server_address,timeout=1)
        self.addCleanup(connection.close)
        handshake(connection,True,token)
        sequence=[0]
        def send(request):
            sequence[0]+=1
            frame=server.SlFrame(version=server.SL_PROTOCOL_VERSION,flags=0,seq=sequence[0],ack_seq=0,
                src_id=server.pb.DEVICE_APP,dst_id=server.pb.DEVICE_LOWER,comp_id=server.pb.COMP_CONTROL,
                msg_id=server.pb.MSG_ID_CONTROL_COMMAND,payload=request.SerializeToString())
            connection.sendall(frame.pack())
        request=server.pb.ControlCommand();request.lighting.enabled=True;send(request)
        self.assertTrue(entered.wait(1))
        for _ in range(self.packet_count):
            request=server.pb.ControlCommand();request.manual_drive.speed_ratio=.5
            request.manual_drive.motion=server.pb.MANUAL_MOTION_FORWARD
            send(request);time.sleep(self.send_interval)
        self.assertFalse(release.is_set())
        self.assertGreaterEqual(len(output),int(self.packet_count*.66),'ordinary control blocked manual receipt/application')
        gaps=[b[0]-a[0] for a,b in zip(output,output[1:])]
        self.metrics=dict(sent=self.packet_count,applied=len(output),max_apply_gap_ms=max(gaps)*1000,
                          duration_s=output[-1][0]-output[0][0])
        self.assertLess(max(gaps),.5,'a 500ms watchdog gap despite continuous valid input')
        replies=[];parser=server.SlFrameParser();deadline=time.monotonic()+.5
        while not replies and time.monotonic()<deadline:
            replies.extend(parser.parse(connection.recv(64*1024)))
        self.assertTrue(any(frame.msg_id==server.pb.MSG_ID_CONTROL_COMMAND_RESPONSE and frame.ack_seq>1
                            for frame in replies),'manual ACK waited for unrelated business')
        request.manual_drive.speed_ratio=0.;send(request)
        deadline=time.monotonic()+.5
        while output[-1][1]!=0. and time.monotonic()<deadline:time.sleep(.005)
        self.assertEqual(0.,output[-1][1])
        time.sleep(.1)
        self.assertEqual(0.,output[-1][1],'old movement replayed after release')
        connection.close()

    def test_business_queue_preserves_order_and_rejects_overload(self):
        worker=_BoundedFifoWorker(2,'fifo-regression')
        entered,release=threading.Event(),threading.Event()
        self.addCleanup(lambda:worker.shutdown(wait=True))
        self.addCleanup(release.set)
        applied=[]
        self.assertTrue(worker.submit(lambda:(entered.set(),release.wait(2))))
        self.assertTrue(entered.wait(1))
        self.assertTrue(worker.submit(applied.append,1))
        self.assertTrue(worker.submit(applied.append,2))
        self.assertFalse(worker.submit(applied.append,3))
        release.set()
        deadline=time.monotonic()+1
        while len(applied)<2 and time.monotonic()<deadline:time.sleep(.005)
        self.assertEqual([1,2],applied)


if __name__=='__main__':unittest.main()
