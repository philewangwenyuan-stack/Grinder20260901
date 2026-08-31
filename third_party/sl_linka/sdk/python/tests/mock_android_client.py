import os
import socket
import sys
import time

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sl_link.frame.frame import SlFrame, SlFrameParser, SL_PROTOCOL_VERSION
from sl_link.message_gen import sl_link_pb2 as pb


def send_frame(sock, seq, msg_id, comp_id, payload):
    frame = SlFrame(
        version=SL_PROTOCOL_VERSION,
        flags=0,
        seq=seq,
        ack_seq=0,
        src_id=pb.DEVICE_APP,
        dst_id=pb.DEVICE_LOWER,
        comp_id=comp_id,
        msg_id=msg_id,
        payload=payload,
    )
    sock.sendall(frame.pack())


def main(addr=("127.0.0.1", 8002)):
    parser = SlFrameParser()
    seq = 1

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(5.0)
        s.connect(addr)

        wifi = pb.WifiConfig(ssid="demo-ssid", password="demo-password")
        send_frame(s, seq, pb.MSG_ID_WIFI_CONFIG, pb.COMP_WIFI, wifi.SerializeToString())
        seq += 1

        read_req = pb.SettingsReadRequest(read_chassis=True, read_map=True)
        send_frame(s, seq, pb.MSG_ID_SETTINGS_READ_REQUEST, pb.COMP_SETTINGS, read_req.SerializeToString())
        seq += 1

        control = pb.ControlCommand()
        control.chassis_power.enabled = True
        send_frame(s, seq, pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, control.SerializeToString())
        seq += 1

        manual = pb.ControlCommand()
        manual.manual_drive.motion = pb.MANUAL_MOTION_FORWARD_LEFT
        manual.manual_drive.speed_ratio = 0.7
        send_frame(s, seq, pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, manual.SerializeToString())
        seq += 1

        lift = pb.ControlCommand()
        lift.disc_lift.command = pb.DISC_LIFT_CMD_DOWN
        send_frame(s, seq, pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, lift.SerializeToString())
        seq += 1

        light = pb.ControlCommand()
        light.lighting.enabled = True
        send_frame(s, seq, pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, light.SerializeToString())
        seq += 1

        cam = pb.CameraFrameRequest(snapshot=True, max_chunk_size=512)
        send_frame(s, seq, pb.MSG_ID_CAMERA_FRAME_REQUEST, pb.COMP_MEDIA, cam.SerializeToString())
        seq += 1

        map_req = pb.MapRequest(snapshot=True, max_chunk_size=512)
        send_frame(s, seq, pb.MSG_ID_MAP_REQUEST, pb.COMP_MEDIA, map_req.SerializeToString())

        deadline = time.time() + 5.0
        while time.time() < deadline:
            data = s.recv(4096)
            if not data:
                break

            for parsed in parser.parse(data):
                if parsed.msg_id == pb.MSG_ID_WIFI_STATUS_REPORT:
                    report = pb.WifiStatusReport()
                    report.ParseFromString(parsed.payload)
                    print(f"wifi result={report.result} message={report.message}")
                elif parsed.msg_id == pb.MSG_ID_SETTINGS_READ_RESPONSE:
                    report = pb.SettingsReadResponse()
                    report.ParseFromString(parsed.payload)
                    print(
                        f"settings run_speed={report.chassis.run_speed} "
                        f"disc_rpm={report.chassis.disc_speed_rpm} "
                        f"obstacles={len(report.map.obstacle_regions)}"
                    )
                elif parsed.msg_id == pb.MSG_ID_CONTROL_COMMAND_RESPONSE:
                    report = pb.ControlCommandResponse()
                    report.ParseFromString(parsed.payload)
                    print(f"control result={report.result} message={report.message}")
                elif parsed.msg_id == pb.MSG_ID_CAMERA_FRAME_CHUNK:
                    report = pb.CameraFrameChunk()
                    report.ParseFromString(parsed.payload)
                    print(f"camera frame={report.frame_id} bytes={len(report.data)}")
                elif parsed.msg_id == pb.MSG_ID_MAP_CHUNK:
                    report = pb.MapChunk()
                    report.ParseFromString(parsed.payload)
                    print(f"map id={report.map_id} bytes={len(report.data)}")
                elif parsed.msg_id == pb.MSG_ID_DEVICE_STATUS_REPORT:
                    report = pb.DeviceStatusReport()
                    report.ParseFromString(parsed.payload)
                    print(
                        f"status left={report.left_wheel_speed:.2f} right={report.right_wheel_speed:.2f} "
                        f"mode={report.work_mode} light={report.light_enabled} chassis={report.chassis_enabled}"
                    )


if __name__ == "__main__":
    main()
