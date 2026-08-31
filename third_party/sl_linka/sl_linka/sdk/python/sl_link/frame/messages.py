from ..message_gen import sl_link_pb2 as pb


class MessageHandler:
    def __init__(self, device_id: int):
        self.device_id = device_id
        self.seq = 0
        self.chassis_enabled = True
        self.light_enabled = False
        self.disc_lift_state = pb.DISC_LIFT_STATE_UP
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self.position_x = 1.25
        self.position_y = 2.50
        self.position_heading = 90.0
        self.last_manual_motion = pb.MANUAL_MOTION_STOP

        self.chassis_settings = pb.ChassisSettings()
        self.chassis_settings.run_speed = 0.8
        self.chassis_settings.disc_speed_rpm = 1200
        self.chassis_settings.disc_enabled = False
        self.chassis_settings.work_mode = pb.WORK_MODE_AUTO

        self.map_settings = pb.MapSettings()
        self.map_settings.vehicle_width = 0.72
        self.map_settings.vehicle_length = 1.10
        self.map_settings.default_path_spacing = 0.30
        self.map_settings.turn_radius = 0.45
        self.map_settings.overlap_ratio = 0.15
        self.map_settings.inflation_radius = 0.12
        self._add_demo_regions()

    def _add_demo_regions(self):
        obstacle = self.map_settings.obstacle_regions.add()
        obstacle.name = "obstacle_region_1"
        for x, y in ((1.0, 1.0), (1.4, 1.0), (1.4, 1.4), (1.0, 1.4)):
            pt = obstacle.points.add()
            pt.x = x
            pt.y = y

        work = self.map_settings.work_regions.add()
        work.name = "work_region_1"
        for x, y in ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)):
            pt = work.points.add()
            pt.x = x
            pt.y = y

    def get_next_seq(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return s

    def handle(self, frame):
        handlers = {
            pb.MSG_ID_WIFI_CONFIG: self.handle_wifi_config,
            pb.MSG_ID_SETTINGS_READ_REQUEST: self.handle_settings_read_request,
            pb.MSG_ID_SETTINGS_WRITE_REQUEST: self.handle_settings_write_request,
            pb.MSG_ID_CAMERA_FRAME_REQUEST: self.handle_camera_frame_request,
            pb.MSG_ID_MAP_REQUEST: self.handle_map_request,
            pb.MSG_ID_CONTROL_COMMAND: self.handle_control_command,
        }
        handler = handlers.get(frame.msg_id)
        if handler is None:
            return None, None
        return handler(frame)

    def handle_wifi_config(self, frame):
        req = pb.WifiConfig()
        req.ParseFromString(frame.payload)

        resp = pb.WifiStatusReport()
        resp.result = pb.WIFI_SUCCESS
        resp.message = f"Connected to {req.ssid}"
        return resp.SerializeToString(), pb.MSG_ID_WIFI_STATUS_REPORT

    def handle_settings_read_request(self, frame):
        req = pb.SettingsReadRequest()
        req.ParseFromString(frame.payload)

        resp = pb.SettingsReadResponse()
        resp.result = pb.RESULT_SUCCESS
        resp.message = "Settings read success"
        if req.read_chassis:
            resp.chassis.CopyFrom(self.chassis_settings)
        if req.read_map:
            resp.map.CopyFrom(self.map_settings)
        return resp.SerializeToString(), pb.MSG_ID_SETTINGS_READ_RESPONSE

    def handle_settings_write_request(self, frame):
        req = pb.SettingsWriteRequest()
        req.ParseFromString(frame.payload)

        if req.HasField("chassis"):
            self.chassis_settings.CopyFrom(req.chassis)
        if req.HasField("map"):
            self.map_settings.CopyFrom(req.map)

        resp = pb.SettingsWriteResponse()
        resp.result = pb.RESULT_SUCCESS
        resp.message = "Settings applied"
        resp.chassis.CopyFrom(self.chassis_settings)
        resp.map.CopyFrom(self.map_settings)
        return resp.SerializeToString(), pb.MSG_ID_SETTINGS_WRITE_RESPONSE

    def handle_camera_frame_request(self, frame):
        req = pb.CameraFrameRequest()
        req.ParseFromString(frame.payload)

        chunk = pb.CameraFrameChunk()
        chunk.frame_id = 101
        chunk.utc_time = 1710000100
        chunk.width = 640
        chunk.height = 480
        chunk.codec = pb.CAMERA_CODEC_JPEG
        chunk.chunk_index = 0
        chunk.total_chunks = 1
        chunk.data = b"demo-camera-frame-jpeg"
        return chunk.SerializeToString(), pb.MSG_ID_CAMERA_FRAME_CHUNK

    def handle_map_request(self, frame):
        req = pb.MapRequest()
        req.ParseFromString(frame.payload)

        chunk = pb.MapChunk()
        chunk.map_id = 7
        chunk.utc_time = 1710000200
        chunk.encoding = pb.MAP_ENCODING_JSON
        chunk.width = 200
        chunk.height = 120
        chunk.resolution = 0.05
        chunk.chunk_index = 0
        chunk.total_chunks = 1
        chunk.data = b'{"map":"demo","regions":2}'
        return chunk.SerializeToString(), pb.MSG_ID_MAP_CHUNK

    def handle_control_command(self, frame):
        req = pb.ControlCommand()
        req.ParseFromString(frame.payload)

        if req.HasField("disc_lift"):
            cmd = req.disc_lift.command
            if cmd == pb.DISC_LIFT_CMD_UP:
                self.disc_lift_state = pb.DISC_LIFT_STATE_UP
            elif cmd == pb.DISC_LIFT_CMD_DOWN:
                self.disc_lift_state = pb.DISC_LIFT_STATE_DOWN
        elif req.HasField("lighting"):
            self.light_enabled = req.lighting.enabled
        elif req.HasField("manual_drive"):
            self.last_manual_motion = req.manual_drive.motion
            self._apply_manual_motion(req.manual_drive.motion, req.manual_drive.speed_ratio)
        elif req.HasField("chassis_power"):
            self.chassis_enabled = req.chassis_power.enabled
            if not self.chassis_enabled:
                self.left_wheel_speed = 0.0
                self.right_wheel_speed = 0.0

        resp = pb.ControlCommandResponse()
        resp.result = pb.RESULT_SUCCESS
        resp.message = "Control applied"
        resp.applied_command.CopyFrom(req)
        return resp.SerializeToString(), pb.MSG_ID_CONTROL_COMMAND_RESPONSE

    def _apply_manual_motion(self, motion: int, speed_ratio: float):
        if not self.chassis_enabled:
            self.left_wheel_speed = 0.0
            self.right_wheel_speed = 0.0
            return

        base = self.chassis_settings.run_speed * max(0.0, min(speed_ratio, 1.0))
        if motion == pb.MANUAL_MOTION_FORWARD:
            self.left_wheel_speed = base
            self.right_wheel_speed = base
        elif motion == pb.MANUAL_MOTION_BACKWARD:
            self.left_wheel_speed = -base
            self.right_wheel_speed = -base
        elif motion == pb.MANUAL_MOTION_FORWARD_LEFT:
            self.left_wheel_speed = base * 0.5
            self.right_wheel_speed = base
        elif motion == pb.MANUAL_MOTION_FORWARD_RIGHT:
            self.left_wheel_speed = base
            self.right_wheel_speed = base * 0.5
        elif motion == pb.MANUAL_MOTION_BACKWARD_LEFT:
            self.left_wheel_speed = -base * 0.5
            self.right_wheel_speed = -base
        elif motion == pb.MANUAL_MOTION_BACKWARD_RIGHT:
            self.left_wheel_speed = -base
            self.right_wheel_speed = -base * 0.5
        else:
            self.left_wheel_speed = 0.0
            self.right_wheel_speed = 0.0

    def build_device_status_report(self):
        report = pb.DeviceStatusReport()
        report.utc_time = 1710000000
        report.system_status = pb.SYS_STATUS_NORMAL
        report.wifi_status = pb.WIFI_SUCCESS
        report.work_mode = self.chassis_settings.work_mode
        report.left_wheel_speed = self.left_wheel_speed
        report.right_wheel_speed = self.right_wheel_speed
        report.disc_speed_rpm = self.chassis_settings.disc_speed_rpm
        report.disc_enabled = self.chassis_settings.disc_enabled
        report.disc_lift_state = self.disc_lift_state
        report.light_enabled = self.light_enabled
        report.position.x = self.position_x
        report.position.y = self.position_y
        report.position.heading_deg = self.position_heading
        report.chassis_enabled = self.chassis_enabled
        return report.SerializeToString(), pb.MSG_ID_DEVICE_STATUS_REPORT
