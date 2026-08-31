import errno
import os
import select
import termios
import threading
import time

from grinder_chassis_driver.register_map import to_int16, to_uint16


class ModbusTransportError(RuntimeError):
    """Raised when RS485 Modbus communication fails."""


class _SerialPort:
    _BAUD_RATES = {
        1200: termios.B1200,
        1800: termios.B1800,
        2400: termios.B2400,
        4800: termios.B4800,
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }

    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._fd = None

    def open(self):
        if self._fd is not None:
            return
        if self.baudrate not in self._BAUD_RATES:
            raise ModbusTransportError("Unsupported baudrate: {}".format(self.baudrate))

        flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        try:
            self._fd = os.open(self.port, flags)
        except OSError as exc:
            raise ModbusTransportError("Unable to open serial port {}: {}".format(self.port, exc)) from exc
        try:
            self._configure()
        except Exception as exc:
            self.close()
            raise ModbusTransportError(
                "Unable to configure serial port {}: {}".format(self.port, exc)
            ) from exc

    def close(self):
        if self._fd is None:
            return
        os.close(self._fd)
        self._fd = None

    def write(self, payload):
        self.open()
        self.flush_input()
        view = memoryview(payload)
        total_written = 0
        while total_written < len(payload):
            _, writable, _ = select.select([], [self._fd], [], self.timeout)
            if not writable:
                raise ModbusTransportError("Serial write timeout on {}".format(self.port))
            try:
                written = os.write(self._fd, view[total_written:])
            except OSError as exc:
                raise ModbusTransportError("Serial write failed on {}: {}".format(self.port, exc)) from exc
            total_written += written
        termios.tcdrain(self._fd)

    def read_exactly(self, size):
        self.open()
        chunks = bytearray()
        while len(chunks) < size:
            readable, _, _ = select.select([self._fd], [], [], self.timeout)
            if not readable:
                raise ModbusTransportError(
                    "Serial read timeout on {} after {} of {} bytes".format(self.port, len(chunks), size)
                )
            try:
                chunk = os.read(self._fd, size - len(chunks))
            except OSError as exc:
                if exc.errno == errno.EAGAIN:
                    continue
                raise ModbusTransportError("Serial read failed on {}: {}".format(self.port, exc)) from exc
            if not chunk:
                raise ModbusTransportError("Serial port {} returned no data".format(self.port))
            chunks.extend(chunk)
        return bytes(chunks)

    def flush_input(self):
        if self._fd is not None:
            termios.tcflush(self._fd, termios.TCIFLUSH)

    def _configure(self):
        attrs = termios.tcgetattr(self._fd)
        baud_const = self._BAUD_RATES[self.baudrate]
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud_const
        attrs[5] = baud_const
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIOFLUSH)


class ModbusTransport:
    def __init__(
        self,
        port,
        slave_id,
        baudrate=115200,
        timeout=0.1,
        write_verify=False,
        use_functioncode_16=True,
        raw_log_enabled=False,
        raw_log_file="",
    ):
        self.port = port
        self.slave_id = slave_id
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_verify = write_verify
        self.use_functioncode_16 = use_functioncode_16
        self.raw_log_enabled = bool(raw_log_enabled)
        self.raw_log_file = str(raw_log_file or "").strip()
        self._lock = threading.Lock()
        self._raw_log_lock = threading.Lock()
        self._serial_port = _SerialPort(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        if self.raw_log_enabled and self.raw_log_file:
            log_dir = os.path.dirname(self.raw_log_file)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)

    def connect(self):
        with self._lock:
            self._serial_port.open()

    def close(self):
        with self._lock:
            self._serial_port.close()

    def reconnect(self):
        self.close()
        self.connect()

    def write_int16(self, register_address, value):
        self._write_register(register_address, value, signed=True)

    def write_uint16(self, register_address, value):
        self._write_register(register_address, value, signed=False)

    def write_register_block(self, start_address, values):
        self.connect()
        encoded_values = [to_uint16(value) for value in values]
        try:
            with self._lock:
                payload = bytearray()
                for value in encoded_values:
                    payload.extend(value.to_bytes(2, byteorder="big", signed=False))
                frame = self._build_request(
                    function_code=16,
                    payload=(
                        start_address.to_bytes(2, byteorder="big", signed=False)
                        + len(encoded_values).to_bytes(2, byteorder="big", signed=False)
                        + bytes([len(payload)])
                        + payload
                    ),
                )
                self._serial_port.write(frame)
                self._log_raw_frame("TX", frame)
                response = self._serial_port.read_exactly(8)
                self._log_raw_frame("RX", response)
                self._validate_response(response, expected_function=16)
        except Exception as exc:
            self.close()
            raise ModbusTransportError(str(exc))

    def read_register_block(self, start_address, count, signed_indices=None):
        self.connect()
        signed_indices = set(signed_indices or [])
        try:
            with self._lock:
                frame = self._build_request(
                    function_code=3,
                    payload=(
                        start_address.to_bytes(2, byteorder="big", signed=False)
                        + count.to_bytes(2, byteorder="big", signed=False)
                    ),
                )
                self._serial_port.write(frame)
                self._log_raw_frame("TX", frame)
                header = self._serial_port.read_exactly(3)
                if header[1] & 0x80:
                    response = header + self._serial_port.read_exactly(2)
                    self._log_raw_frame("RX", response)
                    self._raise_for_exception(response)
                byte_count = header[2]
                body = self._serial_port.read_exactly(byte_count + 2)
                response = header + body
                self._log_raw_frame("RX", response)
                self._validate_response(response, expected_function=3)
                values = []
                for index in range(0, byte_count, 2):
                    values.append(int.from_bytes(response[3 + index : 5 + index], byteorder="big", signed=False))
        except Exception as exc:
            self.close()
            raise ModbusTransportError(str(exc))

        result = []
        for index, value in enumerate(values):
            result.append(to_int16(value) if index in signed_indices else value)
        return result

    def _write_register(self, register_address, value, signed):
        self.connect()
        function_code = 16 if self.use_functioncode_16 else 6
        encoded_value = to_uint16(value) if signed else to_uint16(value)
        try:
            with self._lock:
                if function_code == 16:
                    frame = self._build_request(
                        function_code=16,
                        payload=(
                            register_address.to_bytes(2, byteorder="big", signed=False)
                            + (1).to_bytes(2, byteorder="big", signed=False)
                            + b"\x02"
                            + encoded_value.to_bytes(2, byteorder="big", signed=False)
                        ),
                    )
                else:
                    frame = self._build_request(
                        function_code=6,
                        payload=(
                            register_address.to_bytes(2, byteorder="big", signed=False)
                            + encoded_value.to_bytes(2, byteorder="big", signed=False)
                        ),
                    )
                self._serial_port.write(frame)
                self._log_raw_frame("TX", frame)
                response = self._serial_port.read_exactly(8)
                self._log_raw_frame("RX", response)
                self._validate_response(response, expected_function=function_code)
        except Exception as exc:
            self.close()
            raise ModbusTransportError(str(exc))
        if self.write_verify:
            readback = self.read_register_block(register_address, 1, signed_indices=(0,) if signed else ())
            if readback[0] != value:
                raise ModbusTransportError(
                    "Write verify failed for register 0x{:04X}: expected {}, got {}".format(
                        register_address, value, readback[0]
                    )
                )

    def _build_request(self, function_code, payload):
        frame = bytes([self.slave_id, function_code]) + payload
        crc = self._crc16(frame)
        return frame + crc.to_bytes(2, byteorder="little", signed=False)

    def _validate_response(self, response, expected_function):
        if len(response) < 5:
            raise ModbusTransportError("Response frame too short")
        if response[0] != self.slave_id:
            raise ModbusTransportError(
                "Unexpected slave id in response: expected {}, got {}".format(self.slave_id, response[0])
            )
        if response[1] & 0x80:
            self._raise_for_exception(response)
        if response[1] != expected_function:
            raise ModbusTransportError(
                "Unexpected Modbus function code: expected {}, got {}".format(expected_function, response[1])
            )
        received_crc = int.from_bytes(response[-2:], byteorder="little", signed=False)
        calculated_crc = self._crc16(response[:-2])
        if received_crc != calculated_crc:
            raise ModbusTransportError(
                "CRC mismatch: expected 0x{:04X}, got 0x{:04X}".format(calculated_crc, received_crc)
            )

    def _raise_for_exception(self, response):
        exception_code = response[2]
        raise ModbusTransportError(
            "Modbus exception response: function 0x{:02X}, exception code 0x{:02X}".format(response[1], exception_code)
        )

    @staticmethod
    def _crc16(payload):
        crc = 0xFFFF
        for byte in payload:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def _log_raw_frame(self, direction, payload):
        if not self.raw_log_enabled or not self.raw_log_file or payload is None:
            return
        try:
            hex_data = " ".join("{:02X}".format(b) for b in bytes(payload))
            self._append_raw_log_line("{} {}".format(str(direction), hex_data))
        except Exception:
            # Logging failures should never affect control loop.
            pass

    def log_event(self, event, detail=""):
        if not self.raw_log_enabled or not self.raw_log_file:
            return
        try:
            message = str(event)
            if detail:
                message = "{} {}".format(message, str(detail))
            self._append_raw_log_line(message)
        except Exception:
            # Diagnostic logging must never interrupt chassis control.
            pass

    def _append_raw_log_line(self, message):
        now_ns = time.time_ns()
        seconds = now_ns // 1000000000
        milliseconds = (now_ns // 1000000) % 1000
        ts = "{}.{:03d}".format(
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(seconds)),
            milliseconds,
        )
        line = "[{}] {}\n".format(ts, message)
        with self._raw_log_lock:
            with open(self.raw_log_file, "a", encoding="utf-8") as f:
                f.write(line)
