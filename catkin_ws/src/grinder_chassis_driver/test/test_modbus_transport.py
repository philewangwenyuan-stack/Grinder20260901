import unittest

from grinder_chassis_driver.modbus_transport import ModbusTransport


class FakeSerialPort:
    def __init__(self):
        self.registers = {}
        self.pending_response = b""

    def open(self):
        return

    def close(self):
        return

    def flush_input(self):
        return

    def write(self, payload):
        slave_id = payload[0]
        function_code = payload[1]
        if function_code == 16:
            start_address = int.from_bytes(payload[2:4], byteorder="big", signed=False)
            count = int.from_bytes(payload[4:6], byteorder="big", signed=False)
            byte_count = payload[6]
            data = payload[7 : 7 + byte_count]
            for index in range(count):
                register = int.from_bytes(data[index * 2 : index * 2 + 2], byteorder="big", signed=False)
                self.registers[start_address + index] = register
            self.pending_response = self._build_response(
                slave_id, function_code, payload[2:6]
            )
            return

        if function_code == 6:
            start_address = int.from_bytes(payload[2:4], byteorder="big", signed=False)
            value = int.from_bytes(payload[4:6], byteorder="big", signed=False)
            self.registers[start_address] = value
            self.pending_response = self._build_response(slave_id, function_code, payload[2:6])
            return

        if function_code == 3:
            start_address = int.from_bytes(payload[2:4], byteorder="big", signed=False)
            count = int.from_bytes(payload[4:6], byteorder="big", signed=False)
            registers = bytearray()
            for index in range(count):
                registers.extend(self.registers.get(start_address + index, 0).to_bytes(2, "big"))
            self.pending_response = self._build_response(
                slave_id, function_code, bytes([len(registers)]) + bytes(registers)
            )
            return

        raise AssertionError("Unexpected Modbus function code {}".format(function_code))

    def read_exactly(self, size):
        chunk = self.pending_response[:size]
        self.pending_response = self.pending_response[size:]
        return chunk

    @staticmethod
    def _build_response(slave_id, function_code, payload):
        frame = bytes([slave_id, function_code]) + payload
        crc = ModbusTransport._crc16(frame)
        return frame + crc.to_bytes(2, byteorder="little", signed=False)


class FakeTransport(ModbusTransport):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._serial_port = FakeSerialPort()


class ModbusTransportTest(unittest.TestCase):
    def test_signed_write_and_read(self):
        transport = FakeTransport("/dev/null", 1)
        transport.write_int16(0x4010, -100)
        transport.write_uint16(0x4013, 1)

        values = transport.read_register_block(0x4010, 4, signed_indices=(0,))
        self.assertEqual(values[0], -100)
        self.assertEqual(values[3], 1)

    def test_block_write(self):
        transport = FakeTransport("/dev/null", 1)
        transport.write_register_block(0x4010, [0xFFFF, 2, 3])
        values = transport.read_register_block(0x4010, 3, signed_indices=(0,))
        self.assertEqual(values[0], -1)
        self.assertEqual(values[1], 2)
        self.assertEqual(values[2], 3)


if __name__ == "__main__":
    unittest.main()
