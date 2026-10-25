"""Bounded native serial transactions and write-over-poll scheduling; no hardware."""
import threading
import time
import tempfile
import unittest
from unittest.mock import patch

from grinder_chassis_driver.modbus_transport import ModbusTransportError, _SerialPort
from test_modbus_transport import FakeTransport, FakeSerialPort


class SerialSchedulingTest(unittest.TestCase):
    def test_slow_raw_log_file_does_not_hold_modbus_transactions(self):
        with tempfile.TemporaryDirectory() as directory:
            transport=FakeTransport('/dev/fake',1,raw_log_enabled=True,raw_log_file=directory+'/raw.log')
            entered,release=threading.Event(),threading.Event()
            transport._raw_log_writer._write=lambda lines:(entered.set(),release.wait(2))
            try:
                transport.write_register_block(0x4010,[1,1])
                self.assertTrue(entered.wait(1))
                start=time.monotonic()
                for speed in range(100):transport.write_register_block(0x4010,[speed,speed])
                self.assertLess(time.monotonic()-start,.5)
                self.assertEqual(99,transport.read_register_block(0x4010,2)[0])
            finally:
                release.set();transport.shutdown()

    def test_fragmented_response_cannot_extend_transaction_deadline(self):
        port = _SerialPort('/dev/fake',19200,.1)
        port._fd=123
        port._transaction_deadline=10.1
        now=[10.]
        waits=[]
        def select_ready(read,write,error,timeout):
            waits.append(timeout)
            now[0]+=.04
            return read,write,[]
        with patch.object(port,'open'), patch('grinder_chassis_driver.modbus_transport.time.monotonic',side_effect=lambda:now[0]), \
                patch('grinder_chassis_driver.modbus_transport.select.select',side_effect=select_ready), \
                patch('grinder_chassis_driver.modbus_transport.os.read',return_value=b'x'):
            with self.assertRaises(ModbusTransportError):port.read_exactly(20)
        self.assertLessEqual(len(waits),3)
        self.assertGreater(waits[0],waits[1])
        self.assertGreater(waits[1],waits[2])

    def test_response_header_and_body_share_budget(self):
        port=_SerialPort('/dev/fake',19200,.1)
        port._fd=123
        port._transaction_deadline=10.1
        now=[10.]
        def ready(read,write,error,timeout):
            now[0]+=.04
            return read,write,[]
        with patch.object(port,'open'), patch('grinder_chassis_driver.modbus_transport.time.monotonic',side_effect=lambda:now[0]), \
                patch('grinder_chassis_driver.modbus_transport.select.select',side_effect=ready), \
                patch('grinder_chassis_driver.modbus_transport.os.read',return_value=b'x'):
            self.assertEqual(b'x',port.read_exactly(1))
            self.assertEqual(b'x',port.read_exactly(1))
            with self.assertRaises(ModbusTransportError):port.read_exactly(1)

    def test_waiting_write_precedes_waiting_status_poll(self):
        transport=FakeTransport('/dev/fake',1)
        entered,release=threading.Event(),threading.Event()
        self.addCleanup(release.set)
        class Serial(FakeSerialPort):
            def __init__(self):
                super().__init__();self.order=[];self.first=True
            def write(self,payload):
                self.order.append(payload[1])
                super().write(payload)
            def read_exactly(self,size):
                if self.first:
                    self.first=False;entered.set();release.wait(2)
                return super().read_exactly(size)
        transport._serial_port=Serial()
        errors=[]
        def call(function):
            try:function()
            except Exception as exc:errors.append(exc)
        first=threading.Thread(target=call,args=(lambda:transport.read_register_block(0x4017,2),))
        poll=threading.Thread(target=call,args=(lambda:transport.read_register_block(0x4017,2),))
        writer=threading.Thread(target=call,args=(lambda:transport.write_register_block(0x4010,[100,100]),))
        first.start();self.assertTrue(entered.wait(1));poll.start();writer.start()
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            with transport._transactions.condition:
                if transport._transactions.writers:break
            time.sleep(.001)
        else:self.fail('writer did not register')
        release.set()
        for thread in (first,poll,writer):thread.join(2);self.assertFalse(thread.is_alive())
        self.assertEqual([],errors)
        self.assertEqual([3,16,3],transport._serial_port.order)


if __name__=='__main__':unittest.main()
