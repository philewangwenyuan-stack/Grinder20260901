import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location('health_test', Path(__file__).resolve().parents[1] / 'src/grinder_scheduler/system_health.py')
health = importlib.util.module_from_spec(spec)
spec.loader.exec_module(health)
spec = importlib.util.spec_from_file_location('sl_link_pb2', ROOT / 'third_party/sl_linka/sdk/python/sl_link/message_gen/sl_link_pb2.py')
pb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pb)


class SystemHealthTest(unittest.TestCase):
    def reader(self):
        files = {
            '/proc/stat': 'cpu 10 0 20 70 0 0 0 0 10 0\n',
            '/proc/meminfo': 'MemTotal: 1000 kB\nMemAvailable: 400 kB\n',
            '/proc/uptime': '1234.56 0',
            '/thermal/cpu/type': 'cpu-thermal', '/thermal/cpu/temp': '62500',
            '/thermal/gpu/type': 'gpu-thermal', '/thermal/gpu/temp': '90000',
        }
        def read(path):
            path = path.replace('\\', '/')
            if path not in files:
                raise OSError('missing')
            return files[path]
        return files, health.LinuxHealthReader(read, ['/thermal/cpu', '/thermal/gpu'],
                                               lambda _: SimpleNamespace(total=1000, free=250))

    def test_cpu_delta_memory_available_and_cpu_sensor(self):
        files, reader = self.reader()
        first = reader.sample()
        self.assertNotIn('cpu_usage_percent', first)
        self.assertEqual(62.5, first['cpu_temperature_c'])
        self.assertEqual(60.0, first['memory_usage_percent'])
        self.assertEqual(1024000, first['memory_total_bytes'])
        self.assertEqual(75.0, first['disk_usage_percent'])
        self.assertEqual(1234, first['uptime_seconds'])
        files['/proc/stat'] = 'cpu 20 0 40 140 0 0 0 0 20 0'
        self.assertAlmostEqual(30.0, reader.sample()['cpu_usage_percent'])

    def test_missing_and_invalid_metrics_do_not_reuse_old_healthy_values(self):
        files, reader = self.reader()
        reader.sample()
        files.clear()
        second = reader.sample()
        self.assertEqual({'disk_usage_percent': 75.0, 'disk_free_bytes': 250}, second)
        files['/proc/stat'] = 'cpu 1 0 0 0 0 0 0 0'
        files['/proc/meminfo'] = 'MemTotal: 0 kB\nMemAvailable: 1 kB'
        self.assertNotIn('cpu_usage_percent', reader.sample())
        self.assertNotIn('memory_usage_percent', reader.sample())

    def test_signed_registers_and_disconnect_unknown_on_real_protobuf(self):
        report = pb.DeviceStatusReport()
        chassis = SimpleNamespace(left_motor_fault_code=-32768, right_motor_fault_code=0)
        health.fill_system_status(report, chassis, True, {'cpu_usage_percent': 0.0})
        decoded = pb.DeviceStatusReport.FromString(report.SerializeToString())
        self.assertEqual(-32768, decoded.left_motor_fault_code)
        self.assertTrue(decoded.HasField('right_motor_fault_code'))
        self.assertTrue(decoded.system_health.HasField('cpu_usage_percent'))
        report = pb.DeviceStatusReport()
        health.fill_system_status(report, chassis, False, {})
        self.assertFalse(report.HasField('left_motor_fault_code'))
        self.assertFalse(report.HasField('system_health'))

    def test_stalled_sampler_expires(self):
        sampler = health.SystemHealthSampler.__new__(health.SystemHealthSampler)
        sampler._lock = threading.Lock()
        sampler._snapshot = {'cpu_usage_percent': 0.0}
        sampler._at = time.monotonic() - 6
        self.assertEqual({}, sampler.snapshot())

    def test_cellular_recent_signal_and_non_recent_or_missing_modem(self):
        listing = 'modem-list.value[1]: /org/freedesktop/ModemManager1/Modem/3'
        fields = ('modem.generic.state: connected\nmodem.generic.access-technologies: lte\n'
                  'modem.generic.signal-quality.value: 75\nmodem.generic.signal-quality.recent: yes')
        reader = health.CellularHealthReader(lambda args: listing if args == ['--list-modems'] else fields)
        self.assertEqual(75, reader.sample()['cellular_signal_percent'])
        fields = fields.replace('recent: yes', 'recent: no')
        self.assertNotIn('cellular_signal_percent', reader.sample())
        fields = fields.replace('value: 75', 'value: 150').replace('recent: no', 'recent: yes')
        self.assertNotIn('cellular_signal_percent', reader.sample())
        self.assertEqual({}, health.CellularHealthReader(lambda _: '').sample())


if __name__ == '__main__':
    unittest.main()
