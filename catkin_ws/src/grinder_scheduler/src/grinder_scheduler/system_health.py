"""Read-only, 2s Linux board telemetry; no sampling on SLLink/control threads."""
import glob
import math
import os
import shutil
import subprocess
import threading
import time


class LinuxHealthReader:
    def __init__(self, read=None, zones=None, disk_usage=None, disk_path="/"):
        self.read = read or self._read
        self.zones = zones if zones is not None else glob.glob('/sys/class/thermal/thermal_zone*')
        self.disk_usage = disk_usage or shutil.disk_usage
        self.disk_path = disk_path
        self.previous_cpu = None

    @staticmethod
    def _read(path):
        with open(path, encoding='ascii') as handle:
            return handle.read()

    def sample(self):
        result = {}
        try:
            values = [int(v) for v in self.read('/proc/stat').splitlines()[0].split()[1:9]]
            total, idle = sum(values), values[3] + values[4]
            previous = self.previous_cpu
            self.previous_cpu = (total, idle)
            if previous and total > previous[0] and idle >= previous[1]:
                result['cpu_usage_percent'] = max(0.0, min(100.0, 100.0 * (1.0 - (idle - previous[1]) / (total - previous[0]))))
        except (OSError, ValueError, IndexError):
            self.previous_cpu = None
        try:
            memory = {line.split(':')[0]: int(line.split()[1]) * 1024
                      for line in self.read('/proc/meminfo').splitlines() if line.startswith(('MemTotal:', 'MemAvailable:'))}
            total, available = memory['MemTotal'], memory['MemAvailable']
            if total > 0 and 0 <= available <= total:
                result.update(memory_total_bytes=total, memory_available_bytes=available,
                              memory_usage_percent=100.0 * (total - available) / total)
        except (OSError, ValueError, KeyError, IndexError):
            pass
        temperatures = []
        for zone in self.zones:
            try:
                kind = self.read(os.path.join(zone, 'type')).lower()
                if not any(name in kind for name in ('cpu', 'soc', 'package')):
                    continue
                value = float(self.read(os.path.join(zone, 'temp'))) / 1000.0
                if math.isfinite(value) and -40 <= value <= 150:
                    temperatures.append(value)
            except (OSError, ValueError):
                pass
        if temperatures:
            result['cpu_temperature_c'] = max(temperatures)
        try:
            disk = self.disk_usage(self.disk_path)
            if disk.total > 0 and 0 <= disk.free <= disk.total:
                result.update(disk_usage_percent=100.0 * (disk.total - disk.free) / disk.total,
                              disk_free_bytes=disk.free)
        except (OSError, ValueError):
            pass
        try:
            uptime = float(self.read('/proc/uptime').split()[0])
            if math.isfinite(uptime) and uptime >= 0:
                result['uptime_seconds'] = int(uptime)
        except (OSError, ValueError, IndexError):
            pass
        return result


class CellularHealthReader:
    """Query ModemManager only; never enable radios, connect, send AT or change signal sampling."""
    def __init__(self, query=None):
        self.query = query or self._query

    @staticmethod
    def _query(args):
        executable = shutil.which('mmcli')
        if executable is None:
            raise OSError('ModemManager unavailable')
        result = subprocess.run([executable] + args + ['--output-keyvalue'],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                timeout=0.4, check=True, universal_newlines=True)
        return result.stdout

    @staticmethod
    def _fields(text):
        return {key.strip(): value.strip().strip("'\"") for key, separator, value in
                (line.partition(':') for line in text.splitlines()) if separator}

    def sample(self):
        try:
            listed = self._fields(self.query(['--list-modems']))
            modems = [v for k, v in listed.items() if k.startswith('modem-list.value[')]
            if len(modems) != 1 or not modems[0].startswith('/org/freedesktop/ModemManager1/Modem/'):
                return {}
            values = self._fields(self.query(['--modem', modems[0]]))
            state = values.get('modem.generic.state', '')
            technology = values.get('modem.generic.access-technologies', '')
            result = {}
            if state and state != '--':
                result['cellular_state'] = state[:64]
            if technology and technology != '--':
                result['cellular_access_technology'] = technology[:64]
            if (state in ('registered', 'connected', 'connecting') and
                    values.get('modem.generic.signal-quality.recent') == 'yes'):
                signal = int(values.get('modem.generic.signal-quality.value', '-1'))
                if 0 <= signal <= 100:
                    result['cellular_signal_percent'] = signal
            return result
        except (OSError, ValueError, subprocess.SubprocessError):
            return {}


class SystemHealthSampler:
    def __init__(self, disk_path='/'):
        self.reader = LinuxHealthReader(disk_path=disk_path)
        self.cellular = CellularHealthReader()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snapshot = {}
        self._at = None
        self._thread = threading.Thread(target=self._run, name='system-health', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            deadline = time.monotonic() + 2.0
            try:
                sample = self.reader.sample()
                sample.update(self.cellular.sample())
                with self._lock:
                    self._snapshot, self._at = sample, time.monotonic()
            except Exception:
                # Previous snapshot expires; never block or fail motion on diagnostics.
                pass
            self._stop.wait(max(0.0, deadline - time.monotonic()))

    def snapshot(self):
        with self._lock:
            return dict(self._snapshot) if self._at is not None and 0 <= time.monotonic() - self._at <= 5 else {}

    def close(self):
        self._stop.set()
        self._thread.join(timeout=0.2)


def fill_system_status(report, chassis, connected, health):
    if connected is True and chassis is not None:
        for name in ('left_motor_fault_code', 'right_motor_fault_code'):
            value = getattr(chassis, name, None)
            if isinstance(value, int) and -32768 <= value <= 32767:
                setattr(report, name, value)
    if health:
        for name, value in health.items():
            setattr(report.system_health, name, value)
