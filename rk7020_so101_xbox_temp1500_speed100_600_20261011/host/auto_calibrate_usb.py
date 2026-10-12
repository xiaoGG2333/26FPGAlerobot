"""Direct USB STS3215 calibration using the audited manufacturer's mathematics.

The host owns all timing. There is NO local hardware watchdog: a stopped Python
process, suspended VM, or disconnected USB cannot guarantee TorqueOFF. A status
packet carries neither address nor sequence; quiet draining, ID/checksum/exact
length checks cannot distinguish an old same-ID/same-length valid response.
No EEPROM18/protection disable, old-goal restoration, or velocity-write retries.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import serial

from auto_calibrate_zynq import (Calibrator, CalibrationError, word, signed,
                                wrapped_delta)
from vendor_calibration_math import MOTOR_IDS, SOURCE_URL


class TemperatureNotReproduced(CalibrationError):
    """High feedback was followed by a confirmed STOP and cool rechecks.

    This does NOT prove the initial reading false. A caller may start a bounded
    NEW calibration session; the stopped session and its endpoints stay invalid.
    """


class TemperatureDebounce:
    """User-selected 45 C / 1.5 s continuous feedback policy, per motor.

    Retains actual bytes and events. A fresh <=45 sample ends an episode;
    a pending episode observed/left unmonitored beyond 1.5 s is a fault.
    Device alarms and all other guards bypass this policy entirely.
    """
    threshold = 45
    grace = 1.5

    def __init__(self):
        self.pending = {}
        self.events = []

    def deadlines(self):
        now = time.monotonic()
        for motor, event in self.pending.items():
            if now - event['started_monotonic'] > self.grace:
                raise CalibrationError(f'ID{motor}: temperature feedback >45 C exceeded 1.5 s')

    def observe(self, motor, temperature, source):
        self.deadlines()
        now = time.monotonic()
        if temperature > self.threshold:
            if motor not in self.pending:
                self.pending[motor] = {'id': motor, 'started_monotonic': now,
                    'started_time': time.time(), 'maximum_c': temperature,
                    'high_samples': 0, 'first_source': source,
                    'initial_reading_proven_false': False}
                print(f'TEMP WINDOW ID{motor} T={temperature}: monitoring <=1.5 s', flush=True)
            event = self.pending[motor]
            event['maximum_c'] = max(event['maximum_c'], temperature)
            event['high_samples'] += 1
        elif motor in self.pending:
            event = self.pending.pop(motor)
            event.update(duration_s=now-event['started_monotonic'], recovered_c=temperature,
                         recovered_time=time.time(), classification='SHORT_HIGH_ACCEPTED_BY_USER_POLICY')
            self.events.append(event)
            print(f'TEMP WINDOW ID{motor} recovered T={temperature} after {event["duration_s"]:.3f}s; continuing', flush=True)

    def report(self):
        now = time.monotonic()
        return {'threshold_c': self.threshold, 'maximum_continuous_high_s': self.grace,
                'real_temperature_proven_false': False, 'raw_values_substituted': False,
                'resolved_events': self.events,
                'unresolved_events': [{**x, 'elapsed_since_start_at_report_s': now-x['started_monotonic']}
                                      for x in self.pending.values()]}


def packet(motor, operation, payload=b''):
    body = bytes((motor, len(payload) + 2, operation)) + bytes(payload)
    return b'\xff\xff' + body + bytes((~sum(body) & 255,))


class USBProtocol:
    """One request, one strictly checked status; timeout includes all RX bytes."""
    def __init__(self, port, folder, serial_factory=serial.Serial):
        self.log = (folder / 'usb_packets.jsonl').open('w', encoding='utf8')
        self.log_error = None
        try:
            self.serial = serial_factory(port, 1000000, timeout=0, write_timeout=.05)
        except BaseException:
            self.log.close()
            raise

    def record(self, item):
        if self.log is None:
            return
        try:
            self.log.write(json.dumps(item) + '\n')
            self.log.flush()
        except Exception as error:
            self.log_error = repr(error)
            try:
                self.log.close()
            except Exception:
                pass
            self.log = None

    def exchange(self, motor, operation, payload=b'', expected=0):
        tx = packet(motor, operation, payload)
        started = time.monotonic()
        item = {'time': time.time(), 'id': motor, 'op': operation,
                'tx': tx.hex(), 'drained': '', 'rx': '', 'ok': False}
        drained = bytearray()
        rx = bytearray()
        try:
            # A bounded quiet period avoids knowingly consuming a prior reply.
            end = time.monotonic() + .025
            quiet = time.monotonic()
            while time.monotonic() < end:
                chunk = self.serial.read(min(96, max(1, self.serial.in_waiting)))
                if chunk:
                    drained.extend(chunk)
                    quiet = time.monotonic()
                    if len(drained) > 4096:
                        raise CalibrationError('USB RX did not become quiet')
                elif time.monotonic() - quiet >= .002:
                    break
                else:
                    time.sleep(.0005)
            else:
                raise CalibrationError('USB RX quiet deadline reached')
            if self.serial.write(tx) != len(tx):
                raise CalibrationError('Incomplete USB request write; no retry')
            deadline = time.monotonic() + .05
            buffer = bytearray()
            while time.monotonic() < deadline:
                chunk = self.serial.read(min(96, max(1, self.serial.in_waiting)))
                if not chunk:
                    time.sleep(.0005)
                    continue
                rx.extend(chunk)
                buffer.extend(chunk)
                if len(rx) > 512:
                    raise CalibrationError('Oversized USB response')
                while len(buffer) >= 4:
                    header = buffer.find(b'\xff\xff')
                    if header < 0:
                        buffer[:] = buffer[-1:] if buffer[-1] == 255 else b''
                        break
                    if header:
                        del buffer[:header]
                    if len(buffer) < 4:
                        break
                    size = buffer[3] + 4
                    if not 6 <= size <= 40:
                        raise CalibrationError('Invalid USB response packet length')
                    if len(buffer) < size:
                        break
                    frame = bytes(buffer[:size])
                    del buffer[:size]
                    if frame == tx:
                        continue  # An exact adapter echo is not a status reply.
                    if sum(frame[2:]) & 255 != 255:
                        raise CalibrationError('USB response checksum mismatch')
                    if frame[2] != motor or frame[3] != expected + 2:
                        raise CalibrationError('USB response ID/exact-length mismatch')
                    if frame[4]:
                        raise CalibrationError(f'ID{motor}: servo status 0x{frame[4]:02x}')
                    item['ok'] = True
                    return frame[5:-1]
            raise CalibrationError('USB response timeout; request not retried')
        except BaseException as error:
            item['error'] = repr(error)
            raise
        finally:
            item.update(drained=drained.hex(), rx=rx.hex(),
                        elapsed_ms=(time.monotonic() - started) * 1000)
            self.record(item)

    def read(self, motor, address, size):
        return self.exchange(motor, 2, bytes((address, size)), size)

    def write(self, motor, address, data):
        self.exchange(motor, 3, bytes((address,)) + bytes(data))

    def close(self):
        try:
            self.serial.close()
        finally:
            if self.log is not None:
                self.log.close()


class Bridge:
    """CAL1-compatible guard and off-first rollback for a direct USB adapter."""
    def __init__(self, port, folder, serial_factory=serial.Serial, temperature_debounce=False,
                 discover_mechanical_range=False):
        self.folder = folder
        self.stop_path = folder / 'STOP'
        self.bus = USBProtocol(port, folder, serial_factory)
        self.active = False
        self.active_axis = None
        self.original = {}
        self.fault = None
        self.committed = False
        self.calibrator = None
        self.last_monitor = None
        self.cleanup_lines = None
        self.cleanup_details = None
        self.cleanup_output_error = None
        self.cleaning = False
        self.last_good_temperature = {}
        self.temperature_recheck = None
        self.external_stop_path = None
        self.temperature_guard = TemperatureDebounce() if temperature_debounce else None
        self.discover_mechanical_range = discover_mechanical_range

    def _operator_stop(self):
        return self.stop_path.exists() or bool(self.external_stop_path and self.external_stop_path.exists())

    def validate_feedback(self, motor, data, source='READ56/15'):
        # Position, voltage and device alarm failures are never filtered.
        position = signed(data[:2], 15)
        if (not 0 <= position <= 4095 or not 90 <= data[6] <= 126 or data[9]):
            raise CalibrationError(f'ID{motor}: position/voltage/device alarm fault; no temperature filtering')
        if self.temperature_guard is not None:
            self.temperature_guard.observe(motor, data[7], source)
            return data
        if data[7] > 55:
            return self._recheck_temperature(motor, data)
        self.last_good_temperature[motor] = (data[7], time.monotonic())
        return data

    def _recheck_temperature(self, motor, data, temperature=None, source='READ56/15'):
        temperature = data[7] if temperature is None else temperature
        previous = self.last_good_temperature.get(motor)
        previous_age = time.monotonic() - previous[1] if previous else None
        event = {'id': motor, 'initial_raw': data.hex(), 'initial_temperature': temperature,
                 'initial_source': source,
                 'initial_time': time.time(), 'classification': 'UNRESOLVED',
                 'temperature_threshold_unchanged': 55, 'samples': [],
                 'initial_reading_proven_false': False, 'same_session_resume': False}
        event['previous_healthy_sample_age_at_trigger_s'] = previous_age
        self.temperature_recheck = event
        self.fault = f'ID{motor}: high temperature feedback {temperature}'
        # No reads, Mode1 Speed0 writes, or low-value substitution before OFF.
        # Existing cleanup prioritizes the moving axis and confirms all six OFF.
        self.finish(False)
        event['cleanup_confirmed'] = True
        event['cleanup_errors'] = self.cleanup_details['errors']
        print(f'TEMP ID{motor} T={temperature}: six axes OFF, parameters restored; rechecking', flush=True)
        try:
            if (self._operator_stop() or self.bus.log_error or previous is None
                    or previous_age > 5 or temperature >= 80 or event['cleanup_errors']
                    or self.cleanup_output_error):
                raise CalibrationError('Temperature recheck not eligible for bounded restart')
            # Five spaced groups cover >=2s after confirmed OFF. Require all
            # three packet shapes to be cool, near the last healthy value, and
            # alarm-free. A second high or any uncertainty permanently aborts.
            began = time.monotonic()
            for group in range(5):
                if self._operator_stop() or self.bus.log_error:
                    raise CalibrationError('STOP/log failure during temperature recheck')
                if group:
                    end = began + group * .5
                    while time.monotonic() < end:
                        if self._operator_stop():
                            raise CalibrationError('Operator STOP during temperature recheck')
                        time.sleep(min(.04, end - time.monotonic()))
                single = self.bus.read(motor, 63, 1)[0]
                pair_status = self.bus.read(motor, 62, 4)
                block = self.bus.read(motor, 56, 15)
                row = {'group': group, 'time': time.time(), 'single': single,
                       'pair': pair_status[1], 'block': block[7],
                       'pair_status_raw': pair_status.hex(), 'block_raw': block.hex()}
                event['samples'].append(row)
                self._checked_feedback(motor, block)
                if (not 90 <= pair_status[0] <= 126 or pair_status[3]
                        or any(t > 45 or abs(t - previous[0]) > 3
                               for t in (single, pair_status[1], block[7]))):
                    raise CalibrationError('High/inconsistent temperature or alarm reproduced after OFF')
                if self.bus.read(motor, 40, 1) != b'\0' or self.bus.read(motor, 33, 1) != b'\0':
                    raise CalibrationError('Recheck lost confirmed Torque0/Mode0')
                if self.bus.log_error:
                    raise CalibrationError('Packet logging failed during temperature recheck')
            # All axes must remain cool and OFF with exactly the old EEPROM.
            for axis in range(1, 7):
                if self._operator_stop() or self.bus.log_error:
                    raise CalibrationError('STOP/log failure during final recheck')
                block = self.bus.read(axis, 56, 15)
                self._checked_feedback(axis, block)
                rom = self.bus.read(axis, 0, 32) + self.bus.read(axis, 32, 8)
                if (block[7] > 45 or self.bus.read(axis, 40, 1) != b'\0'
                        or rom != self.original[axis][:40]):
                    raise CalibrationError('All-axis restart preflight failed')
            if self._operator_stop() or self.bus.log_error:
                raise CalibrationError('STOP/log failure after temperature recheck')
            event['classification'] = 'HIGH_NOT_REPRODUCED_AFTER_CONFIRMED_STOP'
            event['recheck_duration_s'] = time.monotonic() - began
        except BaseException as error:
            event['error'] = repr(error)
            raise
        finally:
            (self.folder / 'temperature_recheck.json').write_text(json.dumps(event, indent=2), encoding='utf8')
        raise TemperatureNotReproduced(f'ID{motor}: initial high not reproduced after STOP; new full measurement required')

    @staticmethod
    def _checked_feedback(motor, data):
        position = signed(data[:2], 15)
        if (not 0 <= position <= 4095 or not 90 <= data[6] <= 126
                or data[7] > 55 or data[9]):
            raise CalibrationError(f'ID{motor}: unhealthy USB feedback '
                                   f'pos={position} V={data[6]/10} T={data[7]} status={data[9]}')
        return data

    def _monitor_due(self):
        if self.cleaning:
            return
        if self.temperature_guard is not None:
            self.temperature_guard.deadlines()
        if self.active_axis is None or self.last_monitor is None:
            return
        gap = time.monotonic() - self.last_monitor
        if gap > .3:
            raise CalibrationError('USB motion monitoring gap exceeded 300 ms')
        if gap >= .12:
            data = self.bus.read(self.active_axis, 56, 15)
            self.validate_feedback(self.active_axis, data)
            if time.monotonic() - self.last_monitor > .3:
                raise CalibrationError('USB motion monitoring gap exceeded 300 ms')
            self.last_monitor = time.monotonic()

    def _read(self, motor, address, size):
        self._monitor_due()
        return self.bus.read(motor, address, size)

    def _write(self, motor, address, data):
        self._monitor_due()
        if address < 40:
            if self._read(motor, 40, 1) != b'\0' or self._read(motor, 55, 1) != b'\0':
                raise CalibrationError('EEPROM TX requires confirmed Torque0/unlocked')
        self.bus.write(motor, address, data)
        if address < 40:
            time.sleep(.01)  # Manufacturer EEPROM programming interval.

    def _byte(self, motor, address, value):
        self._write(motor, address, bytes((value,)))

    def _health(self, motor):
        data = self._read(motor, 56, 15)
        return self.validate_feedback(motor, data)

    def _session(self):
        if not self.active or self.fault:
            raise CalibrationError('USB calibration session inactive/faulted')
        if self._operator_stop():
            raise CalibrationError('Operator STOP file detected')
        if self.bus.log_error:
            raise CalibrationError('USB packet logging failed: ' + self.bus.log_error)
        if self.temperature_guard is not None:
            self.temperature_guard.deadlines()
        # The current moving axis is checked between every public RPC, including
        # the existing Calibrator's six-support-axis health loop.
        if self.active_axis is not None:
            now = time.monotonic()
            if self.last_monitor is not None and now - self.last_monitor > .3:
                raise CalibrationError('USB motion monitoring gap exceeded 300 ms')
            self._health(self.active_axis)
            self.last_monitor = time.monotonic()
        if self.temperature_guard is not None:
            # Recheck hot support axes between every RPC as well. Raw reads
            # avoid recursive session calls; the moving axis keeps its monitor.
            for motor in list(self.temperature_guard.pending):
                if motor == self.active_axis:
                    continue
                self._monitor_due()
                self.validate_feedback(motor, self.bus.read(motor, 56, 15), 'pending-axis poll')

    def _guarded(self, action):
        try:
            self._session()
            result = action()
            if self.bus.log_error:
                raise CalibrationError('USB packet logging failed: ' + self.bus.log_error)
            return result
        except BaseException as error:
            if self.active:
                self.fault = repr(error)
                try:
                    self.finish(False)
                except Exception:
                    pass
            raise

    def start(self):
        if self.active or self.original:
            raise CalibrationError('USB bridge cannot be started twice')
        snapshots = {}
        # Read all six complete baselines before any write/goal/torque enable.
        for motor in range(1, 7):
            raw = (self._read(motor, 0, 32) + self._read(motor, 32, 8)
                   + self._read(motor, 40, 32) + self._read(motor, 72, 8))
            pos = signed(raw[56:58], 15)
            low, high = word(raw[9:11]), word(raw[11:13])
            if (word(raw[3:5]) != 777 or raw[5] != motor or raw[33] != 0
                    or raw[40] != 0 or not 90 <= raw[62] <= 126 or raw[63] > (45 if self.temperature_guard else 55)
                    or raw[55] not in (0, 1) or raw[65]
                    or not 0 <= low < high <= 4095 or not 0 <= pos <= 4095
                    or (not self.discover_mechanical_range and not low <= pos <= high)):
                raise CalibrationError(f'ID{motor}: USB start baseline is not healthy STS3215/Torque0/Mode0')
            snapshots[motor] = raw
        (self.folder / 'usb_transport_before.json').write_text(json.dumps(
            {'transport': 'direct USB bus servo adapter, 1 Mbaud',
             'hardware_watchdog': False,
             'motors': {str(k): v.hex() for k, v in snapshots.items()}}, indent=2), encoding='utf8')
        self.original = snapshots
        self.active = True
        for motor, raw in snapshots.items():
            self.last_good_temperature[motor] = (raw[63], time.monotonic())

    def backup_snapshot(self, motor):
        # With debounce enabled, retain the all-OFF healthy initial snapshot
        # as the canonical backup, rather than saving an accepted high byte as
        # baseline health. No EEPROM/target changed between start and backup.
        if self.temperature_guard is not None:
            return self._guarded(lambda: self.original[motor])
        return (self.read(motor, 0, 32) + self.read(motor, 32, 8)
                + self.read(motor, 40, 32) + self.read(motor, 72, 8))

    def read(self, motor, address, size):
        def checked():
            if motor not in range(1, 7) or not 0 <= address < 80 or not 1 <= size <= 32 or address + size > 80:
                raise CalibrationError('USB read outside calibration register range')
            data = self._read(motor, address, size)
            if address <= 56 and address + size >= 58:
                position = signed(data[56-address:58-address], 15)
                if not 0 <= position <= 4095:
                    raise CalibrationError('Invalid encoder position; no temperature filtering')
            if address <= 63 < address + size and self.temperature_guard is not None:
                if address <= 62 < address + size and not 90 <= data[62-address] <= 126:
                    raise CalibrationError('Unhealthy voltage; no temperature filtering')
                if address <= 65 < address + size and data[65-address]:
                    raise CalibrationError('Device alarm; no temperature filtering')
                self.temperature_guard.observe(motor, data[63-address], f'READ{address}/{size}')
            elif address <= 63 < address + size and data[63-address] > 55:
                if address <= 62 < address + size and not 90 <= data[62-address] <= 126:
                    raise CalibrationError('High temperature with unhealthy voltage; no restart')
                if address <= 65 < address + size and data[65-address]:
                    raise CalibrationError('Device alarm; no temperature filtering')
                return self._recheck_temperature(motor, data, data[63-address], f'READ{address}/{size}')
            return data
        return self._guarded(checked)

    def _settings(self, motor, acceleration=False):
        self._health(motor)
        if word(self._read(motor, 16, 2)) > 600:
            raise CalibrationError('Maximum torque must be capped at 600')
        cap = 250 if motor in (5, 6) else 400
        if not 0 < word(self._read(motor, 48, 2)) <= cap:
            raise CalibrationError(f'ID{motor}: RAM torque cap exceeds {cap}')
        acc = self._read(motor, 41, 1)[0]
        if acc > 10 or (acceleration and acc != 5):
            raise CalibrationError('Acceleration must be 5 after mode switch')

    def _all_caps(self):
        for motor in range(1, 7):
            self._settings(motor)

    def _validated_write(self, motor, address, data):
        data = bytes(data)
        value = word(data)
        if motor not in range(1, 7):
            raise CalibrationError('Only servo IDs 1..6 are permitted')
        limits = {9: (2, 4095), 11: (2, 4095), 16: (2, 600),
                  31: (2, 4095), 33: (1, 1), 34: (1, 80),
                  35: (1, 200), 36: (1, 95), 40: (1, 1),
                  41: (1, 10), 46: (2, 65535),
                  48: (2, 250 if motor in (5, 6) else 400), 55: (1, 1)}
        if address not in limits or len(data) != limits[address][0] or value > limits[address][1]:
            raise CalibrationError('Register/value outside USB calibration allowlist')
        if address == 46 and abs(signed(data, 15)) > 300:
            raise CalibrationError('Velocity magnitude exceeds 300')
        if address < 40:
            if self._read(motor, 40, 1) != b'\0' or self._read(motor, 55, 1) != b'\0':
                raise CalibrationError('EEPROM writes require confirmed Torque0/unlocked')
        if address in (40, 46) and (value or address == 46):
            mode = self._read(motor, 33, 1)[0]
            # Mode1 Speed0 also auto-enables on this firmware.
            if value or mode == 1:
                self._settings(motor, acceleration=(address == 46 and value != 0))
                if address == 40 and self._read(motor, 46, 2) != b'\0\0':
                    raise CalibrationError('Torque enable requires zero configured velocity')
                if address == 40 and mode == 0:
                    current = signed(self._health(motor)[:2], 15)
                    goal = word(self._read(motor, 42, 2))
                    low = word(self._read(motor, 9, 2))
                    high = word(self._read(motor, 11, 2))
                    if not 0 <= low < high <= 4095 or not low <= current <= high or not low <= goal <= high or abs(goal - current) > 5:
                        raise CalibrationError('Position-mode TorqueON requires a current-position goal')
            if address == 46 and value:
                if mode != 1 or self._read(motor, 40, 1) != b'\1':
                    raise CalibrationError('Nonzero velocity requires Mode1/Torque1')
                if self.active_axis not in (None, motor):
                    raise CalibrationError('A second moving axis is forbidden')
                # Set before TX: a missing ACK must still prioritize this axis.
                self.active_axis = motor
                self.last_monitor = time.monotonic()
        self._write(motor, address, data)
        if self._read(motor, address, len(data)) != data:
            raise CalibrationError(f'ID{motor}: write readback mismatch at {address}')
        if address == 40 and value == 0 and self.active_axis == motor:
            self.active_axis = None
            self.last_monitor = None

    def write(self, motor, address, data):
        return self._guarded(lambda: self._validated_write(motor, address, data))

    def byte(self, motor, address, value):
        return self.write(motor, address, bytes((value,)))

    def uint(self, motor, address, value):
        return self.write(motor, address, int(value).to_bytes(2, 'little'))

    def hold(self, motor):
        def atomic():
            if motor not in range(1, 7):
                raise CalibrationError('Only servo IDs 1..6 are permitted')
            if self.active_axis is not None:
                raise CalibrationError('Atomic hold cannot replace a moving-axis monitor')
            self._all_caps()  # All six support axes capped before first goal.
            mode = self._read(motor, 33, 1)
            torque = self._read(motor, 40, 1)
            if mode != b'\0' or torque != b'\0' or self._read(motor, 46, 2) != b'\0\0':
                raise CalibrationError('Atomic hold requires Mode0/Torque0/Speed0')
            low, high = word(self._read(motor, 9, 2)), word(self._read(motor, 11, 2))
            goal = self._health(motor)[:2]
            position = signed(goal, 15)
            if not 0 <= low < high <= 4095 or not low <= position <= high:
                raise CalibrationError('Atomic hold position outside existing limits')
            # Mode0 Goal42 auto-enables. No intervening goal command is allowed.
            self.active_axis = motor
            self.last_monitor = time.monotonic()
            self._write(motor, 42, goal)
            if self._read(motor, 42, 2) != goal:
                raise CalibrationError('Atomic current-position goal not confirmed')
            if abs(signed(self._health(motor)[:2], 15) - position) > 5:
                raise CalibrationError('Atomic hold moved beyond five counts')
            actual = self._read(motor, 40, 1)
            if actual == b'\0':
                self._byte(motor, 40, 1)
            elif actual != b'\1':
                raise CalibrationError('Invalid torque readback after atomic hold')
            if self._read(motor, 40, 1) != b'\1':
                raise CalibrationError('Holding torque not confirmed')
            self.active_axis = None
            self.last_monitor = None
        return self._guarded(atomic)

    def keep(self):
        return self._guarded(lambda: None)

    def wait(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            time.sleep(min(.04, max(0, end - time.monotonic())))
            self.keep()

    def _commit_validated(self):
        runner = self.calibrator
        if not isinstance(runner, Calibrator) or runner.b is not self or set(runner.results) != set(MOTOR_IDS):
            raise CalibrationError('COMMIT requires the complete six-axis Calibrator.run result')
        labels = {'first boundary', 'second boundary', 'repeat first boundary', 'return second boundary'}
        for name, motor in MOTOR_IDS.items():
            actual_labels = {e['label'] for e in runner.endpoints if e['name'] == name and e['id'] == motor}
            if not labels <= actual_labels or runner.results[name]['id'] != motor:
                raise CalibrationError('COMMIT requires repeatable endpoints for every joint')
        candidate = json.loads((self.folder / 'candidate_calibration.json').read_text(encoding='utf8'))
        verified = json.loads((self.folder / 'after_write_readback.json').read_text(encoding='utf8'))
        if candidate != runner.results or {x['id'] for x in verified} != set(range(1, 7)) or len(verified) != 6:
            raise CalibrationError('COMMIT candidate/readback records incomplete')
        for name, motor in MOTOR_IDS.items():
            result = runner.results[name]
            data = self._read(motor, 9, 32)
            position = signed(self._health(motor)[:2], 15)
            if (signed(data[22:24], 11) != result['homing_offset']
                    or word(data[:2]) != result['range_min'] or word(data[2:4]) != result['range_max']
                    or data[24] != 0 or data[31] != 0
                    or not result['range_min'] - 15 <= position <= result['range_max'] + 15):
                raise CalibrationError(f'ID{motor}: COMMIT final state mismatch')

    def finish(self, commit):
        if self.committed:
            return ['USB COMMIT off_confirmed=1 rollback_success=1']
        if not commit and not self.active and self.cleanup_lines is not None:
            return self.cleanup_lines
        if commit:
            if self.fault or not self.active:
                raise CalibrationError('Faulted/inactive USB session cannot COMMIT')
            self._session()
            # End transient episodes before accepting a final COMMIT. Motion
            # has already been switched OFF by Calibrator.run at this stage.
            while self.temperature_guard is not None and self.temperature_guard.pending:
                self.wait(.04)
            self._commit_validated()
            if self.bus.log_error:
                raise CalibrationError('USB packet logging failed before COMMIT')
        # STOP ignores operator/log faults. First actions are TorqueOFF only.
        self.cleaning = True
        priority = ([self.active_axis] if self.active_axis else [])
        order = priority + [m for m in range(1, 7) if m not in priority]
        off = set()
        errors = []
        for _ in range(3):
            for motor in order:
                if motor in off:
                    continue
                try:
                    self._byte(motor, 40, 0)
                    if self._read(motor, 40, 1) != b'\0':
                        raise CalibrationError('TorqueOFF readback mismatch')
                    off.add(motor)
                except BaseException as error:
                    errors.append(f'ID{motor} OFF: {error}')
        restore_ok = set()
        for motor in order:
            if motor not in off:
                continue  # Never mode/speed/EEPROM write an unconfirmed axis.
            try:
                if motor not in self.original:
                    # An incomplete start never changed EEPROM or enabled
                    # torque. Stop only; no settings writes without a snapshot.
                    restore_ok.add(motor)
                    continue
                if motor in self.original:
                    self._byte(motor, 55, 0)
                self._byte(motor, 33, 0)
                if self._read(motor, 33, 1) != b'\0':
                    raise CalibrationError('Mode0 readback mismatch; skip Speed/EEPROM')
                self._write(motor, 46, b'\0\0')
                if self._read(motor, 46, 2) != b'\0\0':
                    raise CalibrationError('Speed0 readback mismatch')
                old = self.original.get(motor)
                if old is not None:
                    if not commit:
                        self._write(motor, 9, b'\0\0')
                    for address, size in ((9, 2), (11, 2), (16, 2), (31, 2), (34, 1), (35, 1), (36, 1)):
                        if not commit and address == 9:
                            continue
                        if commit and address in (9, 11, 31):
                            continue
                        self._write(motor, address, old[address:address + size])
                        if self._read(motor, address, size) != old[address:address + size]:
                            raise CalibrationError(f'EEPROM restore mismatch at {address}')
                    if not commit:
                        self._write(motor, 9, old[9:11])
                    # Safe RAM only; NEVER restore an old Goal42 or torque state.
                    self._byte(motor, 41, 5)
                    self._write(motor, 48, min(250 if motor in (5, 6) else 400,
                                             word(old[48:50])).to_bytes(2, 'little'))
                    self._byte(motor, 55, old[55])
                    if self._read(motor, 55, 1) != old[55:56]:
                        raise CalibrationError('Lock restore mismatch')
                    if not commit and (self._read(motor, 0, 32) + self._read(motor, 32, 8)) != old[:40]:
                        raise CalibrationError('Final EEPROM byte-for-byte mismatch')
                if self._read(motor, 40, 1) != b'\0':
                    raise CalibrationError('Final TorqueOFF readback mismatch')
                restore_ok.add(motor)
            except BaseException as error:
                errors.append(f'ID{motor} cleanup: {error}')
                # Even after a failed restore do not let cleanup re-enable.
                try:
                    self._byte(motor, 40, 0)
                except BaseException as off_error:
                    errors.append(f'ID{motor} final OFF: {off_error}')
                if motor in self.original:
                    try:
                        if self._read(motor, 40, 1) == b'\0':
                            self._byte(motor, 55, self.original[motor][55])
                            if self._read(motor, 55, 1) != self.original[motor][55:56]:
                                raise CalibrationError('Failure-path lock restore mismatch')
                    except BaseException as lock_error:
                        errors.append(f'ID{motor} lock cleanup: {lock_error}')
        confirmed = len(off) == 6 and len(restore_ok) == 6
        cleanup = {'action': 'COMMIT' if commit else 'STOP', 'off_confirmed': sorted(off),
                   'restored': sorted(restore_ok), 'errors': errors,
                   'confirmed': confirmed, 'fault': self.fault}
        self.cleanup_details = cleanup
        # The confirmed hardware state precedes any fallible output operation.
        if confirmed:
            self.active = False
            self.active_axis = None
            self.last_monitor = None
            self.committed = commit
            self.cleaning = False
            self.cleanup_lines = [f'USB {cleanup["action"]} off_confirmed=1 rollback_success=1']
        try:
            (self.folder / 'usb_cleanup.json').write_text(json.dumps(cleanup, indent=2), encoding='utf8')
        except BaseException as error:
            self.cleanup_output_error = repr(error)
            self.bus.record({'cleanup_output_error': repr(error), 'cleanup': cleanup})
        if not confirmed:
            self.fault = self.fault or 'USB cleanup not confirmed'
            raise CalibrationError('USB STOP/configuration cleanup NOT confirmed: ' + ' | '.join(errors))
        return self.cleanup_lines

    def close(self):
        try:
            if self.active:
                self.finish(False)
        finally:
            self.bus.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--inspect', action='store_true', help='Read baseline; no torque enable or movement')
    parser.add_argument('--velocity', type=int, default=100)
    parser.add_argument('--torque', type=int, default=400)
    parser.add_argument('--stop-file', type=Path, help='Additional parent STOP file for bounded retry runner')
    parser.add_argument('--temperature-debounce', action='store_true',
                        help='User policy: tolerate >45 C feedback only while continuous duration <=1.5 s')
    parser.add_argument('--discover-mechanical-range', action='store_true',
                        help='Manufacturer boundary discovery; old EEPROM soft ranges are references, all motion guards remain')
    args = parser.parse_args(argv)
    if not 50 <= args.velocity <= 300 or not 100 <= args.torque <= 400:
        parser.error('Velocity 50..300 and torque 100..400 required')
    folder = args.out or Path(__file__).resolve().parents[1] / 'results' / ('usb_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    if folder.exists():
        parser.error('Output folder exists; preserve old backups')
    folder.mkdir(parents=True)
    bridge = runner = None
    report = {'status': 'NOT_COMPLETED', 'port': args.port, 'transport': 'direct USB 1 Mbaud',
              'source': SOURCE_URL, 'hardware_watchdog': False,
              'real_calibration_complete': False, 'physical_alignment_verified': False,
              'geometry_verified': False}
    report['discover_mechanical_range'] = args.discover_mechanical_range
    try:
        bridge = Bridge(args.port, folder, temperature_debounce=args.temperature_debounce,
                        discover_mechanical_range=args.discover_mechanical_range)
        bridge.external_stop_path = args.stop_file
        bridge.start()
        runner = Calibrator(bridge, folder, args.velocity, args.torque, args.discover_mechanical_range)
        bridge.calibrator = runner
        data = runner.backup()
        data['transport'] = report['transport']
        (folder / 'before.json').write_text(json.dumps(data, indent=2), encoding='utf8')
        if args.inspect:
            report['cleanup'] = bridge.finish(False)
            report['status'] = 'INSPECT_PASS'
        else:
            report['calibration'] = runner.run()
            report['status'] = 'CALIBRATION_PASS'
            report['real_calibration_complete'] = True
        if bridge.cleanup_output_error:
            raise OSError('Cleanup report write failed: ' + bridge.cleanup_output_error)
    except BaseException as error:
        report['error'] = repr(error)
        if (runner and runner.commit_confirmed) or (bridge and bridge.committed):
            report['status'] = 'COMMITTED_OUTPUT_ERROR'
            report['cleanup'] = runner.commit_cleanup or bridge.cleanup_lines
            report['rollback_not_applicable'] = 'New EEPROM calibration already committed; candidate_calibration.json retained'
        elif bridge:
            try:
                report['cleanup'] = bridge.finish(False)
            except Exception as cleanup_error:
                report['cleanup_error'] = repr(cleanup_error)
        raise
    finally:
        if runner:
            report['commit_confirmed'] = runner.commit_confirmed or bool(bridge and bridge.committed)
            report['endpoint_records'] = runner.endpoints
        if bridge:
            report['temperature_policy'] = bridge.temperature_guard.report() if bridge.temperature_guard else {'mode': 'strict 55 C'}
            report['temperature_recheck'] = bridge.temperature_recheck
            report['cleanup_output_error'] = bridge.cleanup_output_error
            report['transport_log_error'] = bridge.bus.log_error
            try:
                bridge.close()
            except Exception as close_error:
                report['close_error'] = repr(close_error)
        (folder / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf8')
        print('Recorded ' + str(folder), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (CalibrationError, OSError, ValueError, KeyboardInterrupt) as error:
        raise SystemExit(str(error))
