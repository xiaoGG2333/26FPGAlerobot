"""Right Joy-Con -> UART target -> ARM full iteration; --drive enables verified arm motion."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
from finish_target import CompletionTracker

HOME_Q = [0.2, -0.3, 0.4, -0.2, 0.15]
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
CODES = ['CONVERGED', 'MAX_ITER', 'HARDWARE_ERROR', 'NUMERICAL_ERROR', 'JOINT_LIMIT']


def fk_pose(q):
    """Existing LeRobot ET model, used only for replay and coordinate reference."""
    tx = [.0612, .02943, .02798, .13504, .0593]
    tz = [.0598, .05504, .1127, .00519, .00996]
    R = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    p = [0., 0., 0.]
    for i, angle in enumerate(q):
        c, s = math.cos(angle), math.sin(angle)
        p = [p[j] + R[j][0]*tx[i] + R[j][2]*tz[i] for j in range(3)]
        A = ([[c, -s, 0], [s, c, 0], [0, 0, 1]] if i == 0 else
             [[1, 0, 0], [0, c, -s], [0, s, c]] if i == 4 else
             [[c, 0, s], [0, 1, 0], [-s, 0, c]])
        R = [[sum(R[j][k]*A[k][l] for k in range(3)) for l in range(3)] for j in range(3)]
    return p + [math.atan2(R[2][1], R[2][2]),
                math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0])),
                math.atan2(R[1][0], R[0][0])]


def wrap(v):
    return (v + math.pi) % (2*math.pi) - math.pi


class Mapper:
    def __init__(self, config, arm_pose, input_pose):
        self.config, self.arm = config, list(arm_pose)
        self.origin = list(input_pose)
        self.previous = list(arm_pose)

    def target(self, pose, dt):
        if len(pose) != 6 or not all(math.isfinite(float(x)) for x in pose):
            raise ValueError('Joy-Con pose must contain six finite numbers')
        c = self.config
        delta = [float(pose[i])-self.origin[i] for i in range(3)]
        xyz = [self.arm[i] + c['position_scale'] * sum(c['axis_matrix'][i][j]*delta[j]
                for j in range(3)) for i in range(3)]
        # Limit requested travel without pulling an out-of-box measured start
        # sideways. In particular a Z-only request must preserve X and Y.
        for i in range(3):
            if xyz[i] > self.arm[i]:
                xyz[i] = min(xyz[i], max(self.arm[i], c['workspace_max'][i]))
            elif xyz[i] < self.arm[i]:
                xyz[i] = max(xyz[i], min(self.arm[i], c['workspace_min'][i]))
        change = [xyz[i]-self.previous[i] for i in range(3)]
        length = math.sqrt(sum(x*x for x in change))
        cap = c['max_translation_m_s'] * dt
        if length > cap:
            xyz = [self.previous[i] + change[i]*cap/length for i in range(3)]
        roll = self.arm[3] + c['rotation_scale'] * wrap(float(pose[3])-self.origin[3])
        pitch = self.arm[4] + c['rotation_scale'] * wrap(float(pose[4])-self.origin[4])
        # Five axes cannot independently satisfy arbitrary xyz + roll/pitch/yaw.
        # Existing model's shoulder origin is x=.0612; align yaw with shoulder plane.
        yaw = math.atan2(xyz[1], xyz[0]-.0612)
        # Euler yaw may differ from the shoulder plane by pi when the
        # physical pitch passes 90 degrees. Preserve the measured branch.
        yaw = min((wrap(yaw), wrap(yaw+math.pi)),
                  key=lambda value: abs(wrap(value-self.arm[5])))
        rpy = [wrap(roll), max(-1.4, min(1.4, pitch)), yaw]
        for i in (0, 1):
            d = wrap(rpy[i]-self.previous[i+3])
            cap = c['max_rotation_rad_s'] * dt
            rpy[i] = wrap(self.previous[i+3] + max(-cap, min(cap, d)))
        # STATE1 can report an actual pitch outside the POSE1 envelope.
        # Slewing from that reference must never produce an invalid frame.
        rpy[1] = max(-1.4, min(1.4, rpy[1]))
        return xyz + rpy

    def commit(self, target):
        self.previous = list(target)


def rollback_input(source,mapper,pose):
    # Preserve the last accepted Cartesian endpoint and discard only unexecuted input.
    anchor=list(mapper.previous)
    if hasattr(source,'integrator'):
        source.integrator.pose=[0.]*6
        source.previous=time.monotonic()
        pose=list(source.integrator.pose)
    return Mapper(mapper.config,anchor,pose)


def encode(seq, core, pose, gripper):
    values = list(pose) + [gripper]
    if not 0 < seq <= 0xffffffff or core not in (0, 1):
        raise ValueError('Invalid sequence/core')
    if len(values) != 7 or not all(math.isfinite(float(x)) for x in values):
        raise ValueError('Non-finite target')
    return ('POSE1 %d %d ' % (seq, core) + ' '.join('%.9g' % x for x in values) + '\n').encode('ascii')


def parse_result(line, seq):
    fields = line.split()
    if len(fields) != 12 or fields[0] != 'RESULT1' or int(fields[1]) != seq:
        raise ValueError('Wrong RESULT1 frame/sequence')
    code, iterations = int(fields[2]), int(fields[3])
    values = [float(x) for x in fields[4:]]
    if code not in range(5) or not 0 <= iterations <= 200 or not all(math.isfinite(x) for x in values):
        raise ValueError('Invalid result')
    if values[0] < 0 or values[1] < 0 or not 0 <= values[-1] <= 1:
        raise ValueError('Invalid residual/gripper')
    return dict(seq=seq, code=code, status=CODES[code], iterations=iterations,
                position_error=values[0], rotation_error=values[1], q=values[2:7], gripper=values[7])


class UART:
    def __init__(self, port, timeout):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=.1, write_timeout=2)
        self.timeout = timeout
        self.seq = 0
        self.motion_expected = False

    def check_stop_event(self, line, cleanup=False):
        # Asynchronous stop events take precedence over any pending reply.
        # STOP cleanup must still consume the event and confirm torque-off.
        if (getattr(self,'motion_expected',False) and not cleanup
                and line.startswith('ARM1 EVENT ') and 'active=0' in line.split()):
            raise RuntimeError('Board stopped arm: '+line)
        if (getattr(self,'motion_expected',False) and not cleanup
                and line.startswith('ARM1 EVENT ') and 'active=1' in line.split()):
            raise RuntimeError('Board frozen arm: '+line)

    def exchange(self, request, prefix, seq=None):
        deadline = time.monotonic() + self.timeout
        diagnostic = request.startswith((b'HELLO', b'STATE1'))
        if diagnostic:
            print('[UART] Sending ' + request.decode('ascii').strip(), flush=True)
        written = self.serial.write(request)
        if written != len(request):
            raise IOError('Incomplete UART write')
        # Do not call flush()/tcdrain(): USB passthrough can block there outside
        # pyserial's write_timeout. Reading the reply confirms transmission.
        if diagnostic:
            print('[UART] Sent; waiting for ' + prefix.strip(), flush=True)
        buffer = bytearray()
        while time.monotonic() < deadline:
            buffer.extend(self.serial.read_until(b'\n', size=512))
            if len(buffer) > 4096:
                raise ValueError('Overlong UART response')
            if not buffer.endswith(b'\n'):
                continue
            # Opening USB UART after boot can leave NUL padding around the
            # first frame. Trim boundary padding only, never repair payloads.
            line = buffer.decode('ascii', errors='replace').strip('\x00 \t\r\n')
            buffer.clear()
            self.check_stop_event(line)
            if line.startswith('ERR1 '):
                raise RuntimeError(line)
            fields = line.split()
            expected = prefix.split()
            if fields[:len(expected)] == expected and (seq is None or
                    (len(fields) > 1 and fields[1] == str(seq))):
                if diagnostic:
                    print('[UART] Received ' + line, flush=True)
                return line
            if line:
                print('BOARD:', repr(line), flush=True)
        raise TimeoutError('No matching reply: verify POSE1 firmware, PS init, FPGA and port')

    def state(self, reset=False):
        if reset:
            self.seq += 1
        line = self.exchange(('%s %d\n' % ('RESET1' if reset else 'STATE1', self.seq)).encode(), 'STATE1 ', self.seq)
        fields = line.split()
        if len(fields) != 14:
            raise ValueError('Wrong STATE1 firmware version')
        v = [float(x) for x in fields[2:13]]
        if not all(math.isfinite(x) for x in v):
            raise ValueError('Invalid state')
        self.seq = int(fields[13])
        return v[:5], v[5:]

    def solve(self, core, target, gripper):
        self.seq += 1
        started = time.monotonic()
        line = self.exchange(encode(self.seq, core, target, gripper), 'RESULT1 ', self.seq)
        result = parse_result(line, self.seq)
        result['roundtrip_ms'] = (time.monotonic()-started)*1000
        return result

    def close(self):
        self.serial.close()

    def arm(self,command,expected):
        self.serial.write((command+'\n').encode('ascii'))
        deadline=time.monotonic()+self.timeout
        buffer=bytearray();response=None
        while time.monotonic()<deadline:
            buffer.extend(self.serial.read_until(b'\n',size=512))
            if len(buffer)>4096:raise ValueError('Overlong ARM reply')
            if not buffer.endswith(b'\n'):continue
            line=buffer.decode('ascii',errors='replace').strip('\x00 \t\r\n');buffer.clear()
            self.check_stop_event(line,cleanup=command=='ARM1 STOP')
            fields=line.split()
            if len(fields)>=2 and fields[0]=='ARM1':
                if command=='ARM1 STOP' and fields[1]=='EVENT':
                    continue
                if fields[1] in (expected,'BLOCKED','ERROR','EVENT','FROZEN'):response=line
                elif fields[1]=='END' and response:
                    if response.startswith('ARM1 READY '):self.motion_expected=True
                    if command=='ARM1 STOP' and 'torque_off=1' in response.split():self.motion_expected=False
                    return response
        raise TimeoutError('ARM command not confirmed; no retry: '+command)

    def counts(self):
        line=self.exchange(('COUNTS1 %d\n'%self.seq).encode(),'COUNT1 ',self.seq)
        fields=line.split()
        if len(fields)!=10:raise ValueError('Wrong COUNT1 firmware version')
        values=[int(x) for x in fields[2:]]
        if values[0]!=0:raise ValueError('Board model-to-encoder mapping rejected stored state')
        if values[-1] not in (0,1):raise ValueError('Invalid mapping verification flag')
        if any(x<0 or x>4095 for x in values[1:7]):raise ValueError('Invalid encoder count preview')
        return values[1:7],values[-1]


class RightJoyCon:
    def __init__(self):
        try:
            from joyconrobotics import JoyconRobotics
        except ImportError as e:
            raise RuntimeError('Install joycon-robotics in Ubuntu; see README. Replay requires no Joy-Con.') from e
        self.device = JoyconRobotics(device='right',
            offset_position_m=[0., 0., 0.],
            glimit=[[-.4, -.4, -.4, -1.4, -1.4, -3.14], [.4, .4, .4, 1.4, 1.4, 3.14]],
            limit_dof=True, dof_speed=[.2]*6, all_button_return=True)
        time.sleep(.5)

    def read(self, elapsed):
        del elapsed
        if not self.device.thread.is_alive():
            raise RuntimeError('Joy-Con processing thread stopped')
        pose, gripper, button = self.device.get_control()
        return list(pose), float(gripper), int(button), bool(self.device.joycon.get_button_home())

    def close(self):
        self.device.running = False
        self.device.thread.join(timeout=2)
        self.device.disconnnect()  # Upstream intentionally spells this with three n's.


class Replay:
    def read(self, elapsed):
        # Changing reachable targets; this is input generation, never an IK answer.
        q = [HOME_Q[0]+.04*math.sin(elapsed), HOME_Q[1]-.03*math.sin(elapsed),
             HOME_Q[2]+.04*math.sin(elapsed), HOME_Q[3]-.02*math.sin(elapsed),
             HOME_Q[4]+.03*math.sin(elapsed)]
        return fk_pose(q), .5, 0, False

    def close(self):
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', choices=['replay', 'joycon', 'buttons', 'stick', 'xbox'], default='replay')
    ap.add_argument('--port', help='COM16 on Windows, /dev/ttyUSB0 on Ubuntu')
    ap.add_argument('--offline', action='store_true', help='Print target packets only; no IK/FPGA verification')
    ap.add_argument('--drive',action='store_true',help='Require ARM1 ENABLE; send pose targets to closed-loop arm controller')
    ap.add_argument('--test-drive',action='store_true',help='10-second, +/-56-count supported trial; mapping remains unverified')
    ap.add_argument('--manual-control',action='store_true',help='With --test-drive: wait for Enter before enabling; Ctrl+C stops; board protections remain active')
    ap.add_argument('--core', type=int, choices=[0, 1], default=0)
    ap.add_argument('--hz', type=float, default=5.)
    ap.add_argument('--translation-speed', type=float, default=.02,
                    help='Manual stick speed in m/s, default 0.02, maximum 0.05; board limits still apply')
    ap.add_argument('--max-follow-error', type=int, default=24,
                    help='Stop when any solved joint target differs from measured feedback by more than this many counts (1..32)')
    ap.add_argument('--count', type=int, default=0, help='0 = until Ctrl+C')
    ap.add_argument('--timeout', type=float, default=30.)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('right_config.json'))
    ap.add_argument('--csv', type=Path, default=Path('right_joycon_results.csv'))
    ap.add_argument('--no-count-preview',action='store_true',help='Skip new mapped firmware COUNTS1 query')
    ap.add_argument('--xbox-device', help='Linux /dev/input/eventN; auto-select one Xbox if omitted')
    ap.add_argument('--xbox-deadzone', type=float, default=.15)
    ap.add_argument('--xbox-right-axes', choices=['auto','rxry','zrz'], default='auto')
    args = ap.parse_args(argv)
    if not 0 < args.hz <= 20 or args.count < 0 or args.timeout <= 0:
        ap.error('Require 0 < hz <= 20, count >= 0, timeout > 0')
    if not math.isfinite(args.translation_speed) or not 0 < args.translation_speed <= .05:
        ap.error('--translation-speed must be in (0, 0.05] m/s')
    if not 1 <= args.max_follow_error <= 32:
        ap.error('--max-follow-error must be 1..32 counts')
    if not args.offline and not args.port:
        ap.error('--port required unless --offline')
    if args.drive and (args.offline or args.hz<5):
        ap.error('--drive requires real UART and 5..20 Hz')
    if args.drive and args.no_count_preview:
        ap.error('--drive requires mapping/arm state verification; remove --no-count-preview')
    if args.test_drive:
        if args.drive or args.source not in ('joycon','buttons','stick','xbox') or args.offline or args.no_count_preview or not (1<=args.count<=20 or args.manual_control and args.count==0) or not 5<=args.hz<=10:
            ap.error('--test-drive requires JoyCon, real UART, --count 1..20, --hz 5..10, count/feedback checks, and no --drive')
        args.drive=True
    if args.manual_control and not args.test_drive:
        ap.error('--manual-control requires --test-drive')
    if args.source in ('buttons','stick','xbox') and args.drive and not args.manual_control:
        ap.error('Buttons motion requires --test-drive --manual-control')
    config = json.loads(args.config.read_text(encoding='utf-8'))
    if args.test_drive:
        config['max_translation_m_s']=min(config['max_translation_m_s'],args.translation_speed if args.manual_control and args.source in ('stick','xbox') else .005 if args.manual_control else .001)
        config['max_rotation_rad_s']=min(config['max_rotation_rad_s'],.10 if args.manual_control else .03)
    source = uart = None
    armed=False
    enable_attempted=False
    try:
        arm_pose = fk_pose(HOME_Q)
        if not args.offline:
            print('[UART] Opening ' + args.port + ' at 115200 8N1', flush=True)
            uart = UART(args.port, args.timeout)
            print('[UART] Port opened', flush=True)
            ready=uart.exchange(b'HELLO\n', 'POSE1 READY ')
            if 'SO101_135MM' not in ready.split():
                raise RuntimeError('135mm host requires matching SO101_135MM firmware; update the board with this package BOOT.BIN')
            if 'MANUAL_REFERENCE_20261009_V1' not in ready.split():
                raise RuntimeError('Manual reference host requires MANUAL_REFERENCE_20261009_V1 BOOT.BIN; do not mix old firmware and new mapping')
            if args.test_drive and 'LIMITED_DRIVE_V1' not in ready:raise RuntimeError('New trial BOOT.BIN required')
            if args.manual_control and 'MANUAL_OUT_HOLD_V2' not in ready.split():raise RuntimeError('Manual mode requires new MANUAL_OUT_HOLD_V2 BOOT.BIN')
            if args.source=='buttons' and args.drive and 'BUTTONS_INPUT_V1' not in ready.split():raise RuntimeError('Buttons motion requires matching BUTTONS_INPUT_V1 BOOT.BIN')
            if args.source=='stick' and args.drive and 'STICK_INPUT_V1' not in ready.split():raise RuntimeError('Stick motion requires matching STICK_INPUT_V1 BOOT.BIN')
            if args.drive and 'ARM_CONTROL_V1' not in ready:raise RuntimeError('New ARM_CONTROL_V1 firmware required')
            if args.manual_control:
                coordinated='COORDINATED_MANUAL_V1' in ready.split()
                if args.manual_control and 'LIMITS600_TEMP1500_20261011_V1' not in ready.split():
                    raise RuntimeError('This host requires LIMITS600_TEMP1500_20261011_V1 BOOT.BIN; update SD first')
                if args.manual_control and 'SYNC_TARGET_HOLD_V1' not in ready.split():
                    raise RuntimeError('This host requires SYNC_TARGET_HOLD_V1 BOOT.BIN; update SD first')
                print('Board coordinated goal control: '+('YES' if coordinated else 'NO; rebuild/deploy firmware for coordinated stepping. Host following-error stop is active.'),flush=True)
            if not args.drive:_, arm_pose = uart.state()
        if args.source=='xbox':
            from xbox_input import XboxController
            source=XboxController(args.xbox_device, args.xbox_deadzone, args.xbox_right_axes, config['max_translation_m_s'], config['max_rotation_rad_s'])
        elif args.source=='stick':
            from stick_input import StickJoyCon
            source=StickJoyCon()
            if hasattr(source, 'integrator'):
                source.integrator.speed=config['max_translation_m_s']
        elif args.source=='buttons':
            from button_input import ButtonJoyCon
            source=ButtonJoyCon()
        else:
            source = Replay() if args.source == 'replay' else RightJoyCon()
        first, _, _, _ = source.read(0.)
        if args.drive:
            checked=uart.arm('ARM1 TEST_MANUAL_CHECK' if args.manual_control else 'ARM1 TEST_CHECK' if args.test_drive else 'ARM1 CHECK','CHECK_OK')
            if not checked.startswith('ARM1 CHECK_OK '):
                print(uart.arm('ARM1 STATE','STATE'),flush=True)
                raise RuntimeError('Arm preflight failed: '+checked)
            starting_q,arm_pose=uart.state()
            if args.manual_control:
                mapping=json.loads(Path(__file__).with_name('right_mapping.json').read_text(encoding='utf-8'))
                for name,angle in zip(mapping['joint_order'],starting_q):
                    joint=mapping['joint_mapping'][name]
                    if not joint['q_min_rad'] <= angle <= joint['q_max_rad']:
                        raise RuntimeError('Starting model angle outside limits: ID%d (%s) %.2f deg, allowed %.2f..%.2f deg. Verify the reference mapping or reposition with torque off before enabling.' % (joint['id'],name,math.degrees(angle),math.degrees(joint['q_min_rad']),math.degrees(joint['q_max_rad'])))
            if args.manual_control and abs(arm_pose[4]) > 1.4:
                raise RuntimeError('Measured pitch exceeds POSE1 +/-1.4 rad. Reposition the arm with torque off before starting; clamping would change orientation during a translation request.')
            if not args.manual_control and (any(not config['workspace_min'][i]<=arm_pose[i]<=config['workspace_max'][i] for i in range(3)) or abs(arm_pose[4])>1.4):
                raise RuntimeError('Measured starting pose is outside configured workspace; no torque enabled')
            if args.manual_control:
                print('UART CONNECTED; MOTION OFF. Enter starts, Ctrl+C cancels/stops. Manual mode: no local travel envelope; model/encoder limits and heartbeat required.',flush=True)
                input('Press Enter to start: ')
                # Capture the current JoyCon origin after the operator wait.
                first, _, _, _ = source.read(0.)
            enable_attempted=True
            enabled=uart.arm('ARM1 TEST_MANUAL_ENABLE' if args.manual_control else 'ARM1 TEST_ENABLE' if args.test_drive else 'ARM1 ENABLE','READY')
            if not enabled.startswith('ARM1 READY '):raise RuntimeError('Arm motion not enabled: '+enabled)
            armed=True
            print(enabled,flush=True)
            _,arm_pose=uart.state()
        mapper = Mapper(config, arm_pose, first)
        print('RELEASE_FINISH_TARGET_V2 STOP_EVENT_GUARD_V1: neutral input finishes the last valid goal; Ctrl+C cancels/stops.',flush=True)
        print('RIGHT SO101 | '+('MANUAL NO LOCAL ENVELOPE; MAPPING UNVERIFIED; GRIPPER OFF' if args.manual_control and armed else 'LIMITED TRIAL +/-56 COUNTS; MAPPING UNVERIFIED; GRIPPER OFF' if args.test_drive and armed else 'ARM SERVO MOTION ENABLED' if armed else 'NO SERVO COMMANDS')+' | ' + ('OFFLINE PACKETS ONLY' if args.offline else 'REAL FPGA + ARM ITERATION'))
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['elapsed_s', 'seq', 'status', 'iterations', 'position_error_m', 'rotation_error_rad',
                             'roundtrip_ms'] + ['target_'+x for x in ['x','y','z','roll','pitch','yaw']] + NAMES + ['gripper_preview','button']+
                            ['count_id%d'%i for i in range(1,7)]+['count_status','physical_alignment_verified']+
                            ['actual_count_id%d'%i for i in range(1,6)]+(['arm_active','xbox_lx','xbox_ly','xbox_rx','xbox_ry'] if args.source=='xbox' else ['arm_active','stick_h','stick_v','stick_press','button_r']))
            start = previous_time = time.monotonic()
            index, last_home, idle_reanchored = 0, False, False
            completion = CompletionTracker()
            while not args.count or index < args.count:
                try:
                    now = time.monotonic()
                    dt = min(max(now-previous_time, 1/args.hz), .5)
                    previous_time = now
                    pose, gripper, button, home = source.read(now-start)
                    raw_input=list(source.report) if args.source in ('stick','xbox') and getattr(source,'report',None) is not None else ['']*4
                    if home and not last_home:
                        if uart:
                            # Re-center on measured joints while driving. RESET1 is
                            # reserved for preview and is rejected by an active arm.
                            _, arm_pose = uart.state(reset=not armed)
                        else:
                            arm_pose = fk_pose(HOME_Q)
                        mapper = Mapper(config, arm_pose, pose)
                    last_home = home
                    if args.source in ('buttons','stick','xbox') and armed:
                        direction_mask=15 if button&(1<<6) else 63
                        moving=bool(button) if args.source in ('stick','xbox') else bool(button&(1<<7) and button&direction_mask)
                        if not moving:
                            # Neutral input stops generating poses; it does not
                            # cancel the last valid goal already queued in Zynq.
                            alive=uart.arm('ARM1 KEEP','ALIVE')
                            if not alive.startswith('ARM1 ALIVE '):raise RuntimeError('Arm session stopped: '+alive)
                            current=uart.arm('ARM1 STATE','STATE')
                            if not current.startswith('ARM1 STATE active=1 mode=3 aligned=0 geometry=0 '):raise RuntimeError('Arm stopped during target completion: '+current)
                            fields=dict(f.split('=',1) for f in current.split() if '=' in f)
                            if fields.get('torque_off')!='0':raise RuntimeError('Torque state invalid during completion: '+current)
                            actual=[int(v) for v in fields.get('counts','').split(',') if v]
                            goal=[int(v) for v in fields.get('goal','').split(',') if v]
                            reached,gap=completion.observe(actual,goal)
                            target=list(mapper.previous)
                            # Re-anchor once, after three consecutive observations
                            # put all five joints within eight counts of the goal.
                            # STATE1 refreshes encoder feedback; it issues no goals.
                            q,arm_pose=uart.state()
                            if reached and not idle_reanchored:
                                mapper=Mapper(config,arm_pose,pose)
                                idle_reanchored=True
                            status=('XBOX_' if args.source=='xbox' else 'STICK_' if args.source=='stick' else 'BUTTON_')+('HOLD' if reached else 'SETTLING')
                            print(status+' | TORQUE ON | max_goal_gap=%d counts | actual_counts: '%gap+' '.join('ID%d=%d'%(i,c) for i,c in enumerate(actual,1))+' | final_goal: '+' '.join('ID%d=%d'%(i,c) for i,c in enumerate(goal,1)),flush=True)
                            writer.writerow([now-start,uart.seq,status,0,'','','']+target+q+[gripper,button]+goal+['']+['HOLD_REACHED' if reached else 'FINISH_LAST_TARGET',0]+actual+[1]+raw_input)
                            stream.flush();index+=1
                            time.sleep(max(0.,1/args.hz-(time.monotonic()-now)))
                            continue
                        completion.reset()
                        idle_reanchored=False
                    target = mapper.target(pose, dt)
                    if args.offline:
                        print(encode(index+1, args.core, target, gripper).decode().strip())
                        mapper.commit(target)
                        writer.writerow([now-start,index+1,'OFFLINE', '', '', '', '']+target+['']*5+[gripper,button]+['']*14+raw_input)
                    else:
                        if armed:
                            alive=uart.arm('ARM1 KEEP','ALIVE')
                            if not alive.startswith('ARM1 ALIVE '):raise RuntimeError('Arm session stopped: '+alive)
                        result = uart.solve(args.core, target, gripper)
                        if armed and result['code']!=0 and not args.manual_control:raise RuntimeError('IK failed; firmware stopped arm: '+result['status'])
                        if armed and args.manual_control and result['code']!=0:
                            current=uart.arm('ARM1 STATE','STATE')
                            if not current.startswith('ARM1 STATE active=1 mode=3 aligned=0 geometry=0 '):raise RuntimeError('Arm stopped: '+current)
                            print('LeRobot ERROR: rejected; last verified goal retained; input accumulation discarded.',flush=True)
                            mapper=rollback_input(source,mapper,pose)
                        if result['code'] == 0:
                            mapper.commit(target)
                        print('seq=%d %s iter=%d pos=%.3g m rot=%.3g rad roundtrip=%.1f ms' %
                              (result['seq'],result['status'],result['iterations'],result['position_error'],result['rotation_error'],result['roundtrip_ms']))
                        label = 'q_deg' if result['code'] == 0 else 'candidate_q_deg (FAILED)'
                        print(label + ': ' + ' '.join('%s=%+.3f' % (n, math.degrees(q)) for n,q in zip(NAMES,result['q'])))
                        counts=['']*6;count_status='SKIPPED';verified='';actual=['']*5;arm_active='';following_error=None
                        if armed and args.manual_control and result['code']!=0:
                            feedback=[f for f in current.split() if f.startswith('counts=')]
                            if len(feedback)!=1:raise ValueError('Missing OUT hold encoder feedback')
                            actual=[int(v) for v in feedback[0].split('=',1)[1].split(',')]
                            if len(actual)!=5 or any(v<0 or v>4095 for v in actual):raise ValueError('Invalid OUT hold encoder feedback')
                            count_status='REJECTED_KEEP_LAST_GOAL';verified=0;arm_active=1
                            print('actual_counts (REJECTED): '+' '.join('ID%d=%d'%(i,c) for i,c in enumerate(actual,1)),flush=True)
                        if result['code']==0 and not args.no_count_preview:
                            counts,verified=uart.counts();count_status='LIMITED_TRIAL_UNVERIFIED' if args.test_drive and armed else 'SERVO_TARGET' if armed else 'PREVIEW_ONLY'
                            if armed:
                                if verified!=(0 if args.test_drive else 1):raise RuntimeError('Firmware reports an unverified mapping during arm motion')
                                current=uart.arm('ARM1 STATE','STATE')
                                expected_state='ARM1 STATE active=1 mode=3 aligned=0 geometry=0 ' if args.test_drive else 'ARM1 STATE active=1 mode=2 aligned=1 geometry=1 '
                                if not current.startswith(expected_state):raise RuntimeError('Arm stopped or alignment invalid: '+current)
                                feedback=[f for f in current.split() if f.startswith('counts=')]
                                if len(feedback)!=1:raise ValueError('Missing actual encoder counts')
                                actual=[int(v) for v in feedback[0].split('=',1)[1].split(',')]
                                if len(actual)!=5 or any(v<0 or v>4095 for v in actual):raise ValueError('Invalid actual encoder counts')
                                arm_active=1
                                errors=[counts[i]-actual[i] for i in range(5)]
                                worst=max(range(5),key=lambda i:abs(errors[i]))
                                if abs(errors[worst])>args.max_follow_error:
                                    if args.manual_control and coordinated:
                                        # COUNTS1 is the queued final goal, not the command
                                        # currently issued by the firmware's ramp planner.
                                        # Board-side feedback lead/stall checks remain active.
                                        print('QUEUED GOAL GAP: ID%d goal=%d actual=%d gap=%+d counts; board synchronized goals active; no-progress shutdown disabled.' % (worst+1,counts[worst],actual[worst],errors[worst]),flush=True)
                                    else:
                                        result['status']='FOLLOW_ERROR_ID%d'%(worst+1)
                                        following_error='Joint following error: ID%d (%s) target=%d actual=%d error=%+d counts. Stop and inspect goal/feedback, torque and load.' % (worst+1,NAMES[worst],counts[worst],actual[worst],errors[worst])
                                print('actual_counts: '+' '.join('ID%d=%d'%(i,c) for i,c in enumerate(actual,1)),flush=True)
                            print('target_counts ('+('QUEUED ARM TARGET; GRIPPER PREVIEW' if armed else 'PREVIEW ONLY')+'): '+' '.join('ID%d=%d'%(i,c) for i,c in enumerate(counts,1)),flush=True)
                        writer.writerow([now-start,result['seq'],result['status'],result['iterations'],result['position_error'],
                            result['rotation_error'],result['roundtrip_ms']]+target+result['q']+[gripper,button]+counts+[count_status,verified]+actual+[arm_active]+raw_input)
                        if following_error:
                            stream.flush()
                            raise RuntimeError(following_error)
                    stream.flush()
                    index += 1
                    time.sleep(max(0., 1/args.hz-(time.monotonic()-now)))
                except (RuntimeError, OSError, ValueError) as error:
                    if not (armed and args.manual_control and args.source=='xbox') or 'Board stopped arm:' in str(error) or 'active=0' in str(error):raise
                    print('CONTROL FROZEN; torque-off not requested:',error,flush=True)
                    try:uart.arm('ARM1 FREEZE','FROZEN')
                    except Exception as fault:print('Freeze unconfirmed:',fault,flush=True)
                    while True:
                        input('Reconnect controller/UART, centre sticks. Enter resumes; Ctrl+C stops: ')
                        try:
                            if args.source=='xbox':
                                source.close()
                                source=XboxController(args.xbox_device,args.xbox_deadzone,args.xbox_right_axes,config['max_translation_m_s'],config['max_rotation_rad_s'])
                            # Reopen UART after unplug/replug; do not auto-enable torque.
                            if uart:uart.close()
                            uart=UART(args.port,args.timeout)
                            hello=uart.exchange(b'HELLO\n','POSE1 READY')
                            if 'LIMITS600_TEMP1500_20261011_V1' not in hello:raise RuntimeError('Matching firmware required')
                            uart.motion_expected=True
                            _,actual_pose=uart.state()
                            resumed=uart.arm('ARM1 RESUME','RESUMED')
                            if not resumed.startswith('ARM1 RESUMED '):raise RuntimeError(resumed)
                            pose,gripper,button,home=source.read(0)
                            mapper=Mapper(config,actual_pose,pose)
                            completion.reset();idle_reanchored=False;previous_time=time.monotonic()
                            print('CONTROL RESUMED from measured pose',flush=True)
                            break
                        except (RuntimeError,OSError,ValueError) as retry:print('Still frozen:',retry,flush=True)
    finally:
        if enable_attempted and uart:
            try:
                stopped=uart.arm('ARM1 STOP','STOP')
                print(stopped,flush=True)
                if 'torque_off=1' not in stopped:print('Torque-off NOT confirmed; inspect servo power/bus.',flush=True)
            except Exception as error:print('STOP not confirmed:',error,flush=True)
        if source:
            source.close()
        if uart:
            uart.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Stopped; check the ARM1 STOP reply if --drive was used.')
    except (RuntimeError,TimeoutError,OSError,ValueError) as error:
        raise SystemExit(str(error))
