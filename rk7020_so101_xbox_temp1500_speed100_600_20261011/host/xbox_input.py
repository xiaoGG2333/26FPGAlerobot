"""Linux Xbox absolute axes -> model XYZ/roll. Buttons and triggers do not move the arm."""
import argparse
import math
import time

LEFT=(0,1)  # ABS_X, ABS_Y
RIGHT_PAIRS={'rxry':(3,4), 'zrz':(2,5)}

def axis_value(value, minimum, maximum, deadzone=.15):
    if not 0 <= deadzone < .9 or maximum <= minimum:
        raise ValueError('Invalid axis range/deadzone')
    if not all(math.isfinite(float(v)) for v in (value,minimum,maximum)):
        raise ValueError('Non-finite Xbox axis')
    if not minimum <= value <= maximum:
        raise ValueError('Xbox axis outside advertised range')
    centre=(minimum+maximum)/2
    raw=(value-centre)/((maximum-minimum)/2)
    if abs(raw)<=deadzone:return 0.
    return math.copysign((abs(raw)-deadzone)/(1-deadzone),raw)

class XboxPose:
    def __init__(self, speed=.02, rotation_speed=.1):
        if not math.isfinite(speed) or not 0 < speed <= .05:
            raise ValueError('Translation speed must be in (0, .05] m/s')
        if not math.isfinite(rotation_speed) or not 0 < rotation_speed <= .5:
            raise ValueError('Rotation speed must be in (0, .5] rad/s')
        self.speed=speed;self.rotation_speed=rotation_speed;self.pose=[0.]*6
    def update(self,lx,ly,rx,ry,dt):
        if not all(math.isfinite(v) and -1<=v<=1 for v in (lx,ly,rx,ry)) or not math.isfinite(dt):
            raise ValueError('Invalid Xbox input')
        if dt<0:raise ValueError('Negative input time')
        dt=min(dt,.2)
        directions=[-ly,lx,-ry]
        length=max(1.,math.sqrt(sum(v*v for v in directions)))
        for i,d in enumerate(directions):self.pose[i]+=self.speed*d*dt/length
        # +model Rx = CCW viewed from jaw tip toward the base, current ID5 sign=-1.
        self.pose[3]+=self.rotation_speed*rx*dt
        mask=sum(1<<(2*i+(d<0)) for i,d in enumerate(directions) if d)
        if rx:mask|=1<<(6+(rx<0))
        return list(self.pose),.5,mask,False

def axis_codes(device):
    # evdev defaults to absinfo=True: EV_ABS entries are (code, AbsInfo)
    # tuples, not integers. Explicitly request codes for axis selection.
    return set(device.capabilities(absinfo=False).get(3,[]))

def pick_axes(device, mode='auto'):
    codes=axis_codes(device)
    if not set(LEFT)<=codes:raise RuntimeError('Selected device has no left stick ABS_X/Y')
    choices=RIGHT_PAIRS.items() if mode=='auto' else [(mode,RIGHT_PAIRS[mode])]
    for name,pair in choices:
        if set(pair)<=codes:return LEFT+pair,name
    raise RuntimeError('Right stick axes not found; inspect --list and choose --right-axes')

def devices():
    try:
        from evdev import InputDevice,list_devices
    except ImportError as e:raise RuntimeError('Install evdev inside the active environment: python -m pip install evdev') from e
    result=[]
    for path in list_devices():
        try:device=InputDevice(path)
        except PermissionError:continue
        try:
            pick_axes(device)
            # Wired Xbox may expose the name "Microsoft Controller" instead
            # of "Xbox". Vendor 045e plus gamepad buttons avoids name-only filtering.
            caps=device.capabilities(absinfo=False)
            vendor=getattr(getattr(device,'info',None),'vendor',None)
            is_gamepad=304 in caps.get(1,[])  # BTN_SOUTH / BTN_GAMEPAD
            if ('xbox' in device.name.lower() or 'x-box' in device.name.lower()
                    or vendor==0x045e and is_gamepad):result.append(device)
            else:device.close()
        except RuntimeError:device.close()
    return result

class XboxController:
    def __init__(self,path=None,deadzone=.15,right_axes='auto',speed=.02,rotation_speed=.1,require_neutral=True):
        if not math.isfinite(deadzone) or not 0<=deadzone<.9:raise ValueError('Invalid Xbox deadzone')
        self.device=None
        try:
            if path:
                try:from evdev import InputDevice
                except ImportError as e:raise RuntimeError('Install evdev: python -m pip install evdev') from e
                self.device=InputDevice(path)
            else:
                found=devices()
                if len(found)!=1:
                    for d in found:d.close()
                    raise RuntimeError('Expected one accessible Xbox controller. Connect/assign it to Ubuntu, check input permissions, or specify --xbox-device /dev/input/eventN')
                self.device=found[0]
            self.axes,self.axis_mode=pick_axes(self.device,right_axes)
            self.deadzone=deadzone;self.integrator=XboxPose(speed,rotation_speed)
            self.previous=time.monotonic();self.report=self._axes()
            if require_neutral and any(self.report):raise RuntimeError('Centre both Xbox sticks before starting')
            print('XBOX:',self.device.path,self.device.name,'right_axes='+self.axis_mode,flush=True)
            print('Left: up/down=X +/-; left/right=Y -/+. Right: up/down=Z +/-; left=CW/right=CCW viewed from jaw tip. Other controls ignored. Ctrl+C stops.',flush=True)
        except BaseException:
            self.close();raise
    def _axes(self):
        result=[]
        for axis in self.axes:
            info=self.device.absinfo(axis)
            if info is None:raise RuntimeError('Xbox axis disappeared')
            result.append(axis_value(info.value,info.min,info.max,self.deadzone))
        return tuple(result)
    def read(self,elapsed):
        del elapsed
        # Read current kernel axis state every tick: a held stick need not emit
        # repeated events. A removed USB device raises OSError -> host STOP.
        # Drain pending events without changing device configuration or grabbing it.
        try:
            for _ in self.device.read():pass
        except BlockingIOError:pass
        self.report=self._axes()
        now=time.monotonic();dt=now-self.previous;self.previous=now
        return self.integrator.update(*self.report,dt)
    def close(self):
        if self.device is not None:self.device.close();self.device=None

def main():
    p=argparse.ArgumentParser(description='List/monitor Xbox input only; no FPGA or motor commands')
    p.add_argument('--list',action='store_true');p.add_argument('--device')
    p.add_argument('--right-axes',choices=['auto','rxry','zrz'],default='auto')
    p.add_argument('--deadzone',type=float,default=.15);a=p.parse_args()
    if a.list:
        found=devices()
        try:
            for d in found:print(d.path,d.name,'axes=',sorted(axis_codes(d)))
            if not found:print('No accessible Xbox input device. Check VMware USB/Bluetooth assignment and input permissions.')
        finally:
            for d in found:d.close()
        return
    d=XboxController(a.device,a.deadzone,a.right_axes,require_neutral=False)
    try:
        while True:
            pose,_,mask,_=d.read(0)
            print('lx=%+.3f ly=%+.3f rx=%+.3f ry=%+.3f mask=%d pose=%s'%(*d.report,mask,pose),flush=True)
            time.sleep(.2)
    finally:d.close()

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('Input monitor stopped; no servo commands sent.')
    except (OSError,RuntimeError,ValueError) as error:raise SystemExit(str(error))
