"""Upstream JoyCon R stick directions; no IMU, face-key or reset actions."""
import math
import threading
import time

class StickPose:
    def __init__(self, speed=.005):
        if not math.isfinite(speed) or not 0 < speed <= .05:
            raise ValueError('Stick speed must be in (0, 0.05] m/s')
        self.pose=[0.]*6
        self.speed=speed
    def update(self,h,v,press,r,dt):
        if not 0<=h<=4095 or not 0<=v<=4095:raise ValueError('Invalid JoyCon stick report')
        # Same raw thresholds and horizontal polarity as upstream common_update.
        x=int(v>4000)-int(v<1000)
        y=int(h<1000)-int(h>4000)
        z=int(bool(r))-int(bool(press))
        directions=[x,y,z]
        dt=max(0.,min(float(dt),.2))
        length=math.sqrt(sum(d*d for d in directions)) or 1.
        for i,d in enumerate(directions):self.pose[i]+=self.speed*d*dt/length
        mask=sum(1<<(2*i+(d<0)) for i,d in enumerate(directions) if d)
        return list(self.pose),.5,mask,False

class StickJoyCon:
    def __init__(self):
        from joyconrobotics.joycon import JoyCon
        from joyconrobotics.device import get_R_id
        ids=get_R_id()
        if ids[0] is None:raise RuntimeError('JoyCon R not found; check Bluetooth/VM assignment')
        self.device=JoyCon(*ids);self.lock=threading.Lock()
        self.report=None;self.last_report=0.;self.previous=time.monotonic();self.integrator=StickPose()
        self.device.register_update_hook(self._report)
        deadline=time.monotonic()+2
        while not self.last_report and time.monotonic()<deadline:time.sleep(.01)
        if not self.last_report:
            self.close();raise RuntimeError('No fresh JoyCon stick reports')
        print('STICK ONLY: up/down=X +/-; left/right=Y -/+; press stick=Z-; R=Z+. Other buttons, IMU and Home ignored. Ctrl+C stops. Leave stick centered before Enter.',flush=True)
        print('STICK RAW h=%d v=%d (neutral window 1000..4000)'%self.report[:2],flush=True)

    def _report(self,d):
        values=(d.get_stick_right_horizontal(),d.get_stick_right_vertical(),bool(d.get_button_r_stick()),bool(d.get_button_r()))
        with self.lock:self.report=values;self.last_report=time.monotonic()

    def read(self,elapsed):
        del elapsed
        now=time.monotonic()
        with self.lock:report=self.report;fresh=self.last_report
        if not self.device._update_input_report_thread.is_alive() or now-fresh>.5:raise RuntimeError('JoyCon stick reports lost/stale; stopping arm')
        dt=now-self.previous;self.previous=now
        return self.integrator.update(*report,dt)

    def close(self):
        self.device._close();self.device._update_input_report_thread.join(timeout=.2)
