"""Buttons-only JoyCon R input. Never consume gyro, accelerometer or sticks."""
import threading
import time

BUTTONS=('x','b','a','y','right_sr','right_sl','r','zr','home','plus')

class ButtonPose:
    """Integrate held buttons in SI units; opposing buttons cancel."""
    def __init__(self):
        self.pose=[0.]*6

    def update(self,buttons,dt):
        if buttons.get('plus',False):
            raise KeyboardInterrupt('JoyCon + stop')
        dt=max(0.,min(float(dt),.2))
        if buttons.get('zr',False):
            dirs=[int(bool(buttons.get('right_sr')))-int(bool(buttons.get('right_sl'))),
                  int(bool(buttons.get('a')))-int(bool(buttons.get('y'))),
                  int(bool(buttons.get('x')))-int(bool(buttons.get('b')))]
            rotation=bool(buttons.get('r'))
            if rotation:
                # Five-axis model: yaw follows the shoulder plane in Mapper.
                dirs=[dirs[1],dirs[2],0]
            offset=3 if rotation else 0
            rate=.10 if rotation else .005
            for i,d in enumerate(dirs):self.pose[offset+i]+=d*rate*dt
        mask=sum((1<<i) for i,n in enumerate(BUTTONS) if buttons.get(n,False))
        return list(self.pose),.5,mask,bool(buttons.get('home'))

class ButtonJoyCon:
    def __init__(self):
        from joyconrobotics.joycon import JoyCon
        from joyconrobotics.device import get_R_id
        ids=get_R_id()
        if ids[0] is None:raise RuntimeError('JoyCon R not found; check Bluetooth and VM assignment')
        self.device=JoyCon(*ids)
        self.lock=threading.Lock()
        self.buttons={}
        self.last_report=0.
        self.integrator=ButtonPose()
        self.previous=time.monotonic()
        self.device.register_update_hook(self._report)
        deadline=time.monotonic()+2
        while not self.last_report and time.monotonic()<deadline:time.sleep(.01)
        if not self.last_report:
            self.close()
            raise RuntimeError('No fresh JoyCon button reports; motion not enabled')
        print('BUTTONS ONLY: hold ZR; X/B=Z +/-; A/Y=Y +/-; SR/SL=X +/-; hold R: A/Y=roll +/-, X/B=pitch +/- (yaw automatic); Home=recenter; +=stop. IMU and sticks ignored.',flush=True)

    def _report(self,device):
        values={n:bool(getattr(device,'get_button_'+n)()) for n in BUTTONS}
        with self.lock:
            self.buttons=values
            self.last_report=time.monotonic()

    def read(self,elapsed):
        del elapsed
        now=time.monotonic()
        with self.lock:buttons=dict(self.buttons);fresh=self.last_report
        if not self.device._update_input_report_thread.is_alive() or now-fresh>.5:
            raise RuntimeError('JoyCon button reports lost/stale; stopping arm')
        dt=now-self.previous;self.previous=now
        return self.integrator.update(buttons,dt)

    def close(self):
        self.device._close()
        self.device._update_input_report_thread.join(timeout=.2)
