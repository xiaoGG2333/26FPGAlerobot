"""Xbox entry point using the existing UART/FPGA/ARM control pipeline."""
import sys
from right_joycon import main

if __name__=='__main__':
    try:main(['--source','xbox']+sys.argv[1:])
    except KeyboardInterrupt:print('Stopped; verify ARM1 STOP torque_off=1 if motion was enabled.')
    except (RuntimeError,TimeoutError,OSError,ValueError) as error:raise SystemExit(str(error))
