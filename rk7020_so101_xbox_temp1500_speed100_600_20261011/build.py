"""Build UART1 application + FSBL using new PS initialization, preserve old app.
Hardware build.tcl must finish first. AMD 2024.2 toolchain and existing tested
BSP supply the unchanged peripheral drivers; UART1 uses explicit config.
"""
from pathlib import Path
import argparse,os,subprocess,shutil,zipfile,hashlib,json
root=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--hardware',type=Path,default=root.parent/'RK7020_SO101_135/boot' if (root.parent/'RK7020_SO101_135/boot').exists() else Path(r'D:\VivadoProjects\RK7020_SO101_135\boot'))
p.add_argument('--vitis',type=Path,default=Path(r'D:\Software\vivado2024.2\Vitis\2024.2'))
p.add_argument('--platform',type=Path,default=Path(r'D:\VivadoProjects\RK7020UART_Test\platform'))
p.add_argument('--gcc',type=Path,help='ARM hard-float bare-metal GCC executable')
p.add_argument('--bootgen',type=Path,help='Bootgen executable')
p.add_argument('--bsp',type=Path,help='Application BSP directory containing include/, lib/ and Xilinx.spec')
p.add_argument('--output',type=Path,default=root/'boot_coordinated',help='Separate output folder; original boot/ is preserved')
p.add_argument('--check',action='store_true',help='Check build dependencies only; no files are changed')
p.add_argument('--compile-only',action='store_true')
p.add_argument('--calibration',type=Path,help='Reviewed physical mapping matching the current FPGA geometry')
a=p.parse_args()
vitis=a.vitis
tools=vitis/('gnu/aarch32/nt/gcc-arm-none-eabi/bin' if os.name=='nt' else 'gnu/aarch32/lin/gcc-arm-none-eabi/bin')
platform=a.platform
bsp=a.bsp or platform/'export/platform/sw/standalone_ps7_cortexa9_0'
gcc=a.gcc or tools/('arm-none-eabi-gcc.exe' if os.name=='nt' else 'arm-none-eabi-gcc')
bootgen=a.bootgen or vitis/('bin/bootgen.bat' if os.name=='nt' else 'bin/bootgen')
original=platform/'zynq_fsbl'
fbsp=original/'zynq_fsbl_bsp'
required=[gcc,bsp/'include/xparameters.h',bsp/'Xilinx.spec',bsp/'lib/libxil.a',
          bsp/'lib/libxilstandalone.a',bsp/'lib/libxiltimer.a']
if not a.compile_only:
    required += [bootgen,original/'lscript.ld',fbsp/'include',fbsp/'Xilinx.spec',
                 *[fbsp/'lib'/('lib'+name+'.a') for name in ('xil','xiltimer','xilffs','xilrsa','xilstandalone')],
                 *[a.hardware/name for name in ('rk7020_uart1.bit','rk7020_uart1.hwh','rk7020_uart1.xsa','timing.rpt','drc.rpt','utilization.rpt')]]
if a.calibration:
    required += [root/'apply_verified_mapping.py',a.calibration]
missing=[path for path in required if not path.exists()]
if missing:
    print('Build dependencies missing (no build or calibration changes made):')
    for path in missing:print('  '+str(path))
    raise SystemExit(1)
if a.check:
    print('Build dependencies present. Compiler/BSP compatibility still requires an actual build.')
    raise SystemExit(0)
if a.calibration:
    from apply_verified_mapping import apply
    apply(a.calibration,root)
tools=gcc.parent
boot=a.output.resolve();boot.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['PATH']=str(tools)+os.pathsep+env.get('PATH','')
def run(args):
    result=subprocess.run([str(x) for x in args],cwd=boot,env=env,capture_output=True,text=True)
    with (boot/'build.log').open('a',encoding='utf8') as f:
        f.write(subprocess.list2cmdline([str(x) for x in args])+'\n'+result.stdout+result.stderr+'\n')
    print(result.stdout+result.stderr,flush=True);result.check_returncode()
(boot/'build.log').write_text('UART1 coordinated manual build\n')
common=['-mcpu=cortex-a9','-mfpu=vfpv3','-mfloat-abi=hard','-DSDT']
run([gcc,*common,'-O0','-g3','-Wall','-Wextra','-Werror',
     '-I'+str(bsp/'include'),'-specs='+str(bsp/'Xilinx.spec'),
     root/'firmware/main.c',root/'firmware/ik_iteration.c',root/'firmware/st3215.c',root/'firmware/motor_control.c',root/'firmware/arm_control.c',root/'firmware/cal_bridge.c',
     '-o',boot/'ik_uart1_servo.elf','-Wl,-T,'+str(root/'firmware/lscript.ld'),
     '-L'+str(bsp/'lib'),'-Wl,--start-group','-lxilstandalone','-lxiltimer','-lxil','-lgcc','-lc','-lm','-Wl,--end-group'])
if a.compile_only:raise SystemExit(0)
for name in ('rk7020_uart1.bit','rk7020_uart1.hwh','rk7020_uart1.xsa','timing.rpt','drc.rpt','utilization.rpt'):
    shutil.copy2(a.hardware/name,boot/name)
fsbl=boot/'fsbl_source';fsbl.mkdir(exist_ok=True)
for source in original.iterdir():
    if source.suffix in ('.c','.h','.S','.ld'):shutil.copy2(source,fsbl/source.name)
with zipfile.ZipFile(boot/'rk7020_uart1.xsa') as z:
    for filename in ('ps7_init.c','ps7_init.h','ps7_init.tcl'):
        matches=[n for n in z.namelist() if n.split('/')[-1]==filename]
        if len(matches)!=1:raise RuntimeError('Cannot locate new '+filename+' in XSA')
        (fsbl/filename).write_bytes(z.read(matches[0]))
# Existing BSP config is valid for unchanged DDR, SD, QSPI, console; UART1 init
# comes from the regenerated ps7_init.c and app's explicit UART1 configuration.
run([gcc,*common,'-O0','-g3','-Os','-I'+str(fsbl),'-I'+str(fbsp/'include'),
     '-specs='+str(fbsp/'Xilinx.spec'),*sorted(fsbl.glob('*.c')),*sorted(fsbl.glob('*.S')),
     '-o',boot/'fsbl_uart1.elf','-Wl,--gc-sections','-n','-Wl,-T,'+str(fsbl/'lscript.ld'),
     '-L'+str(fbsp/'lib'),'-Wl,--start-group','-lxiltimer','-lxilffs','-lxilrsa','-lxil','-lxilstandalone','-lgcc','-lc','-Wl,--end-group'])
(boot/'boot.bif').write_text('the_ROM_image:\n{\n [bootloader] fsbl_uart1.elf\n rk7020_uart1.bit\n ik_uart1_servo.elf\n}\n')
run([bootgen,'-arch','zynq','-image','boot.bif','-o','BOOT.BIN','-w','on'])
manifest={str(f.relative_to(boot)).replace('\\','/'):hashlib.sha256(f.read_bytes()).hexdigest()
          for f in boot.rglob('*') if f.is_file() and f.name!='SHA256.json'}
(boot/'SHA256.json').write_text(json.dumps(manifest,indent=2))
print('BOOT.BIN READY',boot/'BOOT.BIN')
