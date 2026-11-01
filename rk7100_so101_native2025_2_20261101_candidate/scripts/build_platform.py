from pathlib import Path
import vitis
r=Path(__file__).resolve().parents[1]
client=vitis.create_client()
try:
    client.set_workspace(str(r/'build/ws'))
    if (r/'build/ws/platform/vitis-comp.json').exists():
        platform=client.get_component(name='platform')
        platform.update_hw(hw_design=str(r/'build/hardware_export/rk7100_so101.xsa'))
    else:
        platform=client.create_platform_component(name='platform',hw_design=str(r/'build/hardware_export/rk7100_so101.xsa'),cpu='ps7_cortexa9_0',os='standalone',domain_name='standalone_ps7_cortexa9_0',generate_dtb=False)
    platform.build()
    print('PLATFORM_2025_2_COMPLETE')
finally:
    vitis.dispose()
