from pathlib import Path
import vitis
root=Path(__file__).resolve().parents[1]
client=vitis.create_client()
try:
    client.set_workspace(str(root/'build/vitis_workspace'))
    if (root/'build/vitis_workspace/platform/vitis-comp.json').exists():
        platform=client.get_component(name='platform')
        platform.update_hw(hw_design=str(root/'build/hardware_export/rk7100_so101.xsa'))
    else:
        platform=client.create_platform_component(name='platform',hw_design=str(root/'build/hardware_export/rk7100_so101.xsa'),cpu='ps7_cortexa9_0',os='standalone',domain_name='standalone_ps7_cortexa9_0',generate_dtb=False)
    platform.build()
    print('RK7100_PLATFORM_BUILD_COMPLETE')
finally:
    vitis.dispose()
