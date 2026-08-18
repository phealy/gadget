# LEGO slot:0 autostart

from gadget import *
from hub import port
import runloop

async def main():
    await setup(wheel_diameter="small", turning_multiplier=1.432, left_motor=port.A, right_motor=port.E)
    await gyro_move(distance=20, description="Model 1")
    await gyro_move(angle=-90)
    await gyro_move(angle=180)
    await gyro_move(angle=90)
    await gyro_move(angle=0)
    await print_timer()

runloop.run(main())