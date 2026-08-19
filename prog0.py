# LEGO slot:0 autostart

from gadget import *
from hub import port
import runloop

async def main():
    await setup(wheel_diameter="small", turn_factor=1.447, left_motor=port.A, right_motor=port.B)
    await gyro_move(distance=100, angle=0, drive_velocity=800, description='drive to leaf cutter')
    await gyro_move(distance=-100, angle=0, drive_velocity=800, description='push leaf cutter')
    #await gyro_move(distance=20, drive_velocity=100)
    await print_timer()

runloop.run(main())