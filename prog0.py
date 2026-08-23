# LEGO slot:0 autostart
from gadget import *

# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

async def main():
    await setup(wheel_diameter="small", turn_factor=1.447,
                default_drive_velocity=200, default_acceleration=200,
                left_motor=p.A, right_motor=p.B,
                color_sensor=p.C, distance_sensor=p.D, force_sensor=p.F)
    
    # await drive(100, velocity=400, acceleration=200)
    # await drive(-100, velocity=400, acceleration=200)
    #await drive_until(colors=[GREEN, WHITE, BLACK], angle=0)
    #await drive(7)
    await turn_until(colors=[WHITE])
    # await drive_until(distance=5, angle=-90)
    # await drive_until(angle=90, pressed=True)

runloop.run(main())