# LEGO slot:0 autostart
# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

from gadget import *
from hub import port
from color_sensor import color
from color import BLACK, WHITE, GREEN
import runloop

async def main():
    await setup(wheel_diameter="small", turn_factor=1.447, left_motor=port.A, right_motor=port.B, color_sensor=port.C)
    #await gyro_move(distance=-20, drive_velocity=200, description='backing up')
    # await gyro_move(angle=90, long_turn=True, description='face east')
    # await gyro_move(angle=0, description='face north')
    #await gyro_move(drive_until=[GREEN, WHITE, BLACK], drive_velocity=200, description='driveblack')
    # await gyro_move(angle=-85)
    # await gyro_move(angle=20, turn_until=[WHITE, BLACK], turn_velocity=100)
    # await gyro_move(angle=0)
    # await drive(10, -90, long_turn=True)
    await drive_until(GREEN, WHITE, BLACK, angle=-90, long_turn=False, velocity=100)

runloop.run(main())