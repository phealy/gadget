# LEGO slot:0 autostart
# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

from gadget import *
from hub import port
import runloop

async def main():
    await setup(wheel_diameter="small", turn_factor=1.447, left_motor=port.A, right_motor=port.B)
    await gyro_move(angle = 90, long_turn=True)
    await gyro_move(angle = 0, long_turn=True)
    await gyro_move(angle = 90, long_turn=False)
    await gyro_move(angle = 0, long_turn=False)
    await gyro_move(distance = 20)
    await gyro_move(distance = -20)
        
    log("completed")

runloop.run(main())