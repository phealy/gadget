# LEGO slot:0 autostart
from gadget import *

# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

async def main():
    robot = await Gadget.setup(
        left_motor=port.A,
        right_motor=port.B,
        wheel_diameter="small",
        turn_factor=1.447,
        color_sensor=port.C,
        distance_sensor=port.D,
        force_sensor=port.F)
    
    await robot.drive(distance=-20, angle=0)
    await robot.drive_until(angle=0, colors=[GREEN])
    robot.log("completed")

runloop.run(main())