# LEGO slot:0 autostart
from gadget import *

# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

async def main():
    robot = await Gadget.setup(
        left_motor=port.F,
        right_motor=port.B,
        wheel_diameter="small",
        turn_factor=2.0252)
    await robot.setup_attachment("front", port.B, -90, gear_ratio=1, velocity=1000, acceleration=10000)
    await robot.setup_attachment("rear long arm", port.A, 90, gear_ratio=24, velocity=1000, acceleration=10000)
    #await runloop.sleep_ms(500)

    # Test color sensor
    #await robot.drive(distance=-5, angle=0)
    #await robot.drive_until(sensor_distance=25, velocity=800)
    #await robot.drive_until(sensor_distance=10, velocity=100)
    await robot.drive_until(angle=0, colors=[WHITE, BLACK], velocity=400)

    # Test front attachment
    # while True:
    #     await robot.move_attachment(position="rear long arm", angle=0)
    #     await runloop.sleep_ms(500)
    #     await robot.move_attachment(position="rear long arm", angle=-90)
    #     await runloop.sleep_ms(500)
            

    # Test rear attachment
    # await robot.move_attachment(position="rear long arm", angle=0)
    # await runloop.sleep_ms(500)
    # await robot.move_attachment(position="rear long arm", angle=-45)
    # await robot.move_attachment(position="rear long arm", angle=90)

    robot.log("completed")

runloop.run(main())