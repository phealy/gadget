# LEGO slot:0 autostart

import runloop
import motor_pair as mp, motor as m
from hub import port as p, motion_sensor as ms

async def yaw_angle():
    return ms.tilt_angles()[0] * -0.1 + 0

async def yaw_angle_diff(desired: int):
    return (await yaw_angle()) - desired

async def main():
    dp = mp.PAIR_1
    mp.pair(dp, p.A, p.B)

    while True:
        ms.reset_yaw(0)
        print("Initial yaw angle:", await yaw_angle())
        await mp.move_tank_for_degrees(dp, 90, 200, -200, stop=m.HOLD)
        print("Final yaw angle:", await yaw_angle())

runloop.run(main())