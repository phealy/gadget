# LEGO slot:19 autostart
# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

'''
turn_factor.py (Calculate turn factor)
--------------------------------------
Calculates the proper turning factor for a given SPIKE Prime robot. This is
related to the ratio between the distance between the wheels and the diameter
of the wheel. This code experimentally determines it by running 25 turns and
using a moving average function to get the observed turn as close as possible
to the desired turn.
'''

import runloop
import motor_pair as mp, motor as m
from hub import port as p, motion_sensor as ms, light_matrix as lm

dp = mp.PAIR_1
mp.pair(dp, p.A, p.B)

def normalize_angle(delta):
    '''Returns a given angle, scaled to -180 to 180.'''
    return ((delta + 180) % 360) - 180

async def yaw_angle():
    '''Retrieves the angle from the yaw sensor as a float with the proper sign.'''
    await runloop.until(ms.stable)
    return ms.tilt_angles()[0] * -0.1

def clip(value, low, high):
    return max(low, min(value, high))

async def run_system(degrees):
    await mp.move_for_degrees(mp.PAIR_1, int(degrees), 100, velocity=400, acceleration=100, deceleration=400, stop=m.SMART_BRAKE)
    await runloop.sleep_ms(120)
    return await yaw_angle()

async def averaged_yaw(samples=7, delay_ms=40):
    total = 0
    for _ in range(samples):
        await runloop.until(ms.stable)
        total += ms.tilt_angles()[0] * -0.1
        await runloop.sleep_ms(delay_ms)
    return total / samples

async def main():

    ms.reset_yaw(0)
    lm.clear()

    targets = [90, 180, -90, 0]

    f = 1.447
    f_values = []

    for step in range(25):

        T = targets[step % len(targets)]

        start_yaw = await yaw_angle()

        delta = normalize_angle(T - start_yaw)

        # limit commanded movement
        delta_cmd = clip(delta, -180, 180)
        input_value = f * delta_cmd

        # also cap input_value to avoid huge spins
        input_value = clip(input_value, -270, 270)

        await run_system(input_value)

        # average yaw after the move
        y_avg = await averaged_yaw(samples=7, delay_ms=40)
        actual_delta = normalize_angle(y_avg - start_yaw)

        # use the commanded delta (delta_cmd) for error
        if abs(delta_cmd) > 1:
            rel_error = (delta_cmd - actual_delta) / delta_cmd
        else:
            rel_error = 0.0

        # clamp relative error
        rel_error = clip(rel_error, -0.5, 0.5)

        # adaptive alpha
        alpha = clip(1.5 * abs(rel_error), 0.2, 0.6)

        # estimate gain
        if abs(input_value) > 1:
            k_est = actual_delta / input_value
        else:
            k_est = 1.0

        # clamp gain and prevent sign flip
        k_est = clip(k_est, 0.3, 3.0)

        ideal_f = 1.0 / k_est

        # do not allow f to change sign
        if f > 0 and ideal_f > 0:
            f = (1 - alpha) * f + alpha * ideal_f

        f_values.append(f)

        print(
            "{:>3d}".format(step), " | ",
            "target={:>6.1f}".format(T), " | ",
            "yaw={:>10.3f}".format(y_avg), " | ",
            "delta={:>8.3f}".format(delta_cmd), " | ",
            "actual={:>8.3f}".format(actual_delta), " | ",
            "rel_error={:>10.4f}".format(rel_error), " | ",
            "alpha={:>10.4f}".format(alpha), " | ",
            "f={:>10.4f}".format(f)
        )

        lm.set_pixel(step % 5, step // 5, 100)

    final_f = sum(f_values[-5:]) / 5
    print("final f={:.4f}".format(final_f))
    await lm.write("f={:.4f}".format(final_f))

runloop.run(main())
