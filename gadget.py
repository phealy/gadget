# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

from hub import motion_sensor as ms, port as p, light_matrix as lm
from math import pi, copysign
from time import ticks_ms
import color_sensor as cs, color as c
import motor_pair as mp, motor as m
import runloop

COLOR_NAMES = (
    "black", "magenta", "purple", "blue", "azure", "turquoise",
    "green", "yellow", "orange", "red", "white",
)
START_TIME = ticks_ms()

async def setup(
    default_drive_velocity: int = 320,
    default_turn_velocity: int = 200,
    default_acceleration: int = 1000,
    default_deceleration: int = 1000,
    default_stop_type: m.StopType = m.SMART_BRAKE,
    steering_correction: float = 1,
    left_motor: m.Motor = p.B,
    right_motor: m.Motor = p.A,
    color_sensor: p.Port | None = None,
    drive_pair: mp.MotorPair = mp.PAIR_1,
    wheel_diameter: str | float = "small",
    turn_factor: float = 1,
    ): 
    '''
    **MUST BE USED WITH await.**

    Setup the global variables used for the gyro_move() code.

    Args:
        default_drive_velocity: The default driving velocity.
        default_turn_velocity: The default turning velocity.
        default_acceleration: The default acceleration.
        default_deceleration: The default deceleration.
        default_stop_type: The default stop type.
        steering_correction: How aggressively to correct steering while in gyro_move().
        left_motor: The left motor for driving.
        right_motor: The right motor for driving.
        drive_pair: The motor pair to use for driving.
        wheel_diameter: "small" or "large" wheels, or a custom diameter in centimeters.
        turn_factor: The multiplier between degrees to turn and motor rotation. You can calculate this for your robot by running turn_factor.py.
    '''

    assert 100 <= default_drive_velocity <= 1050, "default drive velocity must be between 100 and 1050"
    assert 100 <= default_turn_velocity <= 1050, "default turn velocity must be between 100 and 1050"
    assert 100 <= default_acceleration <= 10000, "default acceleration must be between 100 and 10000"
    assert 100 <= default_deceleration <= 10000, "default deceleration must be between 100 and 10000"
    assert steering_correction >= 0, "steering correction must not be negative"
    assert left_motor != right_motor, "left and right motors must be different"
    assert wheel_diameter in ("small", "large") or (isinstance(wheel_diameter, (int, float)) and wheel_diameter > 0), "wheel diameter must be 'small', 'large', or a positive number"
    assert turn_factor > 0, "turn factor must be greater than 0"

    global DEFAULT_DRIVE_VELOCITY, DEFAULT_TURN_VELOCITY, DEFAULT_ACCELERATION
    global DEFAULT_DECELERATION, DEFAULT_STOP_TYPE, STEERING_CORRECTION
    global DRIVE_MOTORS, DRIVE_PAIR, DEGREES_PER_CM, TURN_FACTOR, COLOR_SENSOR

    await runloop.until(ms.stable)
    DEFAULT_DRIVE_VELOCITY = default_drive_velocity
    DEFAULT_TURN_VELOCITY = default_turn_velocity
    DEFAULT_ACCELERATION = default_acceleration
    DEFAULT_DECELERATION = default_deceleration
    DEFAULT_STOP_TYPE = default_stop_type
    STEERING_CORRECTION = steering_correction
    DRIVE_MOTORS = (left_motor, right_motor)
    DRIVE_PAIR = drive_pair
    COLOR_SENSOR = color_sensor
    if wheel_diameter == "small":
        DEGREES_PER_CM = 360 / (5.6 * pi)
    elif wheel_diameter == "large":
        DEGREES_PER_CM = 360 / (8.8 * pi)
    elif isinstance(wheel_diameter, (int, float)):
        DEGREES_PER_CM = 360 / (wheel_diameter * pi)
    else:
        raise SystemExit("Invalid wheel diameter given.")
    TURN_FACTOR = turn_factor

    mp.pair(DRIVE_PAIR, DRIVE_MOTORS[0], DRIVE_MOTORS[1])
    ms.set_yaw_face(ms.TOP)
    ms.reset_yaw(0)


def check_setup_complete():
    try:
        assert DRIVE_PAIR is not None
    except NameError:
        raise SystemExit("setup() must be called before using drive and turn functions!")


def yaw_angle() -> float:
    '''
    motion_sensor.tilt_angles() returns a (yaw, pitch, roll) tuple in decidegrees.
    [0] gets yaw, but the sign is reversed from the normal SPIKE UI values.
    multiply by -0.1 to get degrees with increasing values to the right.
    add 0 to prevent the angle from showing up as -0.0.
    '''
    yaw_angle = ms.tilt_angles()[0] * -0.1 + 0
    return yaw_angle


def reset_relative_drive_distance():
    '''Reset relative position, we use this to track distance.'''
    for port in DRIVE_MOTORS:
        m.reset_relative_position(port, 0)


def avg_relative_drive_distance_cm() -> float:
    '''Calculate the relative distance driven as an average of the two motor relative distances.'''
    distances = (abs(m.relative_position(DRIVE_MOTORS[0])), abs(m.relative_position(DRIVE_MOTORS[1])))
    return sum(distances) / len(distances) / DEGREES_PER_CM


def angle_difference(angle: float, long_turn: bool = False) -> float:
    '''
    Calculate the difference between the current yaw angle and the desired angle. Wrap to -180 to 180.
        
    Args:
        angle: the desired angle to turn to.
        long_turn: by default steering uses the shorter turn direction. long_turn will use the longer turn direction instead.
    '''
    result = ((angle - yaw_angle() + 180) % 360) - 180
    if long_turn:
        result -= copysign(360, result)
    return result


def steering(angle: float, long_turn: bool = False) -> int:
    '''Calculates the steering for move_for_degrees() based on the STEERING_CORRECTION

    Steering is based on how far off we are from the correct angle but adjusted to be no more than 50.
    
    Args:
        angle: the desired angle to turn to.
        long_turn: by default steering uses the shorter turn direction. long_turn will use the longer turn direction instead.
    '''
    return int(max(-50, min(50, angle_difference(angle, long_turn) * STEERING_CORRECTION)))


async def drive(distance: float,
                angle: float | None = None,
                long_turn: bool = False,
                velocity: int | None = None,
                acceleration: int | None = None):
    '''
    Drives a given distance at a given heading based on wheel rotation and gyroscope yaw.

    Args:
        distance: The distance to drive in centimeters. May be omitted or 0 to just turn. Negative values back up.
        angle: The angle to drive at. Will turn to the angle first if not already facing that way. Defaults to current angle (but this is less precise!).
        drive_velocity: The velocity used for driving in degrees/second (100-1050).
        drive_acceleration: The acceleration value used for driving in degrees/second^2 (100-10000)
    '''
    check_setup_complete()
    angle = yaw_angle() if angle is None else angle
    velocity = DEFAULT_DRIVE_VELOCITY if velocity is None else velocity
    acceleration = DEFAULT_ACCELERATION if acceleration is None else acceleration
    assert 100 <= velocity <= 1050, "velocity must be between 100 and 1050"
    assert 100 <= acceleration <= 10000, "acceleration must be between 100 and 10000"
    assert -180 <= angle <= 180, "angle must be between -180 and 180 degrees"
    (distance, direction) = (abs(distance), int(copysign(1, distance)))

    # If we're too far from our desired angle, turn first.
    if abs(angle_difference(angle, long_turn=long_turn)) > 1:
        await turn(angle, long_turn=long_turn)
    
    # Start driving, continuously correcting in a loop. Break out when we reach our distance.
    reset_relative_drive_distance()
    while int((distance - avg_relative_drive_distance_cm()) * DEGREES_PER_CM) > 0:
        mp.move(DRIVE_PAIR, steering(angle, False) * direction,
                velocity=velocity * direction, acceleration=acceleration)
        await runloop.sleep_ms(20)
    mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)


async def turn(angle: float,
               long_turn: bool = False,
               velocity: int | None = None,
               acceleration: int | None = None,
               deceleration: int | None = None):
    '''
    Tank turns to a global angle (0 degrees is where the robot is pointed when setup() is called).

    Args:
        angle: The angle to turn to.
        long_turn: The default behavior is to turn the shortest direction possible to reach the heading. If long_turn is True, turn the longer direction.
        velocity: The velocity used for turning in degrees/second (100-1050).
        acceleration: The acceleration value used for turning in degrees/second^2 (100-10000)
    '''
    check_setup_complete()
    velocity = DEFAULT_TURN_VELOCITY if velocity is None else velocity
    acceleration = DEFAULT_ACCELERATION if acceleration is None else acceleration
    deceleration = DEFAULT_DECELERATION if deceleration is None else deceleration
    assert -180 <= angle <= 180, "angle must be between -180 and 180 degrees"
    assert 100 <= velocity <= 1050, "turn velocity must be between 100 and 1050"
    assert 100 <= acceleration <= 10000, "acceleration must be between 100 and 10000"
    assert 100 <= deceleration <= 10000, "deceleration must be between 100 and 10000"

    await mp.move_for_degrees(DRIVE_PAIR, int(angle_difference(angle, long_turn) * TURN_FACTOR), 100, stop = DEFAULT_STOP_TYPE, velocity=velocity, acceleration=acceleration, deceleration=deceleration)


async def drive_until(*colors: c.Color,
                      angle: float | None = None,
                      reverse: bool = False,
                      long_turn: bool = False,
                      velocity: int | None = None,
                      acceleration: int | None = None):
    '''
        Drives at a given heading based on gyroscope yaw until colors are seen in a certain order.

    Args:
        colors: Any number of colors from the color module. The robot will drive until it has seen those colors in sequence.
        angle: The angle to drive at. Will turn to the angle first if not already facing that way. Defaults to current angle (but this is less precise!).
        reverse: if True, back up
        long_turn: The default behavior is to turn the shortest direction possible to reach the heading. If long_turn is True, turn the longer direction.
        velocity: The velocity used for driving in degrees/second (100-1050).
        acceleration: The acceleration value used for driving in degrees/second^2 (100-10000)

    Example:
        from colors import GREEN, WHITE, BLACK
        drive_until(GREEN, WHITE, BLACK, velocity=200)
    '''
    check_setup_complete()
    angle = yaw_angle() if angle is None else angle
    velocity = DEFAULT_DRIVE_VELOCITY if velocity is None else velocity
    acceleration = DEFAULT_ACCELERATION if acceleration is None else acceleration
    assert -180 <= angle <= 180, "angle must be between -180 and 180 degrees"
    assert 100 <= velocity <= 1050, "velocity must be between 100 and 1050"
    assert 100 <= acceleration <= 10000, "acceleration must be between 100 and 10000"
    direction = -1 if reverse else 1
    color_list = list(colors)

    # If we're too far from our desired angle, turn first.
    if abs(angle_difference(angle, long_turn=long_turn)) > 1:
        await turn(angle, long_turn=long_turn, velocity=DEFAULT_TURN_VELOCITY)

    # Start driving, continuously correcting in a loop. Break out when we reach our distance.
    while color_list:
        mp.move(DRIVE_PAIR, steering(angle, False) * direction, velocity=velocity, acceleration=acceleration)
        if cs.color(COLOR_SENSOR) == color_list[0]: # type: ignore
            #log(f'gyro_move | {description:<12} | colordrive | saw {COLOR_NAMES[drive_until[0]]:<11} | remaining {[COLOR_NAMES[color] for color in drive_until[1:]]}')
            color_list.pop(0)
        await runloop.sleep_ms(10)
    mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)


async def turn_until(*colors: c.Color,
                      left: bool = False,
                      velocity: int | None = None,
                      acceleration: int | None = None):
    '''
    Turns until colors are seen in a certain order.

    Args:
        colors: Any number of colors from the color module. The robot will drive until it has seen those colors in sequence.
        left: if True, turns left. Defaults to right.
        velocity: The velocity used for driving in degrees/second (100-1050).
        acceleration: The acceleration value used for driving in degrees/second^2 (100-10000)

    Example:
        from colors import GREEN, WHITE, BLACK
        drive_until(GREEN, WHITE, BLACK, velocity=200)
    '''
    check_setup_complete()
    velocity = DEFAULT_DRIVE_VELOCITY if velocity is None else velocity
    acceleration = DEFAULT_ACCELERATION if acceleration is None else acceleration
    assert 100 <= velocity <= 1050, "velocity must be between 100 and 1050"
    assert 100 <= acceleration <= 10000, "acceleration must be between 100 and 10000"
    direction = -1 if left else 1
    color_list = list(colors)

    # Start driving, continuously correcting in a loop. Break out when we reach our distance.
    while color_list:
        mp.move(DRIVE_PAIR, 100 * direction, velocity=velocity, acceleration=acceleration)
        if cs.color(COLOR_SENSOR) == color_list[0]: # type: ignore
            color_list.pop(0)
        await runloop.sleep_ms(10)
    mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)
