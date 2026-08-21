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


def log(message: str):
    '''
    Log a message to the console, prefixed by the timestamp.

    Args:
        message (str): the message to output
    '''
    print(f'{(ticks_ms() - START_TIME) / 1000.0:06.3f}s | {message}')


async def gyro_move(distance: float = 0,
                    angle: float | None = None,
                    drive_until: list[c.Color] | None = None,
                    turn_until: list[c.Color] | None = None,
                    long_turn: bool = False,
                    drive_velocity: int | None = None,
                    drive_acceleration: int | None = None,
                    turn_velocity: int | None = None,
                    turn_acceleration: int | None = None,
                    turn_deceleration: int | None = None,
                    description: str = "none"):
    '''
    **MUST BE USED WITH await.**
    
    Moves and turns based on the gyro sensor and global directions.

    If both distance and angle are supplied, the robot will perform a tank turn in place to
    the specified angle, then drive and maintain the angle even if bumped off course.
    
    Args:
        distance: The distance to drive in centimeters. May be omitted or 0 to just turn. Negative values back up.
        angle: The angle to turn to and/or drive at. If not supplied, the current yaw angle will be used.
        drive_until: A list[] of colors from the color module. The robot will drive until it has seen those colors in sequence.
        turn_until: A list[] of colors from the color module. The robot will turn until it has seen those colors in sequence. When using turn_until, angle determines the maximum amount of rotation and the direction.
        long_turn: The default behavior is to turn the shortest direction possible to reach the heading. If long_turn is True, turn the longer direction.
        drive_velocity: The velocity used for driving in degrees/second (100-1050).
        drive_acceleration: The acceleration value used for driving in degrees/second^2 (100-10000)
        turn_velocity: The velocity used for turning in degrees/second (100-1050).
        turn_acceleration: The acceleration value used for turning in degrees/second^2 (100-10000)
        turn_acceleration: The deceleration value used for turning in degrees/second^2 (100-10000)
        description: The description printed in the status console output.
    '''
    try:
        assert DRIVE_PAIR is not None
    except NameError:
        raise SystemExit("setup() must be called before calling gyro_move()")

    angle_supplied = True if angle is not None else False
    angle = yaw_angle() if angle is None else angle
    drive_velocity = DEFAULT_DRIVE_VELOCITY if drive_velocity is None else drive_velocity
    turn_velocity = DEFAULT_TURN_VELOCITY if turn_velocity is None else turn_velocity
    drive_acceleration = DEFAULT_ACCELERATION if drive_acceleration is None else drive_acceleration
    turn_acceleration = DEFAULT_ACCELERATION if turn_acceleration is None else turn_acceleration
    turn_deceleration = DEFAULT_DECELERATION if turn_deceleration is None else turn_deceleration

    # Check parameters
    assert 100 <= drive_velocity <= 1050, "drive velocity must be between 100 and 1050"
    assert 100 <= turn_velocity <= 1050, "turn velocity must be between 100 and 1050"
    assert 100 <= drive_acceleration <= 10000, "acceleration must be between 100 and 10000"
    assert 100 <= turn_acceleration <= 10000, "deceleration must be between 100 and 10000"
    assert -180 <= angle <= 180, "angle must be between -180 and 180 degrees"
    if drive_until is not None or turn_until is not None:
        assert COLOR_SENSOR is not None, "until specified but color sensor not initialized"
    if turn_until is not None:
        assert angle_supplied, "angle must be supplied when using turn_until"

    # Internal functions
    async def print_status(status: str):
        '''
        Print the current gyro_move() status.

        Example:
            05.077s | gyro_move | none         | driving    | Distance     20 | Desired yaw  -0.7 | Velocity 1000 | Accel 1000 | Yaw   -0.7 | ΔYaw      0 | Driven   20.5 | ΔDegrees   -15
        
        Args:
            status (str): The status field to show at the start of the line.
        '''    
        print(f'{(ticks_ms() - START_TIME) / 1000.0:06.3f}s',
                f'| gyro_move | {description:<12}',
                f'| {status:<10}',
                f'| Distance {distance:>6}',
                f'| Desired yaw {angle:>5}',
                f'| Yaw {round(yaw_angle(), 1):>6}',
                f'| ΔYaw {round(angle_difference(angle, long_turn), 1):>6}',
                f'| Driven {round(avg_relative_drive_distance_cm(), 1):>6}',
                f'| ΔDegrees {remaining_degrees_drive():>5}')

    def remaining_degrees_drive() -> int:
        '''Returns the degrees remaining to drive in the current execution'''
        return int((distance - avg_relative_drive_distance_cm()) * DEGREES_PER_CM)

    # Flip velocity and steering correction if we drive backwards, then make distance positive.
    direction = int(copysign(1, distance))
    distance = abs(distance)
    drive_velocity = drive_velocity * direction

    # Not doing color matching
    if drive_until is None and turn_until is None:
        if angle_supplied:
            # Do a gyro tank turn
            log("TODO: implement move turn instead of move_for_degrees")
            await print_status("turning")
            await mp.move_for_degrees(DRIVE_PAIR, int(angle_difference(angle, long_turn) * TURN_FACTOR), 100, stop = DEFAULT_STOP_TYPE, velocity=turn_velocity, acceleration=turn_acceleration, deceleration=turn_deceleration)

        if distance is not None:
            # Start driving, continuously correcting in a loop. Break out when we reach our distance.
            reset_relative_drive_distance()
            await print_status("driving")
            while remaining_degrees_drive() > 0:
                mp.move(DRIVE_PAIR, steering(angle, False) * direction, velocity=drive_velocity, acceleration=drive_acceleration)
                await runloop.sleep_ms(20)
            mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)

    # Turn color matching
    elif turn_until is not None:
        log(f'gyro_move | {description:<12} | colorturn  |                | remaining {[COLOR_NAMES[color] for color in turn_until]}')
        mp.move(DRIVE_PAIR, int(copysign(100, angle_difference(angle))), velocity=turn_velocity, acceleration=turn_acceleration)
        # Keep turning while there are colors left to match and we haven't hit our max angle
        while len(turn_until) > 0 and abs(angle_difference(angle)) > 1:
            # If the current color sensor value matches the one at the front of the list, log and remove it.
            if cs.color(COLOR_SENSOR) == turn_until[0]: # type: ignore
                log(f'gyro_move | {description:<12} | colorturn  | saw {COLOR_NAMES[turn_until[0]]:<11} | remaining {[COLOR_NAMES[color] for color in turn_until[1:]]}')
                turn_until.pop(0)
            await runloop.sleep_ms(10)
        log(f'gyro_move | {description:<12} | colorturn  | sequence complete')
        mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)

    # Drive color matching
    elif drive_until is not None:
        # Start driving, continuously correcting in a loop. Break out when we reach our distance.
        log(f'gyro_move | {description:<12} | colordrive |                 | remaining {[COLOR_NAMES[color] for color in drive_until]}')
        reset_relative_drive_distance()
        while len(drive_until) > 0:
            mp.move(DRIVE_PAIR, steering(angle, False) * direction, velocity=drive_velocity, acceleration=drive_acceleration)
            if cs.color(COLOR_SENSOR) == drive_until[0]: # type: ignore
                log(f'gyro_move | {description:<12} | colordrive | saw {COLOR_NAMES[drive_until[0]]:<11} | remaining {[COLOR_NAMES[color] for color in drive_until[1:]]}')
                drive_until.pop(0)
            await runloop.sleep_ms(10)
        mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)

    await print_status("finish")
