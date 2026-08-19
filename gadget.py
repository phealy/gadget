from hub import motion_sensor as ms, port as p, light_matrix as lm
from math import pi, copysign
from time import ticks_ms
import motor_pair as mp, motor as m
import runloop

# Control parameters
DEFAULT_DRIVE_VELOCITY = 800
DEFAULT_TURN_VELOCITY = 800
DEFAULT_ACCELERATION = 1000
DEFAULT_DECELERATION = 1000
DEFAULT_STOP_TYPE = m.SMART_BRAKE   # Default stop type
STEERING_CORRECTION = 1             # How aggressively to correct steering while in gyro_drive()
DRIVE_MOTORS = (p.B, p.A)           # Left motor first, then right motor second.
DRIVE_PAIR = mp.PAIR_1              # Pair to use for driving
DEGREES_PER_CM = 360 / (5.6 * pi)   # Small wheels
# DEGREES_PER_CM = 360 / (8.8 * pi)  # Large wheels
TURNING_MULTIPLIER = 1.432          # Multiplier between degrees to turn and motor rotation
START_TIME = ticks_ms()

# TURNING_MULTIPLIER is calculated by finding the diameter of the turning circle (center to center of the
# wheels, measured horizontally) and dividing it by the diameter of the wheel. It should be experimentally
# tweaked for each robot.

async def setup(
    default_drive_velocity: int = 600,
    default_turn_velocity: int = 600,
    default_acceleration: int = 800,
    default_deceleration: int = 800,
    default_stop_type: m.StopType = m.SMART_BRAKE,      # Default stop type
    steering_correction: float = 5,                     # How aggressively to correct steering while in gyro_drive()
    left_motor: m.Motor = p.B,                          # Left motor for driving
    right_motor: m.Motor = p.A,                         # Right motor for driving
    drive_pair: mp.MotorPair = mp.PAIR_1,               # Pair to use for driving
    wheel_diameter: any = "small",                      # "small" or "large" wheels or a custom diameter in cm
    turning_multiplier: float = 1.432,                  # Multiplier between degrees to turn and motor rotation
):
    global DEFAULT_DRIVE_VELOCITY, DEFAULT_TURN_VELOCITY, DEFAULT_ACCELERATION
    global DEFAULT_DECELERATION, DEFAULT_STOP_TYPE, STEERING_CORRECTION
    global DRIVE_MOTORS, DRIVE_PAIR, DEGREES_PER_CM, TURNING_MULTIPLIER

    await runloop.until(ms.stable)
    DEFAULT_DRIVE_VELOCITY = default_drive_velocity
    DEFAULT_TURN_VELOCITY = default_turn_velocity
    DEFAULT_ACCELERATION = default_acceleration
    DEFAULT_DECELERATION = default_deceleration
    DEFAULT_STOP_TYPE = default_stop_type
    STEERING_CORRECTION = steering_correction
    DRIVE_MOTORS = (left_motor, right_motor)
    DRIVE_PAIR = drive_pair
    if wheel_diameter == "small":
        DEGREES_PER_CM = 360 / (5.6 * pi)
    elif wheel_diameter == "large":
        DEGREES_PER_CM = 360 / (8.8 * pi)
    else:
        DEGREES_PER_CM = 360 / (wheel_diameter * pi)
    TURNING_MULTIPLIER = turning_multiplier

    mp.pair(DRIVE_PAIR, DRIVE_MOTORS[0], DRIVE_MOTORS[1])
    ms.set_yaw_face(ms.TOP)
    ms.reset_yaw(0)


def yaw_angle() -> float:
    # tilt_angles() returns a (yaw, pitch, roll) tuple in decidegrees. [0] gets yaw.
    # multiply by -0.1 to get degrees with increasing values to the right.
    # add 0 to prevent the angle from showing up as -0.0.
    yaw_angle = ms.tilt_angles()[0] * -0.1 + 0
    return yaw_angle


# Reset relative position, we use this to track distance
def reset_relative_drive_distance():
    for port in DRIVE_MOTORS:
        m.reset_relative_position(port, 0)


# # Calculate the relative distance driven as an average of the two motor relative distances
def avg_relative_drive_distance_cm() -> float:
    distances = (abs(m.relative_position(DRIVE_MOTORS[0])), abs(m.relative_position(DRIVE_MOTORS[1])))
    return sum(distances) / len(distances) / DEGREES_PER_CM


# Returns the difference between current and desired angle, scaled to -179.9 to 180
def angle_difference(angle: float) -> float:
    return ((angle - yaw_angle() + 180) % 360) - 180


def steering(angle: float) -> int:
    # Steering is based on how far off we are from the correct angle but adjusted to be no more than 100.
    # STEERING_CORRECTION makes us move back to center more quickly
    return int(max(-50, min(50, angle_difference(angle) * STEERING_CORRECTION)))

def log(message: str):
    print(f'{(ticks_ms() - START_TIME) / 1000.0:06.3f}s | {message}')


async def gyro_move(distance: float = 0,                          # Drive distance in cm, use 0 to just turn, negatives drive backwards
                    angle: float = None,                          # Global angle to drive (-179.9 to 180) - defaults to current
                    drive_velocity: int = None,                   # Driving velocity (0 to 1050)
                    turn_velocity: int = None,                    # Turning velocity (0 to 1050)
                    acceleration: int = None,                     # Acceleration (0 to 10000 in deg/sec^2)
                    deceleration: int = None,                     # Deceleration (0 to 10000 in deg/sec^2)
                    description: str = "none"):                   # Description of task for log output

    angle = yaw_angle() if angle is None else angle
    drive_velocity = DEFAULT_DRIVE_VELOCITY if drive_velocity is None else drive_velocity
    turn_velocity = DEFAULT_TURN_VELOCITY if turn_velocity is None else turn_velocity
    acceleration = DEFAULT_ACCELERATION if acceleration is None else acceleration
    deceleration = DEFAULT_DECELERATION if deceleration is None else deceleration

    # Check parameters
    assert drive_velocity >= 0 and drive_velocity <= 1050, "drive velocity must be between 0 and 1050"
    assert turn_velocity >= 0 and turn_velocity <= 1050, "drive velocity must be between 0 and 1050"
    assert acceleration >= 0 and acceleration <= 10000, "acceleration must be between 0 and 10000"
    assert deceleration >= 0 and deceleration <= 10000, "deceleration must be between 0 and 10000"
    assert angle >= -179.9 and angle <= 180, "angle must be between -179.9 and 180 degrees"

    # Internal functions
    async def print_status(status: str):
                print(f'{(ticks_ms() - START_TIME) / 1000.0:06.3f}s',
                      f'| gyro_move | {description:<12}',
                      f'| {status:<10}',
                      f'| Distance {distance:>6}',
                      f'| Desired yaw {angle:>5}',
                      f'| Velocity {velocity:>4}',
                      f'| Accel {acceleration:>4}',
                      f'| Yaw {round(yaw_angle(), 1):>6}',
                      f'| ΔYaw {round(angle_difference(angle), 1):>6}',
                      f'| Driven {round(avg_relative_drive_distance_cm(), 1):>6}',
                      f'| ΔDegrees {remaining_degrees_drive():>5}')

    # Return the degrees remaining to drive in the current execution
    def remaining_degrees_drive() -> int:
        return int((distance - avg_relative_drive_distance_cm()) * DEGREES_PER_CM)

    # Flip velocity and steering correction if we drive backwards, then make distance positive.
    direction = int(copysign(1, distance))
    velocity = drive_velocity * direction
    distance = abs(distance)
    
    reset_relative_drive_distance()
    await print_status("start")

    # Do a gyro tank turn
    await mp.move_for_degrees(DRIVE_PAIR, int(angle_difference(angle) * TURNING_MULTIPLIER), 100, stop = DEFAULT_STOP_TYPE, acceleration=acceleration, deceleration=deceleration)

    await print_status("post-turn")

    # Start driving, continuously correcting in a loop. Break out when we reach our distance.
    reset_relative_drive_distance()
    while remaining_degrees_drive() > 0:
        mp.move(DRIVE_PAIR, steering(angle) * direction, velocity=velocity, acceleration=acceleration)
    mp.stop(DRIVE_PAIR, stop=DEFAULT_STOP_TYPE)

    await print_status("finish")


async def print_timer():
     log("Timer printed.")
     await lm.write(f'{(ticks_ms() - START_TIME) / 1000.0:.1f}s')
