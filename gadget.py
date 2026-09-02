# Copyright (c) 2026 Patrick W. Healy <phealy@phealy.com>
# SPDX-License-Identifier: MIT

from hub import motion_sensor, port
from math import pi, copysign
from time import ticks_ms
import force_sensor, distance_sensor
import color_sensor, color
import color_sensor as color_sensor_module
import distance_sensor as distance_sensor_module
import force_sensor as force_sensor_module
from color import BLACK, WHITE, GREEN, AZURE, BLUE
from color import MAGENTA, ORANGE, PURPLE, RED
from color import TURQUOISE, YELLOW, UNKNOWN
import motor_pair, motor
import runloop


COLOR_NAMES = {
    color.BLACK: "BLACK", color.WHITE: "WHITE", color.GREEN: "GREEN", color.AZURE: "AZURE",
    color.BLUE: "BLUE", color.MAGENTA: "MAGENTA", color.ORANGE: "ORANGE", color.PURPLE: "PURPLE",
    color.RED: "RED", color.TURQUOISE: "TURQUOISE", color.YELLOW: "YELLOW", color.UNKNOWN: "UNKNOWN"
}
PORT_NAMES = {port.A: "A", port.B: "B", port.C: "C", port.D: "D", port.E: "E", port.F: "F"}
MOTOR_TYPES = {48: "medium", 49:"large"}


class Gadget:
    def __init__(
        self: Gadget,
        left_motor: motor.Motor,
        right_motor: motor.Motor,
        wheel_diameter: str | float,
        turn_factor: float,
        default_drive_velocity: int = 320,
        default_turn_velocity: int = 200,
        default_acceleration: int = 1000,
        default_deceleration: int = 1000,
        default_stop_type: motor.StopType = motor.SMART_BRAKE,
        steering_correction: float = 1,
        color_sensor: port.Port | None = None,
        force_sensor: port.Port | None = None,
        distance_sensor: port.Port | None = None,
        drive_pair: motor_pair.MotorPair = motor_pair.PAIR_1): 
        '''
        Setup the global variables used for the gadget module.

        Args:
            left_motor: The left motor for driving.
            right_motor: The right motor for driving.
            wheel_diameter: "small" or "large" wheels, or a custom diameter in centimeters.
            turn_factor: The multiplier between degrees to turn and motor rotation. You can calculate this for your robot by running turn_factor.py.
            default_drive_velocity: The default driving velocity (320).
            default_turn_velocity: The default turning velocity (200).
            default_acceleration: The default acceleration (1000).
            default_deceleration: The default deceleration (1000).
            default_stop_type: The default stop type (SMART_BRAKE).
            steering_correction: How aggressively to correct steering while driving.
            color_sensor: The port where the color sensor is attached, if any.
            force_sensor: The port where the force sensor is attached, if any.
            distance_sensor: The port where the distance sensor is attached, if any.
            drive_pair: The motor pair to use for driving (PAIR_1).
        '''

        # Store the time the code started running for logging
        self._start_time = ticks_ms()
        self._last_log_time = self._start_time

        print("\n" * 10)
        print("=" * 80)
        print("Time     | Delta    | Message")
        print("=" * 80)
        self.log("initializing Gadget module")

        # Check parameter values
        self._check_parameters(velocity=default_drive_velocity,
                              acceleration=default_acceleration,
                              deceleration=default_deceleration,
                              steering_correction=steering_correction,
                              left_motor=left_motor,
                              right_motor=right_motor)
        self._check_parameters(velocity=default_turn_velocity)

        # Store defaults
        self._default_drive_velocity = default_drive_velocity
        self._default_turn_velocity = default_turn_velocity
        self._default_acceleration = default_acceleration
        self._default_deceleration = default_deceleration
        self._default_stop_type = default_stop_type
        self._steering_correction = steering_correction
        self._drive_motors = (left_motor, right_motor)
        self._drive_pair = drive_pair
        self._color_sensor = color_sensor
        self._force_sensor = force_sensor
        self._distance_sensor = distance_sensor
        self._attachments = {}

        # Set degrees per centimeter based on wheel size
        if wheel_diameter == "small":
            self._wheel_degrees_per_cm = 360 / (5.6 * pi)
        elif wheel_diameter == "large":
            self._wheel_degrees_per_cm = 360 / (8.8 * pi)
        elif isinstance(wheel_diameter, (int, float)):
            self._wheel_degrees_per_cm = 360 / (wheel_diameter * pi)
        else:
            raise SystemExit("Invalid wheel diameter given.")

        # If turn factor is unknown, run turn_factor.py with your robot to calculate it.
        self._turn_factor = turn_factor


    @classmethod
    async def setup(
            cls,
            left_motor: motor.Motor,
            right_motor: motor.Motor,
            wheel_diameter: str | float,
            turn_factor: float,
            yaw_angle: float = 0,
            yaw_face: motion_sensor.HubFace = motion_sensor.TOP,
            default_drive_velocity: int = 320,
            default_turn_velocity: int = 200,
            default_acceleration: int = 1000,
            default_deceleration: int = 1000,
            default_stop_type: motor.StopType = motor.SMART_BRAKE,
            steering_correction: float = 1,
            color_sensor: port.Port | None = None,
            force_sensor: port.Port | None = None,
            distance_sensor: port.Port | None = None,
            drive_pair: motor_pair.MotorPair = motor_pair.PAIR_1) -> Gadget:
        '''Wait for the motion sensor and return a configured robot controller.'''
        robot = cls(
            left_motor=left_motor,
            right_motor=right_motor,
            wheel_diameter=wheel_diameter,
            turn_factor=turn_factor,
            default_drive_velocity=default_drive_velocity,
            default_turn_velocity=default_turn_velocity,
            default_acceleration=default_acceleration,
            default_deceleration=default_deceleration,
            default_stop_type=default_stop_type,
            steering_correction=steering_correction,
            color_sensor=color_sensor,
            force_sensor=force_sensor,
            distance_sensor=distance_sensor,
            drive_pair=drive_pair,
        )

        # Identify ports and set sensors if detected
        for i in port.A, port.B, port.C, port.D, port.E, port.F:
            type = robot._identify_port(i)
            robot.log("Port " + PORT_NAMES[i] + " is a " + type)
            if type == "color sensor" and robot._color_sensor is None:
                robot._color_sensor = i
            elif type == "distance sensor" and robot._distance_sensor is None:
                robot._distance_sensor = i
            elif type == "force sensor" and robot._force_sensor is None:
                robot._force_sensor = i

        # Check drive motors
        left_motor_type = MOTOR_TYPES.get(motor.info(robot._drive_motors[0])[0])
        right_motor_type = MOTOR_TYPES.get(motor.info(robot._drive_motors[1])[0])
        robot.log(f'Left motor is a {left_motor_type} motor on port {PORT_NAMES[left_motor]}.')
        robot.log(f'Right motor is a {right_motor_type} motor on port {PORT_NAMES[right_motor]}.')
        if left_motor_type != right_motor_type:
            robot.log("Error: drive motors are mismatched. This is almost certainly wrong!", error=True)
        motor_pair.pair(robot._drive_pair, robot._drive_motors[0], robot._drive_motors[1])

        # Reset yaw once we're stable
        robot.log(f"waiting for motion sensor to report stable")
        await runloop.until(motion_sensor.stable)
        motion_sensor.set_yaw_face(yaw_face)
        motion_sensor.reset_yaw(int(yaw_angle*10))
        robot.log(f"reset yaw to {yaw_angle}")
        return robot


    @classmethod
    def _identify_port(cls, port: port.Port) -> str:
        try:
            return f"{MOTOR_TYPES[motor.info(port)[0]]} motor"
        except OSError:
            pass

        try:
            color_sensor_module.color(port)
            return "color sensor"
        except OSError:
            pass

        try:
            distance_sensor_module.distance(port)
            return "distance sensor"
        except OSError:
            pass

        try:
            force_sensor_module.force(port)
            return "force sensor"
        except OSError:
            pass

        return "unknown"


    def log(self: Gadget, message: str, error: bool = False):
        timestamp = f"{(ticks_ms() - self._start_time)/1000:07.3f}s | {(ticks_ms() - self._last_log_time)/1000:07.3f}s |"
        self._last_log_time = ticks_ms()
        if not(error):
            print(timestamp, message)
        else:
            print(timestamp, "Error occurred!")
            raise SystemExit(message)


    def _check_parameters(
            self: Gadget,
            velocity: int | None = None,
            angle: float | None = None,
            acceleration: int | None = None,
            deceleration: int | None = None,
            distance: float | None = None,
            steering_correction: float = 1,
            left_motor: motor.Motor = port.B,
            right_motor: motor.Motor = port.A,
            wheel_diameter: str | float = "small",
            turn_factor: float = 1):
        '''Checks parameter values for correctness.'''
        assert velocity is None or 100 <= velocity <= 1050, "velocity must be between 100 and 1050"
        assert angle is None or -180 <= angle <= 180, "angle must be between -180 and 180 degrees"
        assert acceleration is None or 100 <= acceleration <= 10000, "acceleration must be between 100 and 10000"
        assert deceleration is None or 100 <= deceleration <= 10000, "deceleration must be between 100 and 10000"
        assert distance is None or 5 <= distance <= 100, "distance must be between 5 and 100cm"
        assert steering_correction >= 0, "steering correction must not be negative"
        assert left_motor != right_motor, "left and right motors must be different"
        assert wheel_diameter in ("small", "large") or (isinstance(wheel_diameter, (int, float)) and wheel_diameter > 0), "wheel diameter must be 'small', 'large', or a positive number"
        assert turn_factor > 0, "turn factor must be greater than 0"


    @staticmethod
    def yaw_angle() -> float:
        '''
        motion_sensor.tilt_angles() returns a (yaw, pitch, roll) tuple in decidegrees.
        [0] gets yaw, but the sign is reversed from the normal SPIKE UI values.
        multiply by -0.1 to get degrees with increasing values to the right.
        add 0 to prevent the angle from showing up as -0.0.
        '''
        yaw_angle = motion_sensor.tilt_angles()[0] * -0.1 + 0
        return yaw_angle


    def _reset_relative_drive_distance(self: Gadget):
        '''Reset relative position, we use this to track distance.'''
        for port in self._drive_motors:
            motor.reset_relative_position(port, 0)


    def _avg_relative_drive_distance_cm(self: Gadget) -> float:
        '''Calculate the relative distance driven as an average of the two motor relative distances.'''
        distances = (abs(motor.relative_position(self._drive_motors[0])), abs(motor.relative_position(self._drive_motors[1])))
        return sum(distances) / len(distances) / self._wheel_degrees_per_cm


    def _angle_difference(self: Gadget, angle: float, long_turn: bool = False) -> float:
        '''
        Calculate the difference between the current yaw angle and the desired angle. Wrap to -180 to 180.
            
        Args:
            angle: the desired angle to turn to.
            long_turn: by default steering uses the shorter turn direction. long_turn will use the longer turn direction instead.
        '''
        result = ((angle - self.yaw_angle() + 180) % 360) - 180
        if long_turn:
            result -= copysign(360, result)
        return result


    def _steering(self: Gadget, angle: float, long_turn: bool = False) -> int:
        '''Calculates the steering for move_for_degrees() based on the steering_correction.

        Steering is based on how far off we are from the correct angle but adjusted to be no more than 50.
        
        Args:
            angle: the desired angle to turn to.
            long_turn: by default steering uses the shorter turn direction. long_turn will use the longer turn direction instead.
        '''
        return int(max(-50, min(50, self._angle_difference(angle, long_turn) * self._steering_correction)))


    def _read_color(self: Gadget) -> color.Color:
        '''Read the color sensor and return the result as a Color.'''
        assert self._color_sensor is not None, "you must initialize color_sensor in Gadget.setup()!"
        return color_sensor.color(self._color_sensor)


    def _read_distance(self: Gadget) -> float:
        '''Read the distance sensor and return the value in cm as a float.'''
        assert self._distance_sensor is not None, "you must initialize distance_sensor in Gadget.setup()!"
        return distance_sensor.distance(self._distance_sensor) / 10


    def _read_pressed(self: Gadget) -> bool:
        '''Read the force sensor pressed value and return the result as a boolean.'''
        assert self._force_sensor is not None, "you must initialize force_sensor in Gadget.setup()!"
        return force_sensor.pressed(self._force_sensor)


    async def drive(self: Gadget,
                    distance: float,
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
        angle = self.yaw_angle() if angle is None else angle
        velocity = self._default_drive_velocity if velocity is None else velocity
        acceleration = self._default_acceleration if acceleration is None else acceleration
        self._check_parameters(velocity=velocity, acceleration=acceleration, angle=angle)

        self.log(f"driving {distance}cm at {angle}°")

        # Set distance to positive but retain the sign on the direction multiplier - 
        # this makes calculating driven distance easier.
        (distance, direction) = (abs(distance), int(copysign(1, distance)))

        # If we're too far from our desired angle, turn first.
        if abs(self._angle_difference(angle, long_turn=long_turn)) > 1:
            await self.turn(angle, long_turn=long_turn)
        
        # Start driving, continuously correcting in a loop. Break out when we reach our distance.
        self._reset_relative_drive_distance()
        while distance - self._avg_relative_drive_distance_cm() > 0:
            motor_pair.move(self._drive_pair, self._steering(angle, False) * direction,
                    velocity=velocity * direction, acceleration=acceleration)
            await runloop.sleep_ms(20)
        motor_pair.stop(self._drive_pair, stop=self._default_stop_type)


    async def setup_attachment(self: Gadget,
                               position: str,
                               port: port.Port,
                               angle: int = 0,
                               gear_ratio: float = 1,
                               zero_direction: str | None = None,
                               velocity: int | None = 320,
                               acceleration: int | None = 800):
        '''
        Reset an attachment motor's relative position

        Args:
            port: the motor's port
            position: "front" or "rear"
            angle: the angle to move the attachment to
            gear_ratio: the gear ratio of the current attachment (see https://marian42.de/gears/ or https://gears.sariel.pl/)
            zero_direction: "positive", "negative", or None - if set, the robot will attempt to zero the motor until it stalls
        '''

        self.log(f"reset_attachment | setting {position} attachment port={port}, gear ratio={gear_ratio}, position to {angle}°")
        self.log(f"reset_attachment | setting {position} attachment port={port}, velocity={velocity}, acceleration={acceleration}")

        velocity = self._default_drive_velocity if velocity is None else velocity
        acceleration = self._default_acceleration if acceleration is None else acceleration

        self._attachments[position] = {}
        self._attachments[position]["port"] = port
        self._attachments[position]["gear_ratio"] = gear_ratio
        self._attachments[position]["velocity"] = velocity
        self._attachments[position]["acceleration"] = acceleration

        direction = 1 if zero_direction == "positive" else -1

        if zero_direction is not None:
            motor.run(port, velocity * direction)
            while motor.status(port) != motor.STALLED:
                await runloop.sleep_ms(10)
            motor.stop(port, stop=motor.HOLD)
            self.log(f"setup_attachment | zeroed attachment {position}")

        motor.reset_relative_position(port, int(angle*gear_ratio))


    async def move_attachment(self: Gadget,
                   position: str | None = None,
                   angle: int | None = None,
                   velocity: int | None = None,
                   acceleration: int | None = None):
        '''
        Moves an attachment to a given relative position.

        Args:
            port: The port for the attachment motor
            position: the attachment position ("front" or "rear")
            velocity: The velocity used for the attachment in degrees/second (100-1050).
            acceleration: The acceleration value used for the attachment in degrees/second^2 (100-10000)
        '''
        self._check_parameters(angle=angle)
        self.log(f"lift | moving attachment {position} to {angle}°")

        port = self._attachments[position]["port"]
        gear_ratio = self._attachments[position]["gear_ratio"]
        velocity = self._attachments[position]["velocity"] if velocity is None else velocity
        acceleration = self._attachments[position]["acceleration"] if acceleration is None else acceleration
        self._check_parameters(velocity=velocity, acceleration=acceleration)

        await motor.run_to_relative_position(port, int(angle * gear_ratio), velocity, acceleration=acceleration) # type: ignore


    async def turn(self: Gadget,
                   angle: float,
                   long_turn: bool = False,
                   velocity: int | None = None,
                   acceleration: int | None = None,
                   deceleration: int | None = None):
        '''
        Tank turns to a global angle (0 degrees is where the robot is pointed when Gadget.setup() is called).

        Args:
            angle: The angle to turn to.
            long_turn: The default behavior is to turn the shortest direction possible to reach the heading. If long_turn is True, turn the longer direction.
            velocity: The velocity used for turning in degrees/second (100-1050).
            acceleration: The acceleration value used for turning in degrees/second^2 (100-10000)
        '''
        velocity = self._default_turn_velocity if velocity is None else velocity
        acceleration = self._default_acceleration if acceleration is None else acceleration
        deceleration = self._default_deceleration if deceleration is None else deceleration
        self._check_parameters(velocity=velocity, angle=angle, acceleration=acceleration, deceleration=deceleration)

        self.log(f"turning to {angle}°")

        await motor_pair.move_for_degrees(self._drive_pair, int(self._angle_difference(angle, long_turn) * self._turn_factor), 100, stop = self._default_stop_type, velocity=velocity, acceleration=acceleration, deceleration=deceleration)


    async def drive_until(self: Gadget,
                          colors: list[color.Color] | None = None,
                          sensor_distance: float | None = None,
                          pressed: bool | None = None,
                          angle: float | None = None,
                          reverse: bool = False,
                          long_turn: bool = False,
                          velocity: int | None = None,
                          acceleration: int | None = None):
        '''
        Drives at a given heading based on gyroscope yaw until colors are seen in a certain order.

        Args:
            colors: A list[] with any number of colors from the color module. The robot will drive until it has seen those colors in sequence.
            sensor_distance: The robot will drive until it sees a value less than or equal to this on the distance sensor.
            pressed: The robot will drive until the force sensor reports it is pressed.
            angle: The angle to drive at. Will turn to the angle first if not already facing that way. Defaults to current angle (but this is less precise!).
            reverse: if True, back up
            long_turn: The default behavior is to turn the shortest direction possible to reach the heading. If long_turn is True, turn the longer direction.
            velocity: The velocity used for driving in degrees/second (100-1050).
            acceleration: The acceleration value used for driving in degrees/second^2 (100-10000)

        Example:
            from colors import GREEN, WHITE, BLACK
            drive_until(GREEN, WHITE, BLACK, velocity=200)
        '''
        angle = self.yaw_angle() if angle is None else angle
        velocity = self._default_drive_velocity if velocity is None else velocity
        acceleration = self._default_acceleration if acceleration is None else acceleration
        self._check_parameters(velocity=velocity, angle=angle, acceleration=acceleration, distance=sensor_distance)
        direction = -1 if reverse else 1

        if colors is not None and sensor_distance is None and pressed is None:
            self.log(f"driving at {angle}° until colors: {[COLOR_NAMES[i] for i in colors]}")

            # If we're too far from our desired angle, turn first.
            if abs(self._angle_difference(angle, long_turn=long_turn)) > 1:
                await self.turn(angle, long_turn=long_turn, velocity=self._default_turn_velocity)

            # Make a copy of the color list before we modify it
            color_list = colors.copy()

            # Start driving. Each time we detect a color, remove it from the list and look for the next one.
            while color_list:
                motor_pair.move(self._drive_pair, self._steering(angle, False) * direction, velocity=velocity, acceleration=acceleration)
                if self._read_color() == color_list[0]:
                    self.log(f"detected color {COLOR_NAMES[colors[0]]}")
                    color_list.pop(0)
                await runloop.sleep_ms(10)
            motor_pair.stop(self._drive_pair, stop=self._default_stop_type)
        elif sensor_distance is not None and colors is None and pressed is None:
            self.log(f"driving at {angle}° until sensor distance: {sensor_distance}")
            
            # If we're too far from our desired angle, turn first.
            if abs(self._angle_difference(angle, long_turn=long_turn)) > 1:
                await self.turn(angle, long_turn=long_turn, velocity=self._default_turn_velocity)

            motor_pair.move(self._drive_pair, self._steering(angle, False) * direction, velocity=velocity, acceleration=acceleration)
            while True:
                if self._read_distance() <= sensor_distance:
                    motor_pair.stop(self._drive_pair, stop=self._default_stop_type)
                    break
                await runloop.sleep_ms(10)
        elif pressed is not None and sensor_distance is None and colors is None:
            self.log(f"driving at {angle}° until force sensor pressed")

            # If we're too far from our desired angle, turn first.
            if abs(self._angle_difference(angle, long_turn=long_turn)) > 1:
                await self.turn(angle, long_turn=long_turn, velocity=self._default_turn_velocity)

            motor_pair.move(self._drive_pair, self._steering(angle, False) * direction, velocity=velocity, acceleration=acceleration)
            while True:
                if self._read_pressed():
                    motor_pair.stop(self._drive_pair, stop=self._default_stop_type)
                    break
                await runloop.sleep_ms(10)
        else:
            self.log("unknown combination of parameters.", error=True)


    async def turn_until(self: Gadget,
                         colors: list[color.Color],
                         left_turn: bool = False,
                         velocity: int | None = None,
                         acceleration: int | None = None):
        '''
        Turns until colors are seen in a certain order.

        Args:
            colors: Any number of colors from the color module. The robot will drive until it has seen those colors in sequence.
            left_turn: if True, turns left. Defaults to right.
            velocity: The velocity used for driving in degrees/second (100-1050).
            acceleration: The acceleration value used for driving in degrees/second^2 (100-10000)

        Example:
            from colors import GREEN, WHITE, BLACK
            drive_until(GREEN, WHITE, BLACK, velocity=200)
        '''
        velocity = self._default_drive_velocity if velocity is None else velocity
        acceleration = self._default_acceleration if acceleration is None else acceleration
        self._check_parameters(velocity=velocity, acceleration=acceleration)
        direction = -1 if left_turn else 1

        self.log(f"turning until colors: {[COLOR_NAMES[i] for i in colors]}")

        # Make a copy of the color list before we modify it
        color_list = colors.copy()

        # Start turning. Each time we detect a color, remove it from the list and look for the next one.
        while color_list:
            motor_pair.move(self._drive_pair, 100 * direction, velocity=velocity, acceleration=acceleration)
            if self._read_color() == color_list[0]:
                self.log(f"detected color {COLOR_NAMES[colors[0]]}")
                color_list.pop(0)
            await runloop.sleep_ms(10)
        motor_pair.stop(self._drive_pair, stop=self._default_stop_type)
