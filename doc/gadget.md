# Gadget module

The `gadget` module provides a high-level, asynchronous driving API for a LEGO Education SPIKE Prime robot. It combines paired drive motors, the hub yaw sensor, and optional color, distance, and force sensors.

> [!IMPORTANT]
> This module is designed for the [`lego-spikeprime-mindstorms-vscode`](https://github.com/phealy/lego-spikeprime-mindstorms-vscode/) extension. It does not run in standard desktop Python or in the LEGO Education SPIKE Prime app. When the extension sees `from gadget import *`, it merges `gadget.py` into the main program, making the module's SPIKE imports, color constants, and `Gadget` class available without additional imports.

## Quick start

Call `Gadget.setup()` inside an async entry point and run the entry point with `runloop`. The factory waits for the motion sensor to stabilize before resetting yaw:

```python
from gadget import *


async def main():
    robot = await Gadget.setup(
        left_motor=port.A,
        right_motor=port.B,
        wheel_diameter="small",
        turn_factor=1.447,
        color_sensor=port.C,
        distance_sensor=port.D,
        force_sensor=port.F,
    )

    await robot.drive(distance=-20, angle=0)
    await robot.drive_until(angle=0, colors=[GREEN])
    robot.log("completed")


runloop.run(main())
```

All movement methods are coroutines and must be called with `await`.

## Coordinate and motion conventions

- `yaw_angle=0` defines the robot's initial global heading unless another initial angle is supplied.
- Heading values range from `-180` to `180` degrees. Positive headings turn right; negative headings turn left.
- `drive()` accepts a negative distance to move backward.
- `drive_until()` uses `reverse=True` to move backward.
- Velocity is measured in motor degrees per second and must be from `100` through `1050`.
- Acceleration and deceleration are measured in motor degrees per second squared and must be from `100` through `10000`.
- Distance-sensor thresholds must be from `5` through `100` cm.

## `Gadget.setup()`

```python
await Gadget.setup(
    left_motor,
    right_motor,
    wheel_diameter,
    turn_factor,
    yaw_angle=0,
    yaw_face=motion_sensor.TOP,
    default_drive_velocity=320,
    default_turn_velocity=200,
    default_acceleration=1000,
    default_deceleration=1000,
    default_stop_type=motor.SMART_BRAKE,
    steering_correction=1,
    color_sensor=None,
    force_sensor=None,
    distance_sensor=None,
    drive_pair=motor_pair.PAIR_1,
)
```

Creates the controller, pairs its drive motors, waits for the motion sensor to stabilize, configures its yaw face, resets yaw to `yaw_angle`, and returns the controller.

| Parameter | Description |
| --- | --- |
| `left_motor` | **Required.** Port for the left drive motor. Must differ from `right_motor`. |
| `right_motor` | **Required.** Port for the right drive motor. Must differ from `left_motor`. |
| `wheel_diameter` | **Required.** `"small"` (5.6 cm), `"large"` (8.8 cm), or a positive custom diameter in centimeters. |
| `turn_factor` | **Required.** Positive conversion factor from desired yaw change to motor rotation. Calculate it with `turn_factor.py`. |
| `yaw_angle` | Initial global heading in degrees. Default: `0`. |
| `yaw_face` | Hub face pointing upward. Default: `motion_sensor.TOP`. |
| `default_drive_velocity` | Driving velocity. Default: `320`. |
| `default_turn_velocity` | Turning velocity. Default: `200`. |
| `default_acceleration` | Movement acceleration. Default: `1000`. |
| `default_deceleration` | Turn deceleration. Default: `1000`. |
| `default_stop_type` | Motor stop behavior. Default: `motor.SMART_BRAKE`. |
| `steering_correction` | Nonnegative gyro correction multiplier. Default: `1`. |
| `color_sensor` | Color-sensor port required for color-based movement. Default: `None`. |
| `force_sensor` | Force-sensor port required for press-based movement. Default: `None`. |
| `distance_sensor` | Distance-sensor port required for distance-based movement. Default: `None`. |
| `drive_pair` | Motor-pair identifier. Default: `motor_pair.PAIR_1`. |

## `log()`

```python
robot.log(message, error=False)
```

Prints `message` with both the elapsed time since the `Gadget` was created and the time since the previous log message. When `error=True`, raises `SystemError` with the timestamped message instead of printing it.

```python
robot.log("starting mission")
```

## `yaw_angle()`

```python
heading = Gadget.yaw_angle()
# or
heading = robot.yaw_angle()
```

Returns the current yaw heading as a `float` in degrees. The result uses the module's sign convention, where values increase to the right. This is a static method and does not require an instance.

## `drive()`

```python
await robot.drive(
    distance,
    angle=None,
    long_turn=False,
    velocity=None,
    acceleration=None,
)
```

Drives a distance in centimeters while correcting steering with the yaw sensor. If the requested heading differs by more than one degree, the robot turns to that heading before driving.

| Parameter | Description |
| --- | --- |
| `distance` | Distance in centimeters. Use a negative value to drive backward; use `0` to turn without driving. |
| `angle` | Global heading to maintain. Defaults to the current heading. |
| `long_turn` | Use the longer route when initially turning to `angle`. |
| `velocity` | Drive velocity, or the configured default when omitted. |
| `acceleration` | Drive acceleration, or the configured default when omitted. |

```python
await robot.drive(distance=30, angle=0)
await robot.drive(distance=-15, angle=0)
```

## `turn()`

```python
await robot.turn(
    angle,
    long_turn=False,
    velocity=None,
    acceleration=None,
    deceleration=None,
)
```

Tank-turns to an absolute global heading.

| Parameter | Description |
| --- | --- |
| `angle` | Target heading from `-180` through `180` degrees. |
| `long_turn` | Use the longer route to the target instead of the shortest route. |
| `velocity` | Turn velocity, or the configured turn default when omitted. |
| `acceleration` | Turn acceleration, or the configured default when omitted. |
| `deceleration` | Turn deceleration, or the configured default when omitted. |

```python
await robot.turn(angle=-90)
await robot.turn(angle=0, long_turn=True)
```

## `drive_until()`

```python
await robot.drive_until(
    colors=None,
    sensor_distance=None,
    pressed=None,
    angle=None,
    reverse=False,
    long_turn=False,
    velocity=None,
    acceleration=None,
)
```

Drives at a heading until exactly one configured sensor condition is met. Supply one of `colors`, `sensor_distance`, or `pressed`; combining conditions or omitting all conditions raises `SystemError`.

| Parameter | Description |
| --- | --- |
| `colors` | Ordered list of colors to detect. Requires `color_sensor` in `Gadget.setup()`. |
| `sensor_distance` | Stop when the distance sensor reads this value or less, in centimeters. Requires `distance_sensor` in `Gadget.setup()`. |
| `pressed` | Supply a non-`None` value to stop when the force sensor is pressed. Requires `force_sensor` in `Gadget.setup()`. |
| `angle` | Global heading to maintain. Defaults to the current heading. |
| `reverse` | Drive backward when `True`. |
| `long_turn` | Use the longer route when initially turning to `angle`. |
| `velocity` | Drive velocity, or the configured default when omitted. |
| `acceleration` | Drive acceleration, or the configured default when omitted. |

The method copies the supplied `colors` list before processing it, so the caller's list remains unchanged.

```python
await robot.drive_until(colors=[GREEN, WHITE], angle=0)
await robot.drive_until(sensor_distance=10, angle=0)
await robot.drive_until(pressed=True, reverse=True)
```

## `turn_until()`

```python
await robot.turn_until(
    colors,
    left_turn=False,
    velocity=None,
    acceleration=None,
)
```

Turns in place until the color sensor detects every color in the supplied order. A color sensor must be configured in `Gadget.setup()`.

| Parameter | Description |
| --- | --- |
| `colors` | Ordered list of colors to detect. |
| `left_turn` | Turn left when `True`; otherwise turn right. |
| `velocity` | Motor velocity, or the configured drive velocity when omitted. |
| `acceleration` | Acceleration, or the configured default when omitted. |

The method copies the supplied `colors` list before processing it, so the caller's list remains unchanged.

```python
await robot.turn_until(colors=[BLACK, WHITE], left_turn=True, velocity=200)
```

## Sensor examples

```python
from gadget import *

robot = await Gadget.setup(
    left_motor=port.A,
    right_motor=port.B,
    wheel_diameter="small",
    turn_factor=1.447,
    color_sensor=port.C,
    distance_sensor=port.D,
    force_sensor=port.F,
)

await robot.drive_until(colors=[GREEN, WHITE, BLACK])
await robot.drive_until(sensor_distance=15)
await robot.drive_until(pressed=True)
```

Only initialize ports for sensors physically connected to the hub. Calling a sensor-dependent method without its corresponding port raises an assertion error.
