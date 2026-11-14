# Behavior-Based Object Clearance Robot

An autonomous LEGO MINDSTORMS EV3 robot programmed in MicroPython. It explores its environment using simple behaviors, follows walls when needed, detects a marked goal area, and uses an ultrasonic sensor to search for nearby objects to clear.

![Behavior-based object clearance robot](https://user-images.githubusercontent.com/70733761/144649882-f12ad050-00ea-461c-bc11-b85f1fdfc699.png)

## What it does

The program combines a few focused behaviors to navigate a bounded area:

- **Wander:** drive in a chosen direction while monitoring the floor and nearby boundaries.
- **Align and follow walls:** use reflected-light readings from the color sensors to find and trace a wall.
- **Search for objects:** scan with the ultrasonic sensor and approach when an object is detected nearby.
- **Respond to the goal marker:** use red as the goal color, then perform a driving maneuver intended to move the object.

This is an experimental behavior-based robotics project, not a general-purpose navigation or obstacle-avoidance system. Sensor thresholds, distances, and turns are tuned in the source and may need adjustment for your robot and course.

## Hardware

The current program expects this EV3 configuration:

| Port | Component |
| --- | --- |
| A | Right drive motor |
| D | Left drive motor |
| S1 | Ultrasonic sensor |
| S2 | Left color sensor |
| S4 | Right color sensor |

The robot also needs a red goal marker in its environment. Check the port assignments in `behavior_based/main.py` and update them if your build differs.

## Requirements

- LEGO MINDSTORMS EV3 brick and a compatible two-motor drive base
- LEGO EV3 MicroPython v2.0 or later
- Two color sensors and one ultrasonic sensor, connected as listed above
- Visual Studio Code with the EV3 MicroPython extension (or another compatible workflow for transferring programs to the EV3)

## Run it

1. Connect the EV3 to your computer and open `behavior_based/main.py` in your EV3 MicroPython environment.
2. Confirm the sensor and motor ports match your robot.
3. Place the robot in a clear test area with the red goal marker and an object to detect.
4. Transfer and run `main.py` on the EV3.

Keep the robot on a contained course, supervise its movement, and be ready to stop it. Calibrate `wallColor`, the goal color, and the movement thresholds for your lighting, surface, and robot build.

## Related robotics projects

Explore more robotics work:

- [Robot path planning](https://github.com/Vyadavgit/Robot-path-planning)
- [Forward and inverse kinematics](https://github.com/Vyadavgit/Robotics-Forward-and-Inverse-Kinematics)
- [Robotics edge detection](https://github.com/Vyadavgit/Robotics-edge-detection-program)
- [Template matching using normalized convolution](https://github.com/Vyadavgit/Robotics-Template-Matching-Using-Normalized-Convolution)

These are separate projects and are not dependencies of this robot.
