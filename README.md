# Spinny - A High-Performance Line Follower

Spinny is a state-of-the-art line follower robot designed to navigate complex environments with ease and precision. Equipped with a QTR-8A reflectance sensor, two DC motors and an L293D motor driver, Spinny is capable of reaching high speeds while maintaining a stable trajectory.

## Components List
* 1 x Arduino Uno
* Zip-ties
* Power source (LiPo battery)
* 2 x Wheels
* Female-male wires for the line sensor
* 1 x QTR-8A reflectance sensor with screws
* 1 x Ball caster
* Extra wires
* 1 x Chassis
* 1 x Breadboard
* 1 x L293D motor driver
* 2 x DC motors

<img src ="https://i.imgur.com/qKG5RFa.png" alt="Components" style="width: 720px; height: 540px;"/>

## Features
* **Autonomous calibration**: Spinny is able to calibrate itself by performing a series of left and right movements, ensuring optimal performance for any environment.
* **Proportional-Integral-Derivative (PID) control**: Spinny utilizes a PID control algorithm to accurately track the line and make precise adjustments to its trajectory.
* **High-speed operation**: Spinny is capable of reaching speeds of up to 255 units while maintaining stability and control.
* **Robust design**: Spinny is built with durability in mind, featuring a sturdy chassis and secure motor attachments to withstand the demands of high-speed navigation.

## Design and Assembly
The ball caster serves as the support point for the robot and provides the necessary mobility to navigate a predefined path. The QTR-8A reflectance sensor is mounted on the front of the robot and is used to detect the position of the line. The sensor uses infrared reflectance to measure the intensity of the light reflected off a surface, allowing it to detect the position of a line.

The DC motors are mounted on the sides of the robot and are responsible for propelling the robot forward. The motors are secured in place using zip-ties to minimize vibration and ensure stability during operation. The L293D motor driver is used to control the speed and direction of the motors. It is mounted on the breadboard, which is also secured in place using pressure.

The Arduino Uno microcontroller serves as the brain of the robot and is responsible for processing sensor data, implementing the control algorithms and communicating with the motor driver. The robot is powered by a LiPo battery, which is connected to the VIN pin on the Arduino.

## Calibration
Before the robot can begin following a line, it must first undergo calibration. Calibration is the process of determining the minimum and maximum values of the sensor, allowing the robot to accurately detect the position of the line.

Spinny's calibration process is fully autonomous, initiated upon power-up. The robot performs a series of movements, moving left and right to ensure that the sensor readings are optimized for the current environment.

## PID Approach
Spinny utilizes a Proportional-Integral-Derivative (PID) control algorithm to track the line and make precise adjustments to its trajectory. The PID control system continuously calculates an error value as the difference between a desired setpoint and the current process variable. The controller then uses the error value to calculate and adjust the control output, in this case, the speed of the motors. The proportional, integral, and derivative constants (kp, ki, and kd) are used to fine-tune the performance of the controller, ensuring stable and accurate navigation.

## Control
The control system employed by the Spinny line follower is a PID controller. A PID controller is a control loop feedback mechanism that continuously calculates an error value as the difference between a desired setpoint and the current process variable. The controller then uses the error value to calculate and adjust the control output, in this case, the speed of the motors.

The proportional, integral and derivative constants (kp, ki, and kd) are used to fine-tune the performance of the controller. The kp constant is responsible for the proportion of the error that is used to adjust the control output. The ki constant is responsible for the accumulated error over time and is used to eliminate steady-state errors. The kd constant is responsible for the rate of change of the error and is used to eliminate oscillations.

## Performance
Spinny was put to the test in a controlled environment, where it was tasked with navigating a challenging route that included both straightaways and various types of curves. The route was specifically designed to test the capabilities of Spinny's sensors, control algorithms and overall design. In this test, Spinny successfully completed the route in **21.423 seconds**, showcasing its ability to navigate a complex course with precision and efficiency. The robot demonstrated its capability to follow straight lines as well as to take different types of curves, such as tight and wide ones, with great accuracy. This impressive result showcases the effectiveness of the design and programming of Spinny, and it's ability to adapt to different types of obstacles.

## Pictures of Spinny
<div style="display: inline-block">
  <img src ="https://i.imgur.com/OLfYfzj.jpg" alt="Picture 1 - Spinny" style="width: 360px; height: 480px;"/>
  <img src ="https://i.imgur.com/N8dgw56.jpg" alt="Picture 2 - Spinny" style="width: 360px; height: 480px;"/>
  <img src ="https://i.imgur.com/394LbIl.jpg" alt="Picture 3 - Spinny" style="width: 360px; height: 480px;"/>
  <img src ="https://i.imgur.com/ywpfERF.jpg" alt="Picture 4 - Spinny" style="width: 360px; height: 480px;"/>
</div>

## Video with Spinny in action
Click on the picture to see Spinny in action:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=c5-uvHWlw78" target="_blank"><img src="http://img.youtube.com/vi/c5-uvHWlw78/0.jpg" alt="Video - Snippy in action" width="480" height="360" border="10"></a>

## Why "Spinny"?
Click **[(here)](http://www.youtube.com/watch?feature=player_embedded&v=DC57d_xXY_A)** to find the answer.

## Repository of Teammate
The repository of my teammate can be found **[(here)](https://github.com/ZahariaDiana132/Spinny---line-follower-robot)**.

## Conclusion
Spinny is a highly advanced line follower robot, capable of navigating complex environments with ease and precision. Its robust design, autonomous calibration, and advanced control algorithm make it an ideal choice for a wide range of applications. With Spinny, you can be confident that your robot will perform at the highest level.
