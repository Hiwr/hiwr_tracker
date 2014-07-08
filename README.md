hiwr\_tracker
===============================================
 
The hiwr\_tracker ROS package aims to track objects or faces. It uses ROI data provided by another nodes, like hiwr\_opencv\_detector, and the current states of servomotors. The servomotor data are provided by a [ROS dynamixel package](http://wiki.ros.org/dynamixel_motor). It is fully developed in C++ in order to keep efficiency.

The behavior may be undefined when using two or more identical cameras.

Contributing
----------------------

Contributions via pull request are welcome and may be included under the
same license as below.

Copyright
----------------------

hiwr\_camera\_controller, except where otherwise noted, is released under the
[Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0.html).
See the LICENSE file located in the root directory.

How it works
----------------------

When this node is running, it takes as an input a [Region Of Interest message](http://docs.ros.org/api/sensor_msgs/html/msg/RegionOfInterest.html), and the state of each servomotor, with [JointState.msg](https://github.com/arebgun/dynamixel_motor/blob/master/dynamixel_msgs/msg/JointState.msg). Then, according to ROI data and JointState data, we are able to compute a displacement of the ROI from the center of the image. When this displacement is computed, we can set a speed proportional to the distance of the target, and move servomotors to the right direction. This tracking behavior was inspired by [rbx1 head\_tracker.py](https://github.com/pirobot/rbx1).

Build
----------------------
It requires [ROS dynamixel package](http://wiki.ros.org/dynamixel_motor) to work properly.

Execution
----------------------
Make sure that your project is compiled and sourced.

To start hiwr\_tracker, do the following (assuming you
have a working ROS core running):


Launch using roslaunch:

    $ roslaunch hiwr_tracker controller_manager.launch 

Launch from another launchfile:

    <include file="$(find hiwr_tracker)/launch/controller_manager.launch" />
 
Node
----------------------

### Subscribed Topics
- `/uvc_cam_node/roi`
 * The region of interest around the tracked object

- `/pan_joint/state`
 * The [JointState ]((https://github.com/arebgun/dynamixel_motor/blob/master/dynamixel_msgs/msg/JointState.msg) of the pan servomotor

- `/tilt_joint/state`
 * The [JointState ]((https://github.com/arebgun/dynamixel_motor/blob/master/dynamixel_msgs/msg/JointState.msg) of the tilt servomotor

### Published Topics

- `/pan_joint/command`
  * Set the goal angle to reach in radian (Float64)

- `/tilt_joint/command`
 * Set the goal angle to reach in radian (Float64)

### Services

- `/pan_joint/set_speed`
 * Set the new speed of the pan joint (Float64)

- `/tilt_joint/set_speed`
 * Set the new speed of the tilt joint (Float64)

- `/pan_joint/torque_enable`
 * Enable or disable pan joint, so it can be moved by hand (Bool)

- `/tilt_joint/torque_enable`
 * Enable or disable tilt joint, so it can be moved by hand (Bool)

### Parameters

- `max_joint_speed` (float, default= 1)
 * define the maximum reachable speed (common to all servos)

- `lead_target_angle` (float, default= 0.6)
 * When we are tracking, move current servo angle to  +/- lead\_target\_angle

- `pan_threshold` (float, default= 0.25)
 * Threshold percentage. Move pan servo if the target exceed the threshold.

- `tilt_threshold`(float, default= 0.25)
 * Threshold percentage. Move tilt servo if the target exceed the threshold.

- `gain_pan` (float, default= 0.8)
 * Adjust pan speed with this multiplicative factor

- `gain_tilt` (float, default= 0.8)
 * Adjust tilt speed with this multiplicative factor

- `image_height` (int, default= 480)
 * Image height of the tracking camera

- `image_width` (int, default = 640)
 * Image with of the tracking camera

- `pan_name` (string)
 * Given name of the pan from dynamixel node

- `tilt_name` (string)
 * Given name of the tilt from dynamixel node

- `rate` (int, default=5)
 * Frequency rate of hiwr_tracking node