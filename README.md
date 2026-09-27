# KUKA LWR 4+ 

[![In Progress](https://badge.waffle.io/CentroEPiaggio/kuka-lwr.svg?label=in progress&title=in%20progress)](http://waffle.io/CentroEPiaggio/kuka-lwr) [![In Review](https://badge.waffle.io/CentroEPiaggio/kuka-lwr.svg?label=in review&title=in%20review)](http://waffle.io/CentroEPiaggio/kuka-lwr)

You need help? Press [<kbd>F1</kbd>](https://github.com/CentroEPiaggio/kuka-lwr/issues?utf8=%E2%9C%93&q=label%3Aquestion+) for FAQs, [issue](https://github.com/CentroEPiaggio/kuka-lwr/issues) your question, or [![Join the chat at https://gitter.im/CentroEPiaggio/kuka-lwr](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/CentroEPiaggio/kuka-lwr?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

## Overview

This is the **ROS 2** (Humble, `ros2_control`) version of the packages. The main packages are:
- [__lwr_description__](lwr_description): a package that defines the model of the robot (ToDo: name it __lwr_model__), including its `ros2_control` description.
- [__lwr_hw__](lwr_hw): a package that contains the LWR 4+ definition within the ros2_control framework, and also final hardware interface plugins using Kuka FRI, Stanford FRI Library or Gazebo (through `gazebo_ros2_control`). Read adding an interface below if you wish to add a different non-existing interface.
- [__lwr_controllers__](lwr_controllers): a package that implement a set of useful controllers (ToDo: perhaps moving this to a forked version of `ros2_controllers` would be ok, but some controllers are specific for the a 7-dof arm).
- [__single_lwr_example__](single_lwr_example): a cofiguration-based meta-package that shows how to use the `kuka_lwr` packages.
	- [__single_lwr_robot__](single_lwr_example/single_lwr_robot): the package where you define your robot using the LWR 4+ arm.
	- [__single_lwr_moveit__](single_lwr_example/single_lwr_moveit): the MoveIt 2 configuration for your `single_lwr_robot` description.
	- [__single_lwr_launch__](single_lwr_example/single_lwr_launch): a launch interface to load different components and configuration of your setup be it real, simulation, moveit, visualization, etc.

For an example using two LWR 4+ arms and two Pisa/IIT SoftHands, see the [Vito robot](https://github.com/CentroEPiaggio/vito-robot).

## Build

```bash
cd ~/ros2_ws/src && git clone https://github.com/CentroEPiaggio/kuka-lwr.git
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

The Stanford FRI library is downloaded and built with `lwr_hw` (it needs network access); disable it with
`colcon build --cmake-args -DLWR_HW_BUILD_FRIL=OFF`. The Gazebo plugin can be disabled with `-DLWR_HW_BUILD_GAZEBO=OFF`.

## Notes on the ROS 2 port

- The controller manager runs in the `/lwr` namespace (`/lwr/controller_manager`), so topics are as in ROS 1 (e.g. `/lwr/joint_states`, `/lwr/one_task_inverse_kinematics/command`). Use `ros2 control ... -c /lwr/controller_manager`.
- The hardware interface is selected with the `hardware_interface` xacro argument of the `kuka_lwr` macro: `gazebo`, `fri`, `fril` or `mock` (no robot, e.g. for the MoveIt demo). The FRI port/ip and the FRIL init file are xacro arguments too.
- Interfaces exported by `lwr_hw` for an arm called `lwr`: for each joint (`lwr_a1_joint` ... `lwr_a6_joint`) the command interfaces `position`, `effort`, `stiffness`, `damping`, `set_point` and the state interfaces `position`, `velocity`, `effort`, `stiffness`, `damping`; and the cartesian variables of the KRC in the `lwr_cart` gpio (`rot_xx` ... `pos_z`, `X_stiffness` ... `C_wrench`). In ROS 1 the stiffness, damping and set point were separate handles called `<joint>_stiffness`, etc.
- The control strategy of the arm is chosen from the command interfaces claimed by the controllers being started: `position` -> JOINT_POSITION, `effort`/`stiffness`/`damping`/`set_point` -> JOINT_IMPEDANCE, `lwr_cart/*` -> CARTESIAN_IMPEDANCE (they can't be mixed).
- The `stiffness_trajectory_controller` of the example is now `joint_stiffness_controller` (a `forward_command_controller`), since the ROS 2 trajectory controller can't command non-standard interfaces.
- `CartesianImpedancePoint` fields are lower case now: `x_fri`, `k_fri`, `d_fri`, `f_fri` (ROS 2 requirement).
- The controllers read the URDF from their `robot_description` parameter or, if empty, from the `/robot_description` topic.
- E-stop (FRI): while `/lwr/emergency_stop` is `true` the hardware ignores the controller commands and holds the measured position. Unlike ROS 1 the controllers are not restarted automatically, restart them before releasing the e-stop.
- For safety, the position and set point commands are initialized with the measured position at startup, and the set point is kept at the commanded position on every switch.

## Adding more interfaces/platforms

The package [lwr_hw](lwr_hw) contains the abstraction that allows to make the most of the ros2_control framework. The class [`lwr_hw::LWRHW`](lwr_hw/include/lwr_hw/lwr_hw.h) holds the state, commands, limits, KDL model and the control strategy switching logic. To create an instance of the arm you need to call [`bool LWRHW::create(const hardware_interface::HardwareInfo & info)`](lwr_hw/include/lwr_hw/lwr_hw.h), which reads the `name` hardware parameter (it MUST match the name you give to the xacro instance in the URDF) and the robot description (it can contain several lwr if they are called differently).

Adding an interface boils down to write a ros2_control plugin that inherits from the corresponding hardware interface base class (`hardware_interface::SystemInterface`, `gazebo_ros2_control::GazeboSystemInterface`, ...) and from `LWRHW`, forward `export_state_interfaces()`, `export_command_interfaces()`, `prepare_command_mode_switch()` and `perform_command_mode_switch()` to the `LWRHW` helpers, and implement `read()` and `write()` according to your final platform. This way you can use all your planning and controllers setup in any final real or simulated robot.

Examples of final interface implementations are found for the [Kuka FRI](lwr_hw/src/lwr_hw_fri.cpp), [Stanford FRI library](lwr_hw/src/lwr_hw_fril.cpp) and a [Gazebo simulation](lwr_hw/src/lwr_hw_gazebo.cpp).

## How to run a real LWR 4+

1. Load the script [`lwr_hw/krl/ros_control.src`](lwr_hw/krl/ros_control.src) and the corresponding [`.dat`](lwr_hw/krl/ros_control.src) on the robot. 
2. Place the robot in a position where joints 1 and 3 are as bent as possible (at least 45 degrees) to avoid the "__FRI interpolation error__" message. A good way to check this is to go into gravity compensation, and see if the robot enters succesfully in that mode with the configured tool.
3. Set the robot in __Position__ control.
4. Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button. You can also use the script in semi-automatic mode. 
5. Start the ROS 2 launch file (e.g. `ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true`), controllers will not start until the handshake is done.
6. From this point on, you can manage/start/stop/run/switch controllers from ROS, and depending on which interface they use, the switch in the KRC unit is done automatically. Everytime there is a switch of interface, it might take a while to get the controllers running again. If the controllers use the same interface, the switch is done only in ROS, which is faster.

### Additional information to use the Stanford FRI Library (not fully tested)

You need to provide your user name with real time priority and memlock limits higher than the default ones. You can do it permanently like this:

1. `sudo nano /etc/security/limits.conf` and add these lines: 
```
YOUR_USERNAME hard rtprio 95
YOUR_USERNAME soft rtprio 95
YOUR_USERNAME hard memlock unlimited
YOUR_USERNAME soft memlock unlimited
```
2. `sudo  nano /etc/pam.d/common-session` and add `session required pam_limits.so`
3. Reboot, open a terminal, and check that `ulimit -r -l` gives you the values set above.
4. Use the `fril` hardware interface: `ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true real_interface:=fril file:=/path/to/your-FRI-Driver.init`
5. Load the KRL script that is downloaded with the library to the Robot, and follow the instructions from the [original link](http://cs.stanford.edu/people/tkr/fri/html/) to set up network and other requirements properly.
