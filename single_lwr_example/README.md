# single_lwr_example

This package is a guide-trough-example to show how to use the [kuka_lwr](https://github.com/CentroEPiaggio/kuka-lwr) packages.

## 1. Set your scenario

Create you robot and environment using the lwr model. 

For instance, in the package [__single_lwr_robot__](./single_lwr_robot/), the [robot](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/robot/single_lwr_robot.urdf.xacro) is a lwr mounted on a box, and the [environment](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/worlds/simple_environment.world) is just a ground plane with a sun.

[Controllers](./single_lwr_robot/config/controllers.yaml) (the controller manager configuration) are in the [config](./single_lwr_robot/config) folder. Note that the name space of controllers and the joint names contain the word `lwr`, which is the name you gave to the arm when creating the robot [here](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/robot/single_lwr_robot.urdf.xacro#L36).

__NOTE__: Other than standard ros-controllers, there are custom controllers that can be found in the [lwr_controllers](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_controllers) package.

## 2. Configure MoveIt 2

Use the setup assistant (`ros2 launch single_lwr_moveit setup_assistant.launch.py`) to configure a MoveIt package of your robot.

We already did that in the package [__single_lwr_moveit__](./single_lwr_moveit/). Try it without any robot or simulator (mock hardware) with `ros2 launch single_lwr_moveit demo.launch.py`.

Note that the [name of the controller and joint names](./single_lwr_moveit/config/moveit_controllers.yaml) must coincide with the ones given in the robot package [controllers configuration](./single_lwr_robot/config/controllers.yaml). The controller manager runs in the `lwr` namespace, hence the controller name `lwr/joint_trajectory_controller`.

__NOTE__: recall that to run a real LWR 4+, you must follow instructions in [lwr_hw](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_hw).

## 3. Set your launch control panel

We encourage to have a separated package with your launch file to configure your different uses of your robot, including the network parameters.

We have an example in the package [__single_lwr_launch__](./single_lwr_launch). To check which arguments are available use:

`ros2 launch single_lwr_launch single_lwr.launch.py --show-args`

## 4. Test in simulation

__a.__ Just bring your robot on:

`ros2 launch single_lwr_launch single_lwr.launch.py gui:=true use_rviz:=true`

By default, it does not load the MoveIt configuration. Eventhough, you can open `rqt`, and open Plugins->Robot Tools->Joint trajectory controller and move the robot manually with slides.

__b.__ Testing MoveIt the configuration in simulation:

`ros2 launch single_lwr_launch single_lwr.launch.py load_moveit:=true`

## 5. Test in real

__a.__ Once everything went well in simulation, you can procede to use the real robot by using:

`ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true`

__b.__ If you wish to use MoveIt, you can potentially use:

`ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true load_moveit:=true`

If the robot complains a lot about bad communication quality, then just launch the robot with the previous command, and after everything has started, then use:

`ros2 launch single_lwr_moveit move_group.launch.py allow_trajectory_execution:=true info:=true hardware_interface:=fri`

And that's it!  :metal:
