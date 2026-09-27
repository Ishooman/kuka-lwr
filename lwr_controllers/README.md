# KUKA LWR CONTROLLERS

ROS 2 (ros2_control) package with various controllers implemented for the LWR 4+

## Overview

This package contains the implementation of different control strategies for the KUKA LWR 4+. 


## Usage

The desired controller can be ran with the aid of the launch files from the _single_lwr_launch_ package. 
The following example loads the one_task_inverse_kinematics controller:  
```ros2 launch single_lwr_launch single_lwr.launch.py controllers:=one_task_inverse_kinematics```  
Each of the controllers listed here can be loaded this way, either for use on a simulated robot or on a real one, depending on the value of the _use_lwr_sim_ parameter.  
With a robot already running, `ros2 launch lwr_controllers load_controller.launch.py controller:=one_task_inverse_kinematics` spawns controllers in `/lwr/controller_manager`.  

Also more than one controller can be loaded at once:  
```ros2 launch single_lwr_launch single_lwr.launch.py controllers:="joint_position_position_controller joint_trajectory_controller"```  
(please note the quotes surrounding the list of controllers to be loaded).  

Sometimes it is desired to load multiple controllers, but start only one of them, with the aim of switch between them at runtime with the controller manager.  
This can be achieved with the _stopped_controllers_ parameter:  
```ros2 launch single_lwr_launch single_lwr.launch.py controllers:="one_task_inverse_kinematics"  stopped_controllers:="computed_torque_controller"```  
and then:  
```ros2 control switch_controllers -c /lwr/controller_manager --deactivate one_task_inverse_kinematics --activate computed_torque_controller```  

Once the planning environment has loaded, commands can be sent to any joint in another terminal.

- Impedance Controller:
`ros2 topic pub --once /lwr/joint_impedance_controller/command std_msgs/msg/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- Inverse Dynamics Controller
`ros2 topic pub --once /lwr/inverse_dynamics_controller/ext_wrench geometry_msgs/msg/WrenchStamped '{wrench: {force:  {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0,y: 0.0,z: 0.0}}}'`

- Computed Torque Controller:
`ros2 topic pub --once /lwr/computed_torque_controller/command std_msgs/msg/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- One Task Inverse Kinematics Controller: 

  - Full pose (note the IDs)
`ros2 topic pub --once /lwr/one_task_inverse_kinematics/command lwr_controllers/msg/PoseRPY '{id: 0, position: {x: -0.4, y: 0.3, z: 1.5}, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

  - Position only
`ros2 topic pub --once /lwr/one_task_inverse_kinematics/command lwr_controllers/msg/PoseRPY '{id: 1, position: {x: -0.4, y: 0.3, z: 1.5}}'`

  - Orientation only
`ros2 topic pub --once /lwr/one_task_inverse_kinematics/command lwr_controllers/msg/PoseRPY '{id: 2, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

  - PID Gains setting
`ros2 param set /lwr/<controller> pid_lwr_a1_joint.p <value>` (read when the controller is configured)

- Multi Task Priority Inverse Kinematics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`ros2 topic pub --once /lwr/multi_task_priority_inverse_kinematics/command lwr_controllers/msg/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

  - PID Gains setting
`ros2 param set /lwr/<controller> pid_lwr_a1_joint.p <value>` (read when the controller is configured)

- Multi Task Priority Inverse Dynamics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`ros2 topic pub --once /lwr/multi_task_priority_inverse_dynamics/command lwr_controllers/msg/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

- Dynamic Sliding PID Control (Task Space)
`ros2 topic pub --once /lwr/dynamics_sliding_mode_controller_task_space_controller/command std_msgs/msg/Float64MultiArray '{data:[-0.4,0.3,1.5,0,0,0]}'`

- Minimum Effort Inverse Dynamics Controller:
just run the controller (no command support)

- Backstepping Controller:
just run the controller (no command support)






