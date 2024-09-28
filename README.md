# Dynamixel_CubeMars_Ros2
Morpheus _ ROS2 interface codes for Dyanmixel and CubeMars motors

There are two main methods for creating an interface between ROS2 and Dynamixel motors. The more systematic approach involves using ROS2 Control, which employs a standardized architecture and multiple abstraction layers. This makes the interface universal and adaptable for different motors and hardware setups. Although this method has not yet been widely used for controlling Dynamixel or CubeMars motors, it is worth investigating due to its potential advantages.

To understand and learn more about ROS2 Control, you can watch these YouTube videos:

https://www.youtube.com/watch?v=4QKsDf1c4hc&ab_channel=ArticulatedRobotics
https://www.youtube.com/watch?v=J02jEKawE5U&ab_channel=ArticulatedRobotics


For Dynamixel, there are already GitHub repositories that have implemented this method. Here is the link:
https://github.com/dynamixel-community/dynamixel_hardware

# Direct Communication Method

The other, simpler method involves directly communicating with the motor by writing a node that receives messages, creates packets, and sends them directly to the associated motors via a dedicated communication line. In this case, there are no abstraction layers, and it is not a universal interface. This is the method used for both Dynamixel and AKA motors. It involves developing or modifying code snippets to communicate directly with the hardware via a communication interface, while also adding an extra layer that interfaces with ROS.

The source code that serves as the basis for this method can be found in this GitHub repository:
https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel

Rover-Dynamixel Interface Development

The main source code used to develop our rover-Dynamixel interface is based on the read_write_node.cpp. It is highly recommended that you review this code thoroughly to understand it, as it serves as a foundation for further modifications or use of the derived code.

This code is a ROS2 node specifically designed to interface with DYNAMIXEL servo motors using the U2D2 communication device. It sets up a node named read_write_node, which subscribes to the topic /set_position, allowing the motor's position to be set, and offers the \get_position} service to retrieve the current motor position. The comments within the code are very clear and provide sufficient explanation for further understanding.



The source is available both in GitLab and my GitHub.

Functionality of the Code

The ROS interface uses Float64MultiArray. The reasoning behind using this message type is to allow communication with multiple motors simultaneously, supporting different control modes. For example:


ID (Motor ID), Position
ID, Velocity
ID, Position, Velocity


While controlling velocity via ID may not be directly relevant to robotic arms, the idea was to create a unified interface for all motors, even wheels’ motors, allowing for one controller to manage everything seamlessly. This same interface has been implemented for the AK motors of the arm, though it has not been implemented in the Morpheus, as a different version of the code is currently used for controlling the wheels and AK motors.

# ID, Position Mode Example

For this section, we will focus only on the ID, Position mode. Below is an example of sending a single command to Motor ID 1, setting its position to 60 via the \texttt{/motorCnt} topic:
ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray \
"{layout: {dim: [{label: 'ID', size: 2}, {label: 'Position'}], \
data_offset: 0}, data: [1, 60]}"
You can easily send commands to multiple motors simultaneously, like this:
ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray \
"{layout: {dim: [{label: 'ID', size: 4}, {label: 'Position'}], \
data_offset: 0}, data: [1, 60, 2, 180]}"

To simplify sending commands and avoid using lengthy commands, you can use the provided script manual_control.py. The script is dialogue-based, making it user-friendly and practical for testing purposes. Simply run the following command to execute it:
python3 manual_control.py

# Logic Behind the Code

When a message is delivered to the node, the motor ID is checked against a predefined list. If it is not in the list, the message is ignored. Otherwise, the position is checked against the position limits associated with each motor, and the command is sent to the motor to move to the desired position. The motor attempts to reach this position using the predefined values and velocity profiles that were set during motor initialization.

Once the command is sent, every 0.1 seconds, the motor's load is monitored. If the load on the motor is below a defined trigger threshold, the motor continues moving until the target position is achieved. However, if the load exceeds the threshold, the motor stops to prevent overload.


