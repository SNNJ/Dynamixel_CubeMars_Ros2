# Dynamixel_CubeMars_Ros2
Morpheus _ ROS2 interface codes for Dyanmixel and CubeMars motors



There are two main methods for creating an interface between ROS2 and Dynamixel motors. The more systematic approach involves using ROS2 Control, which employs a standardized architecture and multiple abstraction layers. This makes the interface universal and adaptable for different motors and hardware setups. Although this method has not yet been widely used for controlling Dynamixel or CubeMars motors, it is worth investigating due to its potential advantages.

To understand and learn more about ROS2 Control, you can watch these YouTube videos:

\begin{itemize}
    \item \href{https://www.youtube.com/watch?v=4QKsDf1c4hc&ab_channel=ArticulatedRobotics}{Video 1: Articulated Robotics}
    \item \href{https://www.youtube.com/watch?v=J02jEKawE5U&ab_channel=ArticulatedRobotics}{Video 2: Articulated Robotics}
\end{itemize}

For Dynamixel, there are already GitHub repositories that have implemented this method. Here is the link:
\href{https://github.com/dynamixel-community/dynamixel_hardware}{Dynamixel Hardware GitHub Repository}

\section{Direct Communication Method}

The other, simpler method involves directly communicating with the motor by writing a node that receives messages, creates packets, and sends them directly to the associated motors via a dedicated communication line. In this case, there are no abstraction layers, and it is not a universal interface. This is the method used for both Dynamixel and AKA motors. It involves developing or modifying code snippets to communicate directly with the hardware via a communication interface, while also adding an extra layer that interfaces with ROS.

The source code that serves as the basis for this method can be found in this GitHub repository:
\href{https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel}{Dynamixel SDK GitHub Repository}

\section{Rover-Dynamixel Interface Development}

The main source code used to develop our rover-Dynamixel interface is based on the \texttt{read\_write\_node.cpp}. It is highly recommended that you review this code thoroughly to understand it, as it serves as a foundation for further modifications or use of the derived code.

This code is a ROS2 node specifically designed to interface with DYNAMIXEL servo motors using the U2D2 communication device. It sets up a node named \texttt{read\_write\_node}, which subscribes to the topic \texttt{/set\_position}, allowing the motor's position to be set, and offers the \texttt{/get\_position} service to retrieve the current motor position. The comments within the code are very clear and provide sufficient explanation for further understanding.

