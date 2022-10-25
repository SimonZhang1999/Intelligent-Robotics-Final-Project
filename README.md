# Intelligent-Robotics

Our goal is to build a cruise robot with path planning, trajectory optimization and dynamic obstacle avoidance through the simulation of ROS system. In addition, considering the
robustness of the robot, the robot also needs to have the ability to automatically recover from the tracking dead cycle and escape multiple obstacles . In order to realize these functions, we
consider A * algorithm and RRT algorithm in path planning, but the result of A * algorithm may not find the optimal path. This problem also exists in the random point generation of
RRT algorithm, so although these two algorithms can quickly find the target point, we finally use the RRT * algorithm which can be asymptotically optimized based on sampling. In the
trajectory optimization, we use the mini snap algorithm which can obtain a smooth trajectory (although this algorithm is usually used for the three-dimensional path optimization of UAV). For dynamic obstacle avoidance during motion execution, we use MPC algorithm with
feedback adjustment to optimize motion control.
