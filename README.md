# IITB_Internship Project

This repository contains the code for the implementation of the decentralised control graph exploration algorithm as used in this paper:

Sarat N., L. Vachhani, A. Sinha: “Multi-robot Graph Exploration and Map Building with Collision Avoidance: A Decentralized Approach”, Journal of Intelligent and Robotic Systems, September 2016, Volume 83, Issue 3, pp 503-523.


The above mentioned paper's algorithm is implemented in ROS in the gazebo v7.8 simulator 

The main files associated with the code are:
1)main_algo.py
2)matrix_op.py
3)mybot_world.launch

To run the simulation:
roslaunch mybot_gazebo mybot_world.launch
