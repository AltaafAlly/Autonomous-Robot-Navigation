# Autonomous Robot Navigation Using SLAM, RRT, and Real-Time Path Planning

## Contributors
- Altaaf Ally (2424551)
- Hamzah Mia (2430188)
- Rayhaan Hanslod (2430979)
- Hamdullah Dadabhoy (2441030)

## Project Overview
This project demonstrates the implementation of an autonomous robot navigation system using Simultaneous Localization and Mapping (SLAM), Rapidly-exploring Random Trees (RRT), and real-time path planning. The key components include manual robot control, map generation with RViz and GMapping, path planning using RRT, and real-time navigation.

## Contents
- `Explorer.py`: Script to manually control the robot using keyboard inputs.
- `move_robot.py`: Script implementing the RRT algorithm for path planning.
- `RRT_Path_Finding.py`: Class handling real-time navigation of the robot.
- `report.pdf`: LaTeX report detailing the project implementation in IEEE format.

## Setup Instructions
### Install ROS and Dependencies
1. Ensure you have ROS installed on your system.

## Usage
- Manual Control: Use the keyboard to move the robot around and explore the environment.
- Mapping: Run the mapping process to generate a 2D map of the environment using SLAM.
- Path Planning: Execute the RRT algorithm to find a collision-free path from the robot's current position to a specified goal.
- Real-Time Navigation: Input (x, y) coordinates as the goal, and the robot will navigate towards it, avoiding obstacles dynamically.

## Report
The detailed project report is provided in the report.tex file, written in IEEE format. It includes explanations of the methodology, implementation details, and results of the project.

## Report Outline:
Introduction: Overview of the project and objectives.
Manual Robot Control: Description of the MOVE_ROBOT.py script for manual control.
Map Generation Using RViz and GMapping: Explanation of the mapping process, including the use of RViz and GMapping.
Path Planning Using RRT: Details on the implementation of the RRT algorithm and path planning process.
Real-Time Navigation: Description of real-time navigation, including the use of a PID controller for smooth movement.

## Acknowledgments
We would like to thank our instructors and peers for their guidance and support throughout this project.
