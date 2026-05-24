# Go2 Human-Safe Animal Behaviours
COSC2814 - Programming Autonomous Robots

## Project Overview
Autonomous dog-like behaviours for the Unitree Go2 quadruped robot.

## Software Architecture
- Tier 1 - Reactive: Safety monitor (always on)
- Tier 2 - Sequencing: Behaviour executor
- Tier 3 - Deliberative: Behaviour planner + Behaviour Tree
- Infrastructure: Sport client wrapper node

## Setup
### 1. Clone unitree_ros2 separately
git clone https://github.com/unitreerobotics/unitree_ros2

### 2. Clone this repo into your workspace
cd ~/go2_ws/src
git clone https://github.com/YOURUSERNAME/go2_assignment.git

### 3. Build
cd ~/go2_ws
colcon build
source install/setup.bash

## Team
- Kannan Arora - UG
- Allana Davies - PG
- Dilrukshi Perera - PG
- Mattias Tofte - PG