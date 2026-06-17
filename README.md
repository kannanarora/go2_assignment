# Go2 Human-Safe Animal Behaviours
COSC2814 - Programming Autonomous Robots

Autonomous dog-like behaviours for the Unitree Go2 quadruped.

The project report has information about architecture, design, and evaluation.

## Setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git
git clone https://github.com/kannanarora/go2_assignment.git

cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select go2_interfaces go2_utils go2_behaviours
source install/setup.bash
```

## Run

```bash
ros2 launch go2_behaviours wander.launch.py
```

## Packages

- `go2_interfaces` - shared messages
- `go2_utils` - camera, LiDAR, Whisper, person tracker
- `go2_behaviours` - MUX, wander, avoid, voice, person follow, sounds

## Team
- Kannan Arora - UG
- Allana Davies - PG
- Dilrukshi Perera - PG
- Mattias Tofte - PG