# row-following

## Introduction

`row-following` is a ROS 2 project designed to implement row-following behavior for the SpesBot robot. It includes nodes for navigation and control to keep the robot aligned within defined rows, which is particularly useful in agricultural robotics.

## Installation

### Make a ROS2 workspace and clone the repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/SpesRobotics/row-following.git src/row-following
```
### Install dependencies
```bash
sudo apt install python3-vcstool
vcs import src < src/row-following/row-following.repos
mv src/spes_autonomy/spesbot_description src/
rm -rf src/spes_autonomy
rosdep update
rosdep install --from-paths src --ignore-src -r
```

### Build the Project
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Source the setup script
```bash
source install/setup.bash
```
## Usage
### Before launching the examples change the Isaac source code at the location: `~/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.core_nodes/omni/isaac/core_nodes/ogn/python/nodes/OgnIsaacArticulationController.py`

At line `29` add this attribute:
```bash
self.flag = True
```
At line `50` change the `apply_action` method as follows:
```bash
  def apply_action(self, joint_positions, joint_velocities, joint_efforts):
          if self.initialized:
              joint_actions = ArticulationAction()
              joint_actions.joint_indices = self.joint_indices
  
              if self.flag:
                  if (np.size(joint_positions) > 0 or np.size(joint_velocities) > 0 or np.size(joint_efforts) > 0):
                      max_size = np.max([np.size(joint_positions), np.size(joint_velocities), np.size(joint_efforts)])
                      joint_positions[:max_size] = 0
                      joint_velocities[:max_size] = 0
                      joint_efforts[:max_size] = 0
                      joint_actions.joint_positions = joint_positions
                      joint_actions.joint_velocities = joint_velocities
                      joint_actions.joint_efforts = joint_efforts
                      self.controller_handle.apply_action(control_actions=joint_actions)
                      self.flag = False
              else:
                  joint_actions.joint_positions = joint_positions
                  joint_actions.joint_velocities = joint_velocities
                  joint_actions.joint_efforts = joint_efforts
                  self.controller_handle.apply_action(control_actions=joint_actions)
```
### Start the simulation `row_following_isaac/data/USDs/sim.usda`

### For generating a simulation with new agriculture field:
Run the script at the `~/ros2_ws/src/row-following/row_following_isaac/data/scripts/spawn_plants.py` location from the NVIDIA Isaac Sim script editor.

### For generating a dataset for YOLOv8 training run the following script:
```bash
cd ~/ros2_ws/src/row-following/row_following_isaac/scripts
python3 dataset_generator.py
```
### For launching the row-following node:
```bash
ros2 launch row_following_bringup complete_launch.py 
```
