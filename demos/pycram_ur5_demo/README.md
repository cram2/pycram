# UR5 demo

The goal of this demo is to showcase the interfaces for connecting the Bullet World to the ROS ecosystem.
The demo works for both real and simulated UR5s.

### Prerequisites

1. rosbridge-server: `sudo apt install ros-melodic-rosbridge-server`

### Running the demo

1. Make sure your ROS workspace is sourced, and start the rosbridge server (wrapped in `ur5_demo_prerequisites.launch`
   for convenience):
   ```
   source ~/catkin_ws/devel/setup.bash
   roslaunch pycram ur5_demo_prerequisites.launch
   ```
   
2. **In a new shell** (do **not** source ROS here, we don't want anything from Python2 on our PATH), start the demo:
    ```
    python3 demos/pycram_ur5_demo/demo.py
    ```
    This will start the BulletWorld with the IAI kitchen, a UR5 and a cereal box. The simulation will run for 60 seconds.

3. Interact with the simulation and see the results in ROS:
    
    * Drag the cereal box around
    * Grab the robot by the gripper and move it
   
    You can see the poses of all objects published to `/tf`, the joint positions and velocities to 
   `/pycram/joint_state` and the forces and torques at the TCP to `/pycram/fts`:
    ```
    #  In a new shell
    source ~/catkin_ws/devel/setup.bash
    
   rostopic echo /tf
    rostopic echo /pycram/joint_state
   rostopic echo /pycram/fts
    ```
    
   `/tf` will only be updated when object poses actually change. Moving the cereal box around will cause updates to `/tf`.
