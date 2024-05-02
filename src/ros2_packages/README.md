# Robot controller in ROS2

## Demo
___

1. clone repository
2. build workspace
    ```bash
   colcon build --symlink-install
    ```
3. source workspace
    ```bash
   source install/setup.bash
    ```

4. run gazebo and robot state publisher
    ```bash
   ros2 launch raros_navigation gazebo.launch.py world:=src/raros_navigation/worlds/test.world
    ```
   
5. run rviz
    ```bash
   ros2 launch raros_navigation display.launch.py
    ```
   
6. run teleop
    ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```