# ua_mobile_robots
Hostings assignments made at UA with ROS

## Assignment 1
Obstacle avoidance algorithm.
Tested successfully in Gazebo 7.4 and ROS kinetic.

Requirements:
- Gazebo 7.4 and ROS kinetic or higher. Keep in mind that turtlebot packages change in higher versions and the scaner might not work as intended.
- Make the file executable with ``chmod +x p1.py``. It should look green when running the command ``ls`` on the terminal.

Run on Gazebo
```bash
source ~/catkin_ws/src/devel/setup.bash # .zsh if your using a ZSH
roslaunch turtlebot_gazebo turtlebot_world.launch # starts Gazebo with turtlebot
rosrun package_name p1.py # starts this rosnode
```

Run on Stage
```bash
source ~/catkin_ws/src/devel/setup.bash # .zsh if your using a ZSH
rosrun stage_ros stageros /base_scan:=/scan emplo.world /cmd_vel:=/mobile_base/commands/velocity ejemplo.world
rosrun package_name p1.py # starts this rosnode
```

Changes in speed must be made in order to achieve a good performance. The stage simulation can have un to 1 m/s linear speed while the Gazebo simulation has been tested with accurate results with 0.2 m/s.
