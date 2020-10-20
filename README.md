# ua_mobile_robots
Hostings assignments made at UA with ROS

## Assignment 1
Obstacle avoidance algorithm.
Tested successfully in Gazebo 7.4 and ROS kinetic.

Requirements:
- Gazebo 7.4 and ROS kinetic or higher. Keep in mind that turtlebot packages change in higher versions and the scaner might not work as intended.
- Make the file executable with ```chmod +x p1.py```. It should look green when running the command ``ls`` on the terminal.

Run on gazebo
```bash
source ~/catkin_ws/src/devel/setup.bash # .zsh if your using a ZSH
roslaunch turtlebot_gazebo turtlebot_world.launch # starts Gazebo with turtlebot
rosrun package_name p1.py # starts this rosnode
```

Run on 
```bash
source ~/catkin_ws/src/devel/setup.bash # .zsh if your using a ZSH
rosrun 
roslaunch turtlebot_gazebo turtlebot_world.launch # starts Gazebo with turtlebot
rosrun package_name p1.py # starts this rosnode
