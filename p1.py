#!/usr/bin/env python
""" Nestor Morales De la Fuente 48772014Y nmoralesdlf@gmail.com
www.github.com/nestoregon/ua_mobile_robots

Obstacle avoidance algorithm.
Tested successfully in Gazebo 7.4 and ROS kinetic.

To run:
source ~/catkin_ws/src/devel/setup.bash # .zsh if your using a ZSH
roslaunch turtlebot_gazebo turtlebot_world.launch # starts Gazebo with turtlebot
rosrun package_name main.py # starts this rosnode

**Note that the range of the lidar is
"""

# imports
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# global variables
# Change the following values accordingly to modify the behaviour of the robot
# avoid values lower than 0.5 for MIN_BUMP_DISTANCE

MIN_BUMP_DISTANCE = 0.6 # min value to turn
ANGULAR_SPEED = 0.4     # angular speed
RATE = 10               # rate of the robot

# note the different speeds required for the different simulations.
LINEAR_SPEED = 0.5      # linear speed STAGE
# LINEAR_SPEED = 0.2      # linear speed GAZEBO


# class
class RobotClass():
    """ This is the robot class """

    def __init__(self, robot_name="robot1"):
        """ Constructor with default variable names """
        self.robot_name = robot_name # just in case we want to work with a multirobot system
        self.vel = Twist()
        self.scan = LaserScan()
        self.state = "stop"
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback_laser) # subscribe to
        self.pub_cmd = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=5) # publish to
        self.rate = rospy.Rate(RATE) # 0.1 seconds sleep
        self.laser_right = 0
        self.laser_left = 0
        self.laser_front = 0


    def callback_laser(self, msg):
        """ Laser callback.
        Divides the laser into 3 ranges: left, front and right.
        Gets the minimum value in each range and saves it in a class variable.
        Later those varibles are checked to decide the action of the robot
        """
        max_range = msg.angle_max
        min_range = msg.angle_min
        self.scan = np.array(msg.ranges) # get array of ranges of the laser

        where_are_NaNs = np.isnan(self.scan) # make nans 0
        self.scan[where_are_NaNs] = 1000 # so that it doesn't affect the calculations

        num_values = len(self.scan) # get the number of values to divide the array evenly
        section = (num_values)/3 # dividing the number of sections in 3 (left, front and right)
        # print(num_values, section)
        self.scan_left  = self.scan[section*2 : section*3]
        self.scan_front = self.scan[section*1 : section*2]
        self.scan_right = self.scan[section*0 : section*1]

        # print("Length", len(self.scan_left), len(self.scan_front), len(self.scan_right))
        self.laser_left  = np.min(self.scan_left[np.nonzero(self.scan_left)])
        self.laser_front = np.min(self.scan_front[np.nonzero(self.scan_front)])
        self.laser_right = np.min(self.scan_right[np.nonzero(self.scan_right)])


    # funtions to move the robot around
    def vel_stop(self):
        """ Stop """
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_straight(self):
        """ Go straight """
        self.vel.linear.x = LINEAR_SPEED
        self.vel.angular.z = 0.0
        self.pub_cmd.publish(self.vel)

    def vel_backwards(self):
        """ Go backwards """
        self.vel.linear.x = -LINEAR_SPEED
        self.vel.angular.z = -ANGULAR_SPEED
        self.pub_cmd.publish(self.vel)

    def vel_right(self):
        """ Go right """
        self.vel.linear.x = 0.0
        self.vel.angular.z = -ANGULAR_SPEED
        self.pub_cmd.publish(self.vel)

    def vel_left(self):
        """ Go left """
        self.vel.linear.x = 0.0
        self.vel.angular.z = ANGULAR_SPEED
        self.pub_cmd.publish(self.vel)

    # obstacle avoidance core function
    def check_state(self):
        """ Changing state of the program """
        rospy.loginfo("--------------")
        rospy.loginfo(self.laser_left) #left
        rospy.loginfo(self.laser_front) #front
        rospy.loginfo(self.laser_right) #right

        if self.laser_left < MIN_BUMP_DISTANCE :
            self.state = "turn_right"
        elif self.laser_right < MIN_BUMP_DISTANCE :
            self.state = "turn_left"
        elif self.laser_front < MIN_BUMP_DISTANCE  :
            self.state = "backwards"
        else:
            self.state = "straight"

    # loop
    def control(self):
        """ Control node """
        while not rospy.is_shutdown():
            self.check_state()
            # move accordingly to the state
            if self.state == "stop":
                self.vel_stop()
            elif self.state == "straight":
                self.vel_straight()
            elif self.state == "turn_right":
                self.vel_right()
            elif self.state == "turn_left":
                self.vel_left()
            elif self.state == "backwards":
                self.vel_backwards()

            # sleep
            self.rate.sleep()


def main():
    """Main function"""
    robot_class = RobotClass()
    robot_class.control()
    rospy.spin()

if __name__ == "__main__":
    # initialize node
    rospy.init_node('Main_control_node', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        print("Shutting down publisher")
        pass
