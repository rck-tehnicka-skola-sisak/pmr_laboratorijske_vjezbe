#!/usr/bin/env python3
import rospy
from math import cos, sin, dist, sqrt, atan2, pi, copysign
from obstacle_detection.msg import Obstacles
from geometry_msgs.msg import Twist, PointStamped

class ObstacleFollowing:
    def __init__(self):
        self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub_point = rospy.Publisher("obstacle_position", PointStamped, queue_size=10)
        self.sub_obstacles = rospy.Subscriber('obstacles', Obstacles, self.obstacles_callback)

        self.max_obstacle_distance = rospy.get_param('~max_obstacle_distance')
        self.obstacle_size = rospy.get_param('~obstacle_size')
        self.max_obstacle_size_tolerance = rospy.get_param('~max_obstacle_size_tolerance')
        self.angle_tolerance = rospy.get_param('~angle_tolerance')
        self.follow_distance = rospy.get_param('~follow_distance')
        self.angular_speed = rospy.get_param('~angular_speed')
        self.linear_speed = rospy.get_param('~linear_speed')


    def obstacles_callback(self, obstacles):
        obstacle_distance = self.max_obstacle_distance
        obstacle_found = False
        obstacle_position = None

        # pronadi prepreku za slijediti
        for obstacle, size in zip(obstacles.obstacles, obstacles.sizes):
            # Zadatak 1:

        # ako nema kandidata za prepreku za slijediti, pokusaj ju pronaci okretanjem
        twist = Twist()
        twist.angular.z = self.angular_speed
        if not obstacle_found:
            rospy.loginfo_throttle(1, 'Searching for obstacle...')
            self.pub_twist.publish(twist)
            return

        # Zadatak 2:

        self.pub_twist.publish(twist)

        # Zadatak 3:


if __name__ == '__main__':
    rospy.init_node('obstacle_following')
    sf = ObstacleFollowing()
    rospy.spin()
