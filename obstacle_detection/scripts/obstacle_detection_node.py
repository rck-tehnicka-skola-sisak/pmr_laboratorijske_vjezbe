#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import  Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from math import cos, sin, dist, sqrt
from random import randrange
from obstacle_detection.msg import Obstacles

# jednostavni detektor prepreka
# NAPOMENA: pretpostavlja se da laser ne vidi 360°,
# odnosno ako vidi, prepreke se lome izmedu min i max kuta skena
class ObstacleDetection:
    def __init__(self):
        self.pub_obstacle_marker = rospy.Publisher("obstacle_marker", Marker, queue_size=1)
        self.pub_obstacles = rospy.Publisher("obstacles", Obstacles, queue_size=1)
        self.sub_laser_scan = rospy.Subscriber('scan', LaserScan, self.laser_scan_callback)

        # definicija dijela Marker poruke koji se ne mjenja
        self.marker = Marker()
        self.marker.ns = 'laser'
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.MODIFY
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.lifetime = rospy.Duration(1)

        # nasumicne boje za pojedinu prepreku (max. 25)
        self.colorpicker = []
        for i in range(0, 25):
            color = ColorRGBA()
            color.r = randrange(255)
            color.g = randrange(255)
            color.b = randrange(255)
            color.a = 255
            self.colorpicker.append(color)

    # radi jednostavnosti, pretpostavlja se da se prepreka sastoji
    # iskljucivo od uzastopnih tocaka skena
    def laser_scan_callback(self, scan):
        max_distance_between_points = rospy.get_param('~max_distance_between_points')
        min_obstacle_size = rospy.get_param('~min_obstacle_size')

        obstacles = Obstacles()
        # skupina tocaka koje cine prepreku
        # na pocetku popunjena s prvom tockom radi jednostavnosti
        cluster = [(scan.ranges[0] * cos(scan.angle_min),
                    scan.ranges[0] * sin(scan.angle_min))]

        # petlja za prolazak kroz sve tocke skena
        angles = [scan.angle_min + scan.angle_increment * x for x in range(0, len(scan.ranges))]
        for point in [(scan.ranges[i] * cos(angles[i]),
                       scan.ranges[i] * sin(angles[i]))
                       for i in range(0, len(scan.ranges))]:
            # ako je udaljenost izmedu tocaka manja od max_dist_between_points
            # onda su ova i prosla tocka dio iste prepreke
            # inace ako dvije susjedne tocke nisu dio iste prepreke treba zapoceti
            # novu prepreku i izracunati srediste i velicinu ove prepreke
            if dist(cluster[-1], point) > max_distance_between_points:
                obstacle_size = dist(cluster[0], cluster[-1])
                if obstacle_size > min_obstacle_size:
                    obstacles.sizes.append(obstacle_size)
                    obstacles.obstacles.append(
                      Point(sum([x[0] for x in cluster]) / len(cluster),
                            sum([y[1] for y in cluster]) / len(cluster), 0.0))
                cluster = []

            cluster.append(point)

        # potrebno je dodati posljednju prepreku i objaviti prepreke
        last_obstacle_size = dist(cluster[0], cluster[-1])
        if last_obstacle_size > min_obstacle_size:
            obstacles.sizes.append(last_obstacle_size)
            obstacles.obstacles.append(
              Point(sum([x[0] for x in cluster]) / len(cluster),
                    sum([y[1] for y in cluster]) / len(cluster), 0.0))

        # Zadatak:

        self.marker.header.frame_id = obstacles.header.frame_id = scan.header.frame_id
        self.marker.header.stamp = obstacles.header.stamp = rospy.Time.now()
        self.pub_obstacles.publish(obstacles)

        # vizulacizacija prepreke
        self.marker.points.clear()
        self.marker.colors.clear()
        for i in range(0, len(obstacles.obstacles)):
            self.marker.points.append(Point())
            self.marker.colors.append(self.colorpicker[i % 25])
            self.marker.points.append(Point(obstacles.obstacles[i].x, obstacles.obstacles[i].y, 0))
            self.marker.colors.append(self.colorpicker[i % 25])
            self.marker.points.append(Point())
            self.marker.colors.append(self.colorpicker[i % 25])
        self.pub_obstacle_marker.publish(self.marker)


if __name__ == '__main__':
    rospy.init_node('obstacle_detection')
    sf = ObstacleDetection()
    rospy.spin()
