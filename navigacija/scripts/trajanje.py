#!/usr/bin/env python3
import rosbag
from geometry_msgs.msg import Point, Twist

import sys
from math import sqrt

def distance(pt1, pt2):
    return sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)

if __name__ == '__main__':
    # U sys.arg[1] nalazi se argument
    # s kojim je program pokrenut
    argument = '-h'
    if len(sys.argv) > 1:
        argument = sys.argv[1]

    # Ispis uputa za korištenje programa
    if argument == '-h' or argument == '--help':
        print('Usage: trajanje.py <ime ROS vreće>.bag\n')
        print('Program računa trajanje navigacije za svaki cilj zapisan na '
              'temi /move_base_simple/goal.\nU vreći još trebaju biti zabilježene '
              'teme /amcl_pose i /lab_diff_drive/cmd_vel.')
        exit(0)
    bag = rosbag.Bag(argument)

    # Inicijaliziramo varijable
    start = Point()
    t_start = 0
    goal = Point()
    end = Point()
    t_end = 0

    # Imena tema definiramo kao varijable
    # To je korisno u slučaju da ih naknadno
    # želimo promjeniti
    amcl_topic = '/amcl_pose'
    cmd_topic = '/lab_diff_drive/cmd_vel'
    goal_topic = '/move_base_simple/goal'
    ###
    ### Ovdje dodajte kod za učitavanje podataka iz vreće
    ###

    # Ispis rezultata.
    # Potrebno je ispisati svaku ciljnu pozu.
    print(f'Početna poza:\n{start}\nCiljna poza:\n{goal}')
    print(f'Udaljenost: {distance(end,goal)}\nTrajanje: {t_end - t_start}\n')