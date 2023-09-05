#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt

import sys
from math import sqrt

# Funkcija za izračun udaljenosti
def udaljenost(x, y):
    ud = 0

    ###
    ### Ovdje dodajte kod za izračun udaljenosti
    ###

    return ud

if __name__ == '__main__':
    # U sys.arg[1] nalazi se argument
    # s kojim je program pokrenut
    argument = '-h'
    if len(sys.argv) > 1:
        argument = sys.argv[1]

    # Ispis uputa za korištenje programa
    if argument == '-h' or argument == '--help':
        print('Usage: putanje.py <ime ROS vreće>.bag\n')
        print('Program računa prijeđene udaljenosti i iscrtava putanje '
              'sa tema /amcl_pose i /lab_diff_drive/odom.')
        exit(0)
    bag = rosbag.Bag(sys.argv[1])

    # Inicijaliziramo liste za učitavanje podataka
    x_amcl = []; y_amcl = []
    x_odom = []; y_odom = []
    # Imena tema definiramo kao varijable
    # To je korisno u slučaju da ih naknadno
    # želimo promjeniti
    amcl_topic = '/amcl_pose'
    odom_topic = '/lab_diff_drive/odom'

    ###
    ### Ovdje dodajte kod za učitavanje podataka iz vreće
    ###

    print(f'Prijeđena udaljenost prema AMCL algoritmu: {udaljenost(x_amcl, y_amcl)}')
    print(f'Prijeđena udaljenost prema odometrijie: {udaljenost(x_odom, y_odom)}')

    # Iscrtavanje putanja
    plt.figure()
    plt.plot(x_amcl, y_amcl)
    plt.plot(x_odom, y_odom)
    plt.xlabel('x/m'); plt.ylabel('y/m')
    plt.grid(); plt.legend(['AMCL', 'Odometrija'])
    plt.show()