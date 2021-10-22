#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from Sensores import *
from actions import Actions

class Robo:
    def  __init__(self, actions):
        self.actions = actions

    def completar_volta(self):
        while not rospy.is_shutdown():
            self.actions.seguimento_linha()

if __name__=="__main__":
    rospy.init_node('robo')
    odom = Odom()
    camera  = Camera()
    actions = Actions(camera, 1, odom)
    robo = Robo(actions)
    robo.completar_volta()
