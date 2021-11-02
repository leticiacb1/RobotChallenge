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
            if self.actions.get_estado() == 0:
                self.actions.seguimento_linha()
            elif self.actions.get_estado() == 1:
                self.actions.centraliza_creeper()
            elif self.actions.get_estado() == 2:
                self.actions.controla_garra()
            elif self.actions.get_estado() ==3:
                self.actions.volta_pista()
            elif self.actions.get_estado() == 4:
                self.actions.seguimento_linha()

if __name__=="__main__":
    #cor =  input("Qual cor de creeper desejas: orange,blue ou green: ")
    #id =  input("Qual id desejas (11,12,13,21,22,23) ")
    cor, id = "blue", 22
    rospy.init_node('robo')
    camera  = Camera()
    camera.set_id_creeper(int(id))
    camera.set_cor_creeper(cor)
    actions = Actions(camera)
    robo = Robo(actions)
    robo.completar_volta()
