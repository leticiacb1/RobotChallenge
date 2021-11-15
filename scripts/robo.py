#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from sensores import *
from acoes import Acoes

class Robo:
    def  __init__(self):
        rospy.init_node('robo')
        self.camera = Camera()
        self.actions = Acoes(self.camera)

    def set_params(self,cor,id,estacao):
        self.camera.set_id_creeper(int(id))
        self.camera.set_cor_creeper(cor)
        self.actions.set_estacao(estacao)

    def completar_volta(self):
        while not rospy.is_shutdown():
            if self.actions.get_estado() == 0:
                self.actions.seguimento_linha()
            elif self.actions.get_estado() == 1:
                self.actions.centraliza_creeper()
            elif self.actions.get_estado() == 2:
                self.actions.controla_garra()
            elif self.actions.get_estado() == 3:
                self.actions.volta_pista()
            elif self.actions.get_estado() == 4:
                self.actions.seguimento_linha()
            elif self.actions.get_estado() == 5:
                self.actions.centraliza_estacao()
            elif self.actions.get_estado() == 6:
                self.actions.solta_garra()
            elif self.actions.get_estado() == 7:
                self.actions.volta_pista()
            elif self.actions.get_estado() == 8:
                self.actions.seguimento_linha()

if __name__=="__main__":
    cor, id,estacao = "green", 23, "cow"

    robo = Robo()
    robo.set_params(cor,id,estacao)
    robo.completar_volta()
