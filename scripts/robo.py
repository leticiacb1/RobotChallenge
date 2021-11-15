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
                self.camera.set_texto("Achei o Creeper! Centralizando!")
                print("Achei o Creeper! Centralizando!")
            elif self.actions.get_estado() == 2:
                self.actions.controla_garra()
                self.camera.set_texto("Capturando Creeper")
                print("Capturando Creeper")
            elif self.actions.get_estado() == 3:
                self.actions.volta_pista()
                self.camera.set_texto("Voltando pra pista")
                print("Voltando pra pista")
            elif self.actions.get_estado() == 4:
                self.actions.seguimento_linha()
                self.camera.set_texto(f"Procurando estacao")
                print("Procurando estacao")
            elif self.actions.get_estado() == 5:
                self.actions.centraliza_estacao()
                self.camera.set_texto(f"Achei Estacao, se aproximando!")
                print("Achei Estacao, se aproximando!")
            elif self.actions.get_estado() == 6:
                self.actions.solta_garra()
                self.camera.set_texto(f"Soltando Creeper")
                print("Soltando Creeper")
            elif self.actions.get_estado() == 7:
                self.actions.volta_pista()
                self.camera.set_texto(f"Voltando pra pista")
                print("Voltando pra pista")
            elif self.actions.get_estado() == 8:
                self.actions.seguimento_linha()
                self.camera.set_texto(f"Finalizando circuito")
                print("Finalizando circuito")

if __name__=="__main__":
    cor, id,estacao = "green", 21, "cow"

    robo = Robo()
    robo.set_params(cor,id,estacao)
    robo.completar_volta()
