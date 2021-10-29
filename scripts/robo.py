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
            if(self.actions.get_objetivo()==0):
                self.actions.seguimento_linha()
                self.actions.encontrou_creeper()        # Verificando que o creeper foi encontrado ou não
            elif(self.actions.get_objetivo()==1):
                self.actions.centralizar_creeper()
            elif(self.actions.get_objetivo()==2):
                print("2")
if __name__=="__main__":
    
    # Pegando inputs 
    cor =  input("Qual cor de creeper desejas: orange,blue ou green: ")
    id =  input("Qual id desejas (11,12,13,21,22,23) ")

    rospy.init_node('robo')

    camera  = Camera()
    
    camera.set_id_creeper(int(id))
    camera.set_cor_crepper(cor)

    actions = Actions(camera)
    robo = Robo(actions)

    robo.completar_volta()
