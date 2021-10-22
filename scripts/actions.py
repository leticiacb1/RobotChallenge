#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from math import atan2, degrees
from geometry_msgs.msg import Twist, Point
from Sensores import  *

class Actions:
    ''' Aqui iremos coordenar de maneira ordenada as possíveis ações a serem executadas pelo robô sem ordem lógica'''

    # Construtor
    def __init__(self, camera, laserScan, odometria):
        '''Constroi o objeto, além de coordenar de acordo com os sensores e tarefas a serem executadas,
        o funcionamento adequado das ações de movimentação'''
        self.camera = camera
        self.laserScan  = laserScan
        self.odometria  =  odometria
        self.pub  = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vel = Twist()
        self.v = 0
        self.o = 0

        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1

        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)

        self.x = 0
        self.y = 0 
        self.volta = 0

    def controla_velocidade(self):
        '''Realiza controle da velocidade do Robô'''
        self.vel.linear.x = self.v
        self.vel.angular.z = self.o
        self.pub.publish(self.vel)
        #rospy.loginfo("linear: %f angular: %f", self.vel.linear.x, self.vel.angular.z)
        self.rate.sleep()
    
    def seguimento_linha(self):
        """Ordena o seguimento da linha"""
        # Receberá funções sensoriais da camera (regressão e centro de massa) e decidirá por onde o robô deve prosseguir
        try:
            self.camera.set_cor("amarelo")
            self.cx,self.cy,self.h,self.w = self.camera.get_valores()
            inicio_x = (abs(self.odometria.positions()[0])>0 and abs(self.odometria.positions()[0])<0.3)
            inicio_y = (abs(self.odometria.positions()[1])>0 and abs(self.odometria.positions()[1])<0.3)
            estado_de_parada = self.camera.get_estado()
            if  inicio_x and  inicio_y and estado_de_parada==2:
                self.v = 0
                self.o = 0
                print("Volta Completada!")
            else:
                            
                err = self.cx - self.w/2
                self.v = 0.4
                self.o = -float(err) / 100
                if self.odometria.distancia_centro()>2 and self.volta == 0:
                    self.camera.set_estado(1)
                    self.volta+=1
        except rospy.ROSInterruptException:
	        print("Ocorreu uma exceção com o rospy")

        finally:
            self.controla_velocidade()

    def controla_garra(self):
        """Receberá a garra e irá coordenar suas ações"""
        pass  

    def procura_creeper(self, cor):
        """ Função responsável por identificar via centro de massa o creeper"""
        self.cor =  cor

    def retorna_pista(self):
        '''Irá controlar o retorno a pista'''
        pass

    def decisao_aruco(self):
        pass

if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
      

    
    



