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
    def __init__(self, camera):
        '''Constroi o objeto, além de coordenar de acordo com os sensores e tarefas a serem executadas,
        o funcionamento adequado das ações de movimentação'''
        self.camera = camera
        self.laserScan  = Laser()
        self.odometria  =  Odom()
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

        self.momento = 0
    
    def get_momento(self):
        return self.momento

    def set_momento(self,momento):
        self.momento = momento

    def controla_velocidade(self):
        '''Realiza controle da velocidade do Robô'''
        self.vel.linear.x = self.v
        self.vel.angular.z = self.o
        self.pub.publish(self.vel)
        #rospy.loginfo("linear: %f angular: %f", self.vel.linear.x, self.vel.angular.z)
        self.rate.sleep()

    def identifica_creeper(self):
        "Identifica creeper com base em id e cor"
        try:
            if self.camera.creeper_values()[0][0]!=0 and self.camera.get_ids()== self.camera.get_idCreeper():
                self.momento = 1
                
        except:
            pass

    def centraliza_creeper(self):
        "Centraliza e se movimenta em direção ao creeper"
        if self.momento == 1:
            if self.laserScan.get_dados()>0.15:
                print(f"ARUCO:{self.camera.get_corners()}")
                print(f"COR:{self.camera.creeper_values()[1][0]}")
                print(self.laserScan.get_dados())
                if abs(self.camera.creeper_values()[1][0]-self.camera.creeper_values()[0][0])>4:
                    if self.camera.creeper_values()[1][0]> self.camera.creeper_values()[0][0]:
                        self.v = 0.15
                        self.o = 0.15
                        
                    elif self.camera.creeper_values()[1][0]<self.camera.creeper_values()[0][0]:
                        self.v = 0.15
                        self.o = -0.15
                # try:

                #     if abs(self.camera.creeper_values()[1][0]-self.camera.get_corners())<75:
                #         if self.camera.creeper_values()[1][0]> self.camera.creeper_values()[0][0]:
                #             self.v = 0.1
                #             self.o = 0.15
                            
                #         elif self.camera.creeper_values()[1][0]<self.camera.creeper_values()[0][0]:
                #             self.v = 0.1
                #             self.o = -0.15
                #     else:
                #         self.v = 0
                #         if self.o>0:
                #             self.o = 0.2
                #         else:
                #             self.o = -0.2

                # except:
                if abs(self.camera.creeper_values()[1][0]-self.camera.creeper_values()[0][0])>4:
                    if self.camera.creeper_values()[1][0]> self.camera.creeper_values()[0][0]:
                        self.v = 0.15
                        self.o = 0.15
                        
                    elif self.camera.creeper_values()[1][0]<self.camera.creeper_values()[0][0]:
                        self.v = 0.15
                        self.o = -0.15
            else:
                self.momento = 2

            self.controla_velocidade()
        


    def seguimento_linha(self):
        """Ordena o seguimento da linha"""
        # Receberá funções sensoriais da camera (regressão e centro de massa) e decidirá por onde o robô deve prosseguir
        try:
            if self.momento == 0:
                self.identifica_creeper()
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
                elif self.odometria.distancia_centro()>2 and self.volta == 1 and self.odometria.positions()[0]>1:
                    self.camera.set_estado(2)
                    self.volta+=1
                    
        except rospy.ROSInterruptException:
	        print("Ocorreu uma exceção com o rospy")

        finally:
            self.controla_velocidade()

    def controla_garra(self):
        """Receberá a garra e irá coordenar suas ações"""
        self.v = 0
        self.o = 0
        self.controla_velocidade()
        for i in range(10000000):
            continue
        self.momento = 3

    def  volta_pista(self):
        if self.momento==3 and self.camera.get_contorno():
            self.v = -0.3 
            self.o = 0
            self.momento  = 3
        else:
            self.momento = 4
        self.controla_velocidade()

    def retorna_pista(self):
        '''Irá controlar o retorno a pista'''
        pass

    def decisao_aruco(self):
        pass

if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
      

    
    
