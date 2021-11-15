#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from math import atan2, degrees
from geometry_msgs.msg import Twist, Point
from sensores import  *
from garra import Garra

class Acoes:
    ''' Aqui iremos coordenar de maneira ordenada as possíveis ações a serem executadas pelo robô sem ordem lógica'''

    # Construtor
    def __init__(self, camera):
        '''Constroi o objeto, além de coordenar de acordo com os sensores e tarefas a serem executadas,
        o funcionamento adequado das ações de movimentação'''
        self.camera = camera
        self.laserScan  = Laser()
        self.odometria  = Odom()
        self.neural = Redeneural()
        self.garra = Garra()
        self.pub  = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vel = Twist()
        self.estacao = None
        self.v = 0
        self.o = 0
        self.angulo = 0
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1

        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)

        self.x = 0
        self.y = 0 
        self.volta = 0

        self.estado = 0
    
    def get_estado(self):
        return self.estado

    def set_estado(self,estado):
        self.estado = estado

    def controla_velocidade(self):
        '''Realiza controle da velocidade do Robô'''
        self.vel.linear.x = self.v
        self.vel.angular.z = self.o
        self.pub.publish(self.vel)
        self.rate.sleep()

    def set_estacao(self,estacao):
        self.estacao = estacao

    def identifica_creeper(self):
        "Identifica creeper com base em id e cor"
        try:
            if self.camera.creeper_values()[0][0]!=0 and self.camera.get_ids()== self.camera.get_idCreeper():
                self.estado = 1
        except:
            pass

    def identifica_estacao(self):
        "Identifica rede neural"
        try:
            if self.neural.get_posicao() != 0 and self.neural.get_estacao()==self.estacao:
                self.estado = 5
        except:
            pass

    def centraliza_estacao(self):
        if self.estado == 5:
            if self.laserScan.get_dados()>0.4:
                if self.neural.get_posicao() > 0 :
                    self.v = 0.25
                    self.o = -0.1
                if self.neural.get_posicao() < 0:
                    self.v = 0.25
                    self.o = 0.1
            else:
                self.angulo = self.odometria.get_angulo()
                self.estado = 6
            self.controla_velocidade()


    def centraliza_creeper(self):
        "Centraliza e se movimenta em direção ao creeper"
        if self.estado == 1:
            if self.laserScan.get_dados()>0.18:
                self.garra.posiciona_garra()
                if abs(self.camera.creeper_values()[1][0]-self.camera.creeper_values()[0][0])>4:
                    if self.camera.creeper_values()[1][0]> self.camera.creeper_values()[0][0]:
                        self.v = 0.15
                        self.o = 0.1
                        
                    elif self.camera.creeper_values()[1][0]<self.camera.creeper_values()[0][0]:
                        self.v = 0.15
                        self.o = -0.1
            else:
                self.angulo = self.odometria.get_angulo()
                self.estado = 2

            self.controla_velocidade()
        
    def parada_pista(self):
        inicio_x = (abs(self.odometria.positions()[0])>0 and abs(self.odometria.positions()[0])<0.3)
        inicio_y = (abs(self.odometria.positions()[1])>0 and abs(self.odometria.positions()[1])<0.3)
        estado_de_parada = self.camera.get_curva()
        if  inicio_x and  inicio_y and self.estado==8:
            return True
        return False
        
    def sentido_correto(self):
        inicio_x = (abs(self.odometria.positions()[0])>0 and abs(self.odometria.positions()[0])<0.3)
        inicio_y = (abs(self.odometria.positions()[1])>0 and abs(self.odometria.positions()[1])<0.3)
        rotacao = self.odometria.get_angulo()>230  or (0<self.odometria.get_angulo()<70)
        if inicio_x and inicio_y and rotacao:
            return True
        return False

    def creepers_isolados_21(self):
        if self.camera.cor_creeper == "green" and self.camera.get_idCreeper() == 21:
            inicio_x = (self.odometria.positions()[0]>-2.5 and self.odometria.positions()[0]<-2.2)
            inicio_y = (self.odometria.positions()[1]>-3.15 and self.odometria.positions()[1]<-2.85)
            rotacao = (345<self.odometria.get_angulo()<360) or (0<self.odometria.get_angulo()<165)
            if inicio_x and inicio_y and rotacao:
                return True
        return False
    
    def creepers_isolados_52(self):
        if self.camera.cor_creeper == "green" and self.camera.get_idCreeper() == 52:
            inicio_x = (self.odometria.positions()[0]>2.2 and self.odometria.positions()[0]<2.5)
            inicio_y = (self.odometria.positions()[1]>-3 and self.odometria.positions()[1]<-2.6)
            rotacao = not((345<self.odometria.get_angulo()<360) or (0<self.odometria.get_angulo()<165))
            if inicio_x and inicio_y and rotacao:
                return True
        return False
        #if self.camera.get_ids()== 21 

    def seguimento_linha(self):
        """Ordena o seguimento da linha"""
        # Receberá funções sensoriais da camera (regressão e centro de massa) e decidirá por onde o robô deve prosseguir
        try:
            if self.sentido_correto() or self.creepers_isolados_21() or self.creepers_isolados_52():
                self.v = 0
                self.o = 0.5
            elif self.parada_pista():
                self.v = 0
                self.o = 0
                print("Volta Completada!")
                
            else:
                self.cx,self.cy,self.h,self.w = self.camera.get_valores()
                if self.estado == 0:
                    self.garra.inicio_garra()
                    self.identifica_creeper()
                    self.camera.set_cor("amarelo")
                elif self.estado ==4:
                    self.identifica_estacao()
                err = self.cx - self.w/2
                self.v = 0.3
                self.o = -float(err) / 100
                if self.odometria.distancia_centro()>2 and self.volta == 0:
                    self.camera.set_curva("direita")   
                    self.volta=1
                elif self.odometria.distancia_centro()>2 and self.volta == 1 and self.odometria.positions()[0]>2:
                    self.camera.set_curva("esquerda")
                    self.volta=2
                elif self.odometria.distancia_centro()>2 and self.volta == 2 and self.odometria.positions()[0]<-2:
                    self.camera.set_curva("direita")   
                    self.volta=1

        except rospy.ROSInterruptException:
	        print("Ocorreu uma exceção com o rospy")

        finally:
            self.controla_velocidade()

    def controla_garra(self):
        """Receberá a garra e irá coordenar suas ações"""
        self.v = 0
        self.o = 0
        self.controla_velocidade()
        self.garra.fecha_garra()
        rospy.sleep(1)
        self.garra.levanta_ombro()
        rospy.sleep(1)
        self.estado = 3

    def solta_garra(self):
        """Receberá a garra e irá coordenar suas ações"""
        self.v = 0
        self.o = 0
        self.controla_velocidade()
        self.garra.abaixa_ombro()
        rospy.sleep(1)
        self.garra.abre_garra()
        rospy.sleep(1)
        self.estado = 7

    def  volta_pista(self):
        if self.estado==3:
            if self.camera.get_contorno():
                if self.odometria.get_angulo()> self.angulo:
                    self.v = -0.5 
                    self.o = -0.1
                else:
                    self.v = -0.5 
                    self.o = 0.1
            else:
                self.estado = 4
        elif self.estado==7:
            if self.camera.get_contorno():
                if self.odometria.get_angulo()> self.angulo:
                    self.v = -0.5 
                    self.o = -0.1
                else:
                    self.v = -0.5 
                    self.o = 0.1
            else:
                self.estado = 8
        self.controla_velocidade()


    def decisao_aruco(self):
        pass

if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
      

    
    
