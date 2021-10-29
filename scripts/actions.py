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
        self.laserScan  = Laser(0.22)
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

        # Atualizando objetivos:
        self.objetivo = 0              # Variável que guarda objetivos a serem seguidos. Atualizado pela classe Robô e em Actions.
        
        # Variáveis creeper:
        self.find_creeper = None
        self.centro , self.Xcreeper, self.area =   self.camera.get_creeperValues()

    def get_objetivo(self):
        return self.objetivo

    def set_objetivo(self, momento):
        self.objetivo = momento


    def controla_velocidade(self):
        '''Realiza controle da velocidade do Robô'''
        self.vel.linear.x = self.v
        self.vel.angular.z = self.o
        self.pub.publish(self.vel)
        #rospy.loginfo("linear: %f angular: %f", self.vel.linear.x, self.vel.angular.z)
        self.rate.sleep()
    
    def encontrou_creeper(self):   
        '''Identifica creeper correto por id e cor'''
        try:
            if(self.camera.achei_creeper):
                self.set_objetivo(1)  
                print("MUDANDO OBJETIVO")
        except:
            pass

    def seguimento_linha(self):
        """Ordena o seguimento da linha"""
        # Receberá funções sensoriais da camera (regressão e centro de massa) e decidirá por onde o robô deve prosseguir
        try:
            self.camera.set_cor("amarelo")                             # Mascara para o amarelo da pista
            self.cx,self.cy,self.h,self.w = self.camera.get_valores()
            
            inicio_x = (abs(self.odometria.positions()[0])>0 and abs(self.odometria.positions()[0])<0.3)
            inicio_y = (abs(self.odometria.positions()[1])>0 and abs(self.odometria.positions()[1])<0.3)
            
            estado_de_parada = self.camera.get_estado_na_pista()
            
            if  inicio_x and  inicio_y and estado_de_parada == 4:   #ALTERAR DEPOIS
                self.v = 0
                self.o = 0
                print("Volta Completada!")

            else:
                            
                err = self.cx - self.w/2
                self.v = 0.4
                self.o = -float(err) / 100

                if self.odometria.distancia_centro()>2 and self.volta == 0:
                    self.camera.set_estado_na_pista(1)                            # Curva a esquerda
                    self.volta+=1

                elif self.odometria.distancia_centro()>2 and self.volta == 1 and self.odometria.positions()[0]>1:
                    self.camera.set_estado_na_pista(2)                            # Curva a direita
                    self.volta+=1

        except rospy.ROSInterruptException:
	        print("Ocorreu uma exceção com o rospy")

        finally:
            self.controla_velocidade()

    def centralizar_creeper(self):
        '''Centraliza e segue até o creeper (Casos esse possua o id e a cor correta), utilizando a função segue_ate_creeper para parar'''

        if(self.get_objetivo() == 1):     # Já identificou o creeper correto
                try:
                    if(self.camera.XmedioId() is not None):
                        aruco = (self.camera.XmedioId())                        # X_centro aruco
                        
                        if(self.find_creeper):
                            if(self.centro > 1.02*aruco):
                                self.o = 0.2
                            elif(self.centro < 0.98*aruco):
                                self.o = -0.2
                            else:
                                self.segue_ate_creeper()
                except rospy.ROSInterruptException:
                    print("Ocorreu uma exceção com o rospy")
            
        self.controla_velocidade()
    
    def diff_centroCor_centroId(self):
        '''Função que identifica se a cor e o id são do creeper buscado. Para isso é analisada a distancia do centro do id para o centro da cor do creeper'''

        if(self.centro is not None):
            if(abs(self.centro - self.camera.XmedioId())<10):
                self.find_creeper = True
        self.fins_creeper = False

    def segue_ate_creeper(self):
        '''Segue em frente até atingir a distância estipulada'''
        emFrente = self.laserScan.keep_going()  # True caso possa continuar em frente

        if(emFrente):
            print("Em frente!")
            self.v = 0.08
        else:
            print("PARE!")
            self.v = 0
            self.o = 0

            #Atualiza objetivo  - Aṕos parar na distância pedida o robo deve agora pegar o creeper
            self.set_objetivo(2)
            print("Atualizando objetivo")

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
      

    
    



