#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from math import atan2, degrees
from geometry_msgs.msg import Twist, Point

class Actions:
    ''' Aqui iremos coordenar de maneira ordenada as possíveis ações a serem executadas pelo robô sem ordem lógica'''

    # Construtor
    def __init__(self, camera, laserScan, odometria):
        '''Constroi o objeto, além de coordenar de acordo com os sensores e tarefas a serem executadas,
        o funcionamento adequado das ações de movimentação'''
        
        self.camera = camera
        self.laserScan  = laserScan
        self.odometria  =  odometria
        self.pub  = rospy.Publisher("cmd_vel", Twist, queue_size=3)
        self.vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
    
    def controla_velocidade(self,v,w):
        '''Realiza controle da velocidade do Robô'''
        self.v = v
        self.w = w
        self.vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
        self.pub.publish(self.vel)

    
    def seguimento_linha(self):
        """Ordena o seguimento da linha"""
        # Receberá funções sensoriais da camera (regressão e centro de massa) e decidirá por onde o robô deve prosseguir
        pass

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


    



