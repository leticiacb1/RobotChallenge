#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from sensores import *
from acoes import Acoes

__authors__ = ["Leticia Côelho","Lorran Caetano","Matheus Oliveira","Ykaro de Andrade"]

#================================CLASSES DE CONTROLE DO ROBÔ=================================#

class Robo:
    "Classe que define ações e parãmetros de busca das ações a serem executadas"
    def  __init__(self):
        "Recebe os outros objetos já descritos"
        rospy.init_node('robo')
        self.camera = Camera()
        self.actions = Acoes(self.camera)
        self.estacao = None

    def set_params(self,cor,id,estacao):
        "Exporta os parâmetros de definição da missão escolhida"
        self.camera.set_id_creeper(int(id))
        self.camera.set_cor_creeper(cor)
        self.actions.set_estacao(estacao)
        self.estacao = estacao

    def dar_volta(self):
        "Caso seja escolhido pelo usuário, robô apenas completa a volta, sem escolher creeper"
        while not rospy.is_shutdown():
            if self.actions.get_estado() == 0:
                self.actions.completa_volta()
                self.camera.set_texto("Completando volta")
                print("Completnado volta e parando")

    def circuitocompleto(self):
        "Completa de acordo com ações do usuário, seguindo máquina de estações"
        while not rospy.is_shutdown():
            if self.actions.get_estado() == 0:
                self.actions.seguimento_linha()
                self.camera.set_texto(f"Procurando Creeper {self.camera.get_corCreeper()} com ID Aruco{self.camera.get_idCreeper()}")
                print(f"Procurando Creeper {self.camera.get_corCreeper()} com ID Aruco{self.camera.get_idCreeper()}")
            elif self.actions.get_estado() == 1:
                self.actions.centraliza_creeper()
                self.camera.set_texto("Achei o Creeper! Se Aproximando")
                print("Achei o Creeper! Se Aproximando")
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
                self.camera.set_texto(f"Procurando estacao {self.estacao}")
                print(f"Procurando estacao {self.estacao}")
            elif self.actions.get_estado() == 5:
                self.actions.centraliza_estacao()
                self.camera.set_texto("Achei a Estacao! Aproximando!")
                print("Achei a Estacao! Aproximando!")
            elif self.actions.get_estado() == 6:
                self.actions.solta_garra()
                self.camera.set_texto(f"Soltando Creeper")
                print("Soltando Creeper")
            elif self.actions.get_estado() == 7:
                self.actions.volta_pista()
                self.camera.set_texto(f"Voltando pra pista")
                print("Voltando pra pista")
            elif self.actions.get_estado() == 8:
                self.actions.completa_volta()
                self.camera.set_texto(f"Finalizando circuito")
                print("Finalizando circuito")

if __name__=="__main__":
    cores = ["green","orange","blue"]
    
    print('''Deseja qual objetivo para cumprir?
    
    1 - Apenas seguir pista
    
    2 - Completar uma missão''')
    ans = int(input("Digite aqui: "))
    if ans ==1:
        robo = Robo()
        robo.dar_volta()
    elif ans == 2:
        print('''Digite aqui qual missão deseja executar \n''')
        cor = (input("Cor do Creeper aqui: "))
        id = int(input("Id do Creeper aqui: "))
        estacao = (input("Estacao desejada aqui: "))
        robo = Robo()
        robo.set_params(cor,id,estacao)
        robo.circuitocompleto()
