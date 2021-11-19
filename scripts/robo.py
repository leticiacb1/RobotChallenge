#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from sensores import *
from acoes import Acoes
from pyfiglet import Figlet  #  sudo apt install python3-pyfiglet

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
                
                self.camera.set_linear(f"Velocidade Linear: {self.actions.lin:.2f} m/s")
                self.camera.set_angular(f"Velocidade Angular: {self.actions.ang:.2f} m/s")
                if self.actions.volta_final == 2 and self.actions.odometria.positions()[0]>0 and self.actions.odometria.positions()[0]<0.3:
                    self.camera.set_texto(f"Circuito terminado")
                    print(f.renderText("Circuito Terminado"))
                else:
                    print(f.renderText(f"""Completando volta"""))
                    self.camera.set_texto(f"""Completando volta""")


    def circuitocompleto(self):
        "Completa de acordo com ações do usuário, seguindo máquina de estações"
        while not rospy.is_shutdown():
            self.camera.set_linear(f"Velocidade Linear: {self.actions.lin:.2f} m/s")
            self.camera.set_angular(f"Velocidade Angular: {self.actions.ang:.2f} m/s")
            if self.actions.get_estado() == 0:
                self.actions.seguimento_linha()
                self.camera.set_texto(f"Procurando Creeper {self.camera.get_corCreeper()} ID {self.camera.get_idCreeper()}")
                print(f.renderText(f"Procurando Creeper {self.camera.get_corCreeper()} ID {self.camera.get_idCreeper()}"))
            elif self.actions.get_estado() == 1:
                self.actions.centraliza_creeper()
                self.camera.set_texto("Achei o Creeper! Aproximando")
                print(f.renderText("Achei o Creeper! Aproximando"))
            elif self.actions.get_estado() == 2:
                self.actions.controla_garra()
                self.camera.set_texto("Capturando Creeper")
                print(f.renderText("Capturando Creeper"))
            elif self.actions.get_estado() == 3:
                self.actions.volta_pista()
                self.camera.set_texto("Voltando pra pista")
                print(f.renderText("Voltando pra pista"))
            elif self.actions.get_estado() == 4:
                self.actions.seguimento_linha()
                self.camera.set_texto(f"Procurando estacao {self.estacao}")
                print(f.renderText(f"Procurando estacao {self.estacao}"))
            elif self.actions.get_estado() == 5:
                self.actions.centraliza_estacao()
                self.camera.set_texto("Achei a Estacao! Aproximando!")
                print(f.renderText("Achei a Estacao! Aproximando!"))
            elif self.actions.get_estado() == 6:
                self.actions.solta_garra()
                self.camera.set_texto(f"Soltando Creeper")
                print(f.renderText("Soltando Creeper"))
            elif self.actions.get_estado() == 7:
                self.actions.volta_pista()
                self.camera.set_texto(f"Voltando pra pista")
                print(f.renderText("Voltando pra pista"))
            elif self.actions.get_estado() == 8:
                self.actions.completa_volta()
                if self.actions.volta_final == 2 and self.actions.odometria.positions()[0]>0 and self.actions.odometria.positions()[0]<0.5:
                    self.camera.set_texto(f"Circuito terminado")
                    print(f.renderText("Circuito Terminado"))
                    rospy.signal_shutdown("Final")
                else:
                    self.camera.set_texto(f"Finalizando circuito")
                    print(f.renderText("Finalizando circuito"))
                

if __name__=="__main__":
    cores = ["green","orange","blue"]
    f = Figlet(font='big') 
    print(f.renderText('SEJA BEM-VINDO'))
    print('''Deseja qual objetivo para cumprir?
    
    1 - Apenas seguir pista
    
    2 - Completar uma missão''')
    ans = int(input("Digite aqui: "))
    if ans ==1:
        robo = Robo()
        robo.dar_volta()
    elif ans == 2:
        print('''Digite aqui qual missão deseja executar \n''')
        cor = (input("Cor do Creeper aqui ('blue','orange','green'): "))
        if cor == "green":
            id = int(input("Id do Creeper aqui (13,21,23,52): "))
            if id not in [13,21,23,52]:
                print("Id inválida, recomece o programa")
                rospy.signal_shutdown("Id inválida, recomece o programa")
        elif cor =="orange":
            id = int(input("Id do Creeper aqui (11,21): "))
            if id not in [11,21]:
                print("Id inválida, recomece o programa")
                rospy.signal_shutdown("Id inválida, recomece o programa")      
        elif cor == "blue":
            id = int(input("Id do Creeper aqui (12,22,51): "))
            if id not in [12,22,51]:
                print("Id inválida, recomece o programa")
                rospy.signal_shutdown("Id inválida, recomece o programa")
        else:
            print("Cor inválida, recomece o programa")
            rospy.signal_shutdown("Cor inválida, recomece o programa")
        estacao = (input("Estacao desejada aqui (car,dog,cow,horse): "))
        if estacao not in ["car","dog","cow","horse"]:
            print("Estação inválida, recomece o programa")
            rospy.signal_shutdown("Estação inválida, recomece o programa")
        robo = Robo()
        robo.set_params(cor,id,estacao)
        robo.circuitocompleto()
