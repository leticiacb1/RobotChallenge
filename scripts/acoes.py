#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
from math import atan2, degrees
from geometry_msgs.msg import Twist, Point
from sensores import  *
from garra import Garra
__authors__ = ["Leticia Côelho","Lorran Caetano","Matheus Oliveira","Ykaro de Andrade"]

#================================CLASSES DE AÇÂO=================================#

class Acoes:
    ''' Classe que tem como métodos as ações possíveis de serem executadas pelo robô no cenário'''

    # Construtor
    def __init__(self, camera):
        '''Constroi o objeto, além de coordenar de acordo com os sensores e tarefas a serem executadas,
        o funcionamento adequado das ações de movimentação'''
        #=====Sensores e membros importados==============#
        self.camera = camera
        self.laserScan  = Laser()
        self.odometria  = Odom()
        self.neural = Redeneural()
        self.garra = Garra()
        #=====Atributos para controle de velocidade==============#
        self.pub  = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vel = Twist()
        self.lin = 0
        self.ang = 0
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)
        #=====Atributo de controle da estação==============#
        self.estacao = None
        #=====Controle do estado e odometria do robo==============#
        self.x = 0
        self.y = 0 
        self.volta = 0
        self.volta_final = 0
        #=====Controle do estado de ação==============#
        self.estado = 0
         #=====Para seguir creeper 13, se assim selecionado==============#
        self.segue_creeper_13 = False   # Variável para creeper problemático 13
    
    #================================Controle de estado=================================#
    def get_estado(self):
        "Retorna estado atual da ação"
        return self.estado

    def set_estado(self,estado):
        "Modifica estado atual da ação"
        self.estado = estado

    def set_estacao(self,estacao):
        "Modifica estação de busca"
        self.estacao = estacao
    def set_volta(self,volta):
        "Modifica a volta em relação a curva"
        self.volta = volta
    #================================Controle de velocidade=================================#
    def controla_velocidade(self):
        '''Realiza controle da velocidade do Robô'''
        self.vel.linear.x = self.lin
        self.vel.angular.z = self.ang
        self.pub.publish(self.vel)
        self.rate.sleep()

    #================================Ações de identificação ou aproximação=================================#

    def identifica_creeper(self):
        "Identifica creeper com base em id e cor"
        try:
            
            if self.camera.creeper_values()[0][0]!=0 and self.camera.get_ids()== self.camera.get_idCreeper():
                self.estado = 1
        except:
            pass

    def identifica_estacao(self):
        "Identifica a estação solicitada com a rede neural"
        try:
            if (self.estacao == "horse" and self.odometria.positions()[0]<-2) or (self.estacao == "cow" and self.odometria.positions()[1]<0):
                pass
            else:
                if self.neural.get_posicao() != 0 and self.neural.get_estacao()==self.estacao:
                    self.estado = 5
        except:
            pass

    def centraliza_estacao(self):
        "Realiza centralização e aproximação da estação para soltar creeper"
        if self.estado == 5:
            if self.laserScan.get_dados()>0.9:
                if self.neural.get_posicao() > 0 :
                    self.lin = 0.25
                    self.ang = -0.1
                if self.neural.get_posicao() < 0:
                    self.lin = 0.25
                    self.ang = 0.1
            else:
                self.estado = 6
            self.controla_velocidade()


    def centraliza_creeper(self):
        "Centraliza e se movimenta em direção ao creeper"
        if self.estado == 1:
            if self.laserScan.get_dados()>0.18:
                self.garra.posiciona_garra()
                if abs(self.camera.creeper_values()[1][0]-self.camera.creeper_values()[0][0])>4:
                    if self.camera.creeper_values()[1][0]> self.camera.creeper_values()[0][0]:
                        if self.laserScan.get_dados()>0.4:
                            self.lin = 0.21
                            self.ang = 0.1
                        else:
                            self.lin = 0.15
                            self.ang = 0.1

                        
                    elif self.camera.creeper_values()[1][0]<self.camera.creeper_values()[0][0]:
                        if self.laserScan.get_dados()>0.4:
                            self.lin = 0.21
                            self.ang = -0.1
                        else:
                            self.lin = 0.15
                            self.ang = -0.1
            else:
                self.estado = 2
            self.controla_velocidade()
        
    #=================Ações de correção de sentido, parada ou busca de creeper ou estações que não são vistas no sentidode percorrimento da pista======#

    def parada_pista(self):
        "Gera a parada na pista após completar volta"
        inicio_x = self.odometria.positions()[0]>0 and self.odometria.positions()[0]<0.3
        inicio_y = self.odometria.positions()[1]>0 and self.odometria.positions()[1]<0.3
        estado_de_parada = self.camera.get_curva()
        if  inicio_x and  inicio_y and self.estado==8:
            return True
        return False

        
    def sentido_correto(self):
        "Corrige sentido do robô para percorrer a pista"
        inicio_x = self.odometria.positions()[0]>-0.1 and self.odometria.positions()[0]<0.45
        inicio_y = self.odometria.positions()[1]>-0.1 and self.odometria.positions()[1]<0.45
        rotacao = self.odometria.get_angulo()>230  or (0<self.odometria.get_angulo()<70)

        inicio_x_ponta = self.odometria.positions()[0]>-0.2 and self.odometria.positions()[0]<0.2
        inicio_y_ponta = self.odometria.positions()[1]>1.3 and self.odometria.positions()[1]<3
        rotacao_inversa =  250<self.odometria.get_angulo()<290

        if (inicio_x and inicio_y and rotacao):
            return True
        elif inicio_x_ponta and inicio_y_ponta and not rotacao_inversa:
            return True
        return False

    def estacao_car(self):
        "Posiciona robô para visualizar a estação Carro corretamente"
        estacao = self.estacao == "car"
        inicio_x = (self.odometria.positions()[0]>-3.2 and self.odometria.positions()[0]<-2.9)
        inicio_y = (self.odometria.positions()[1]>-3.15 and self.odometria.positions()[1]<-2.85)
        rotacao = self.odometria.get_angulo()>200  or (0<self.odometria.get_angulo()<70)
        if inicio_x and inicio_y and rotacao and estacao:
                return True
        return False

    def creepers_isolados_21(self):
        "Permite robô ter em seu campo de visão creeper verde com id 21"
        if self.camera.cor_creeper == "green" and self.camera.get_idCreeper() == 21:
            inicio_x = (self.odometria.positions()[0]>-2.5 and self.odometria.positions()[0]<-2.2)
            inicio_y = (self.odometria.positions()[1]>-3.15 and self.odometria.positions()[1]<-2.85)
            rotacao = (345<self.odometria.get_angulo()<360) or (0<self.odometria.get_angulo()<165)
            if inicio_x and inicio_y and rotacao:
                return True
        return False
    
    def creepers_isolados_52(self):
        "Permite robô ter em seu campo de visão creeper verde com id 52"
        if self.camera.cor_creeper == "green" and self.camera.get_idCreeper() == 52:
            inicio_x = (self.odometria.positions()[0]>2.2 and self.odometria.positions()[0]<2.5)
            inicio_y = (self.odometria.positions()[1]>-3 and self.odometria.positions()[1]<-2.6)
            rotacao = not((345<self.odometria.get_angulo()<360) or (0<self.odometria.get_angulo()<165))
            if inicio_x and inicio_y and rotacao:
                return True
        return False


    def creepers_isolados_13(self):
        "Permite robô ter em seu campo de visão creeper verde com id 13"
        if self.camera.cor_creeper == "green" and self.camera.get_idCreeper() == 13:
            inicio_x = (self.odometria.positions()[0]>-0.1 and self.odometria.positions()[0]<0.2)
            inicio_y = (self.odometria.positions()[1]>-0.1 and self.odometria.positions()[1]<0.2)
            rotacao = not((345<self.odometria.get_angulo()<360) or (0<self.odometria.get_angulo()<15))
            if(not(rotacao)):
                self.segue_creeper_13 = True

            if inicio_x and inicio_y and rotacao:
                return True

        return False
    def creeper_isolado_12(self):
        "Permite robô ter em seu campo de visão creeper azul com id 12"
        if self.camera.cor_creeper == "blue" and self.camera.get_idCreeper() == 12:
            inicio_x = (self.odometria.positions()[0]>-0.1 and self.odometria.positions()[0]<0.2)
            inicio_y = (self.odometria.positions()[1]>-0.1 and self.odometria.positions()[1]<0.2)
            rotacao = not(180<self.odometria.get_angulo()<210)
            if inicio_x and inicio_y and rotacao:
                return True
        return False
    #================================Ações de Seguimento de linha=================================#
    def seguimento_linha(self):
        """Ordena o seguimento da linha, além de controle de situações de parada ou correção"""
        # Receberá funções sensoriais da camera (regressão e centro de massa) e decidirá por onde o robô deve prosseguir
        try:
            if self.sentido_correto() and not(self.estado ==0):
                self.lin = 0
                self.ang = 0.5
                print("Ajeitando sentido!")
            
            elif self.estacao_car():
                self.lin = 0
                self.ang = 0.5

            elif self.estado == 0 and (self.creepers_isolados_21() or self.creepers_isolados_52() or self.creeper_isolado_12()):
                self.lin = 0
                self.ang = 0.5

            elif self.estado == 0 and  self.creepers_isolados_13():
                self.lin = 0
                self.ang = -0.5

            elif self.parada_pista():
                self.lin = 0
                self.ang = 0
                print("Volta Completada!")
                
            else:
                self.cx,self.cy,self.h,self.w = self.camera.get_valores()
                if self.estado == 0:
                    self.garra.inicio_garra()
                    self.identifica_creeper()
                elif self.estado ==4:
                    self.identifica_estacao()
                err = self.cx - self.w/2

                try:
                    self.lin = 0.22 + 0.28*math.cos(math.radians(self.camera.angulo_vertical))
                except:
                    self.lin = 0.3
                self.ang = -float(err) / 100
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

    def completa_volta(self):
        "Completa a pista, sem nenhum outro objetivo extra"
        try:
            
            self.cx,self.cy,self.h,self.w = self.camera.get_valores()
            inicio_x = self.odometria.positions()[0]>-0.1 and self.odometria.positions()[0]<0.5
            inicio_y = self.odometria.positions()[1]>-0.1 and self.odometria.positions()[1]<0.5
            
            if self.sentido_correto():
                self.lin = 0
                self.ang = 0.5
                print("Ajeitando sentido!")

            elif self.volta_final ==2 and inicio_x and inicio_y:
                self.lin  = 0
                self.ang = 0
                #print("Completei Volta")
            else:
                err = self.cx - self.w/2
                try:
                    self.lin =  0.22 + 0.28*math.cos(math.radians(self.camera.angulo_vertical))
                except:
                    self.lin = 0.3
                self.ang = -float(err) / 100
                if self.odometria.distancia_centro()>2 and self.volta_final == 0:
                    self.camera.set_curva("direita")   
                    self.volta_final=1
                elif self.odometria.distancia_centro()>2 and self.volta_final == 1 and self.odometria.positions()[0]>2:
                    self.camera.set_curva("esquerda")
                    self.volta_final=2
                
        except rospy.ROSInterruptException:
            print("Ocorreu uma exceção com o rospy")

        finally:
            self.controla_velocidade()

    #================================Ações de uso da garra=================================#
    def controla_garra(self):
        """Receberá a garra e irá coordenar suas ações de capturar"""
        self.lin = 0
        self.ang = 0
        self.controla_velocidade()
        self.garra.fecha_garra()
        rospy.sleep(1)
        self.garra.levanta_ombro()
        rospy.sleep(1)
        self.estado = 3

    def solta_garra(self):
        """Receberá a garra e irá coordenar suas ações de soltar"""
        self.lin = 0
        self.ang = 0
        self.controla_velocidade()
        self.garra.abaixa_ombro()
        rospy.sleep(1)
        self.garra.abre_garra()
        rospy.sleep(1)
        self.estado = 7

    #================================Ações de retorno a pista=================================#
    def  volta_pista(self):
        "Função que coordena e possibilita o robô se posicionar de volta a pista"
        if self.estado==3:
            self.camera.set_mascara_recorte(False)
            if self.camera.get_contorno()<10:
                self.lin = 0
                self.ang = 0.5
            elif self.camera.get_contorno()<250:
                self.cx,self.cy,self.h,self.w = self.camera.get_valores()
                err = self.cx - self.w/2
                self.ang = -float(err) / 100
                self.lin = 0.25
            else:
                self.camera.set_mascara_recorte(True)
                self.estado = 4
                
        elif self.estado==7:
            self.camera.set_mascara_recorte(False)
            if self.camera.get_contorno()<10:
                self.lin = 0
                self.ang = 0.5
               
            elif self.camera.get_contorno()<250:
                self.cx,self.cy,self.h,self.w = self.camera.get_valores()
                err = self.cx - self.w/2
                self.ang = -float(err) / 100
                self.lin = 0.25
            else:
                self.camera.set_mascara_recorte(True)
                self.estado = 8
        self.controla_velocidade()


if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
      

    
    
