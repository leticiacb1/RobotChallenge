#! /usr/bin/env python3
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def identifica_cor(frame, cor1, cor2): #agora recebe o frame e uma string com a cor(ex.: "red")
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    segmentado_cor = cv2.inRange(frame_hsv, cor1, cor2)
    
    centro = (frame.shape[1]//2, frame.shape[0]//2)

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 


    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
	
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)

    # Representa a area e o centro do maior contorno no frame
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

   # cv2.imshow('video', frame)
    cv2.imshow('seg', segmentado_cor)
    cv2.waitKey(1)

    return media, centro, maior_contorno_area

# Testando classes Laser e classe Camera
class Laser:

    def __init__(self, distancia):
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.segue = True                                                           # Variável que irá deterfinar se o robô segue ou não
        self.distancia = distancia
        self.dados = None

    # Recebe self.dados do sensor e distância de parada:
    def scaneou(self, dado):
        
        # print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
        # print("Leituras:")
        # print(np.array(dado.ranges).round(decimals=2))
        self.dados = np.array(dado.ranges)

        for i in range(5):

            if self.dados[i]< self.distancia:
                self.segue = False 
                return
            elif self.dados[i]> self.distancia:
                self.segue = True 
                return
            elif self.dados[359-i]< self.distancia:
                self.segue = False 
                return
            elif self.dados[359-i]> self.distancia :
                self.segue = True 
                return
            
            self.segue = True 
            return

    def keep_going(self):
        return self.segue
        
    def Subscriber(self):
        return self.subscriber

    def get_dados(self):
    	return self.dados

class Camera:

    def __init__(self,cor1 = None, cor2 = None):

        self.topico_imagem = "/camera/image/compressed"
        self.subscriber = rospy.Subscriber(self.topico_imagem , CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
       
        self.corLow = cor1      # Parâmetros para o filtro
        self.corHight = cor2

        self.bridge = CvBridge()

        self.centro = []        # Informações filtro de cor.
        self.mediaCor = []
        self.areaCor = 0.0

    # Inicilamente implementada apenas para filtro de cor:
    def roda_todo_frame(self, imagem):                                          # NÃO SEI SE É VIÁVEL IMPLEMENTAR
        print("Frame filtro")
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
           
            self.mediaCor, self.centro, self.areaCor = identifica_cor(cv_image, self.corLow, self.corHight)  # VER COMO O LORRAN CHAMOU EM AUXILIAR
            
            cv2.imshow("Camera", cv_image)                               #Acho que já é chamado em corModule
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def Subscriber(self):
        return self.subscriber 

if __name__=="__main__":

    rospy.init_node("le_scan")

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    laser = Laser(1)                     #Inserindo distancia de parada
    recebe_scan = laser.Subscriber()     #Sobrescrevendo no tópico

    cor1 = (148,50,50)                   # Setando uma cor para teste:
    cor2 = (152,255,255)

    camera = Camera(cor1, cor2)
    recebedor = camera.Subscriber()
    
    while not rospy.is_shutdown():
        pass
        #print(laser.get_dados())                                        #Testando classe laser.

        if (len(camera.mediaCor) > 1):
            if(camera.centro[0]>1.05*camera.mediaCor[0]):                # Testando classe câmera
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.15))
                velocidade_saida.publish(vel)
                rospy.sleep(0.2)

            elif(camera.centro[0]<0.95*camera.mediaCor[0]):
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.15))
                velocidade_saida.publish(vel)
                rospy.sleep(0.2)

            else:

                if(laser.keep_going()):
                    velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
                else: 
                    velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                
                velocidade_saida.publish(velocidade)
                rospy.sleep(2)
        
        else:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.15))
            velocidade_saida.publish(vel)
            rospy.sleep(0.3)


