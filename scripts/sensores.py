# Imports:
from __future__ import print_function, division
import rospy
import numpy as np
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
import cv2.aruco as aruco
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String,Float64

from auxiliar import *

# Classes de sensores que serão chamadas pelo robô e pela máquina de estações.

# Classe do sensor laser, com algumas de suas funções principais:
class Laser:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.segue = True                                                           # Variável que irá deterfinar se o robô segue ou não
        self.distancia = None
        self.dados = None

    # Recebe self.dados do sensor e distância de parada:
    def scaneou(self, dado):
        self.dados = np.array(dado.ranges).round(decimals=2)
        #print(self.dados[0])


    def keep_going(self):
        return self.segue
    
    def get_dados(self):
        if self.dados is not None:
    	    return self.dados[0]

    def Subscriber(self):
        return self.subscriber


# Classe do sensor Odometria, com algumas de suas funções principais:
class Odom:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.recebe_odometria)
        self.x = 0
        self.y = 0
        self.z = 0
        self.contador = 0
        self.pula = 10
        self.angulo = 0

    def recebe_odometria(self, data):
        "Função assincrona para registrar odometria"
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista))    
        self.angulo = angulos[2]
        if self.contador % self.pula == 0:
            pass
        if self.angulo > 360:
           self.angulo  -= 360
        elif self.angulo  < 0:
            self.angulo  += 360
        #print(angulos[2])
        #print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(self.x, self.y,self.angulo))
        self.contador += 1

    def distancia_centro(self):
        "Retorna distancias relativas ao ponto de partida"
        return np.sqrt(self.x**2 + self.y**2)

    def positions(self):
        "Retorna posições via odometria"
        return (self.x,self.y)

    def get_angulo(self):
        return self.angulo

    def Subscriber(self):
        return self.subscriber

 #Classe de obteção do node da estacao       
class Redeneural:
    def __init__(self):
        self.recebedor = rospy.Subscriber("/estacao", String, self.recebe_estacao)
        self.neural_position = rospy.Subscriber("/corner", Float64, self.recebe_posicao)
        self.estacao = None
        self.posicao = 0 

    def recebe_estacao(self,msg):
        try:
            self.estacao = msg.data
        except:
            pass
    def recebe_posicao(self,msg):
        try:
            self.posicao = msg.data
        except:
            pass
    def get_estacao(self):
        return self.estacao
    
    def get_posicao(self):
        return self.posicao

#  Classe do sensor Câmera, com algumas de suas funções principais:
class Camera:

    def __init__(self):
        #Essenciais para captar imagens
        self.topico_imagem = "/camera/image/compressed"
        self.subscriber = rospy.Subscriber(self.topico_imagem , CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        self.bridge = CvBridge()   # Para compressed image
        self.cv_image = None
        #Para seguir linha
        self.cor = ""
        self.mask = None
        self.M = None
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        #Para identificar aruco
        self.vision =  None
        self.gray = None
        self.calib_path  = "../aruco_assets/"
        self.camera_matrix   = np.loadtxt(self.calib_path+'cameraMatrix_raspi.txt', delimiter=',')
        self.camera_distortion   = np.loadtxt(self.calib_path+'cameraDistortion_raspi.txt', delimiter=',')
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.marker_size  = 3
        self.ids  = None
        self.corners = None
        self.dist_aruco = 0
        self.ret = None
        self.rvec = None
        self.tvec = None
        self.curva = "esquerda"
        #Para centralizar em creeper
        self.centro = []           # Informações filtro de cor.
        self.mediaCor = []
        self.areaCor = 0.0
        self.cor_creeper = None
        self.id_creeper = None
        #Para voltar a pista
        self.maiorcontorno = None
        #texto
        self.text = None

    #getters
    def get_ids(self):
        "Retorno das ids aruco identificadas"
        try:
            return self.ids[0][0]
        except:
            return None

    def get_corners(self):
        m  = 0
        if self.get_ids() == self.get_idCreeper(): 
            try:
                for i in self.corners[0]:
                    m += i[0][0]
                return m
            except:
                pass
        else:
            return 0
    def aruco_distance(self):
        "Retorno da distancia da aruco ao robo"
        return self.dist_aruco

    def get_curva(self):
        "Retorno da virada da curva da pista"
        return self.curva

    def get_valores(self):
        "Getter de valores para aproveitamento em ações executadas"
        return self.cx, self.cy, self.h, self.w
    
    def creeper_values(self):
        "Getter dos valores essenciais ao creeper"
        if self.get_idCreeper()==13:
            if self.areaCor >  50:
                return (self.centro, self.mediaCor, self.areaCor)
            return [0,0],[0,0],0
        else:
            if self.areaCor >  130:
                return (self.centro, self.mediaCor, self.areaCor)
            return [0,0],[0,0],0
    
    def get_idCreeper(self):
        return self.id_creeper

    def get_corCreeper(self):
        return self.cor_creeper
    
    def get_contorno(self):
        if self.maiorcontorno>150:
            return False
        return True

    #setters
    def set_texto(self,texto):
        self.text = texto

    def set_curva(self,direcao):
        "Permite modificação da curva"
        self.curva = direcao

    def set_cor_creeper(self, cor):
        self.cor_creeper = cor
    
    
    def set_id_creeper(self, id):
        self.id_creeper = id

    def set_cor(self,cor):
        "Permite da modificação da cor de segmentação"
        self.cor = cor

    #funções identitárias
    def aruco_detection(self):
        "Detecta arucos"
        self.gray = cv2.cvtColor(self.aruco_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, _ = aruco.detectMarkers(self.gray, self.aruco_dict)

    def aruco_markers(self):
        "Faz marca dos arucos"
        self.aruco_detection()
        try:
            self.ret = aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.camera_distortion)
            self.rvec, self.tvec = self.ret[0][0,0,:], self.ret[1][0,0,:]
            aruco.drawDetectedMarkers(self.cv_image, self.corners, self.ids) 
            aruco.drawAxis(self.cv_image, self.camera_matrix, self.camera_distortion, self.rvec, self.tvec, 1)
            self.dist_aruco = np.sqrt(self.tvec[0]**2 + self.tvec[1]**2 + self.tvec[2]**2)
        except:
            pass

    def faixa_imagem(self):
        "Recorta a faixa a altura de visão do robô para seguir linha, além de cortes para percorre-la"
        self.h, self.w = self.cv_image.shape[:2]
        esquerda = self.mask.copy()
        direita = self.mask.copy()

        if self.aruco_distance()<35 and self.get_ids()==200 and self.curva =="esquerda":
            #print("Virando a esquerda!")
            esquerda[:,400:]=0
            self.vision = esquerda
        elif self.aruco_distance()<35 and self.get_ids()==200 and self.curva == "direita":
            #print("Virando a Direita!")
            direita[:,:250]=0
            self.vision = direita
        else:
            #print("Seguindo Linha!")    
            self.vision =  self.mask
       
    def centro_de_massa(self):
        "Define centro de massa da figura"
        self.aruco_markers()
        self.faixa_imagem()
        self.M = cv2.moments(self.vision)
        if self.M['m00'] > 0:
            self.cx = int(self.M['m10']/self.M['m00'])
            self.cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.cv_image, (self.cx, self.cy), 20, (255,0,0), -1)

    def cross(self,img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 

    def segmenta_linha(self,frame):
        '''
        Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
        '''
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cor_menor = (int(44//2), 180, 180)
        cor_maior = (int(64//2), 255, 255)
        mescla = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        centro = (frame.shape[1]//2, frame.shape[0]//2)
        segmentado_cor = cv2.morphologyEx(mescla,cv2.MORPH_CLOSE,np.ones((7, 7)))	

        self.h, self.w = frame.shape[:2]
        search_top = 3*self.h//4 - 50
        search_bot = 3*self.h//4 + 20
        segmentado_cor[0:search_top, 0:self.w] = 0
        segmentado_cor[search_bot:self.h, 0:self.w] = 0

        contornos, _ = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

        maior_contorno = None
        maior_contorno_area = 0

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > maior_contorno_area:
                maior_contorno = cnt
                maior_contorno_area = area

        # Encontramos o centro do contorno fazendo a média de todos seus pontos.
        if not maior_contorno is None :
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            self.cross(frame, centro, [255,0,0], 1, 17)
        else:
            media = (0, 0)
        
        return segmentado_cor,maior_contorno_area
        

    def segmenta_creeper(self,frame, cor): 
        '''
        Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
        '''
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        if cor == "orange":
            cor_menor = np.array([0, 200, 200])
            cor_maior = np.array([8, 255, 255])
            mescla = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        elif cor == "blue":
            cor_menor = np.array([75, 50, 50])
            cor_maior = np.array([95, 255, 255])
            mescla = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        elif cor == "green":
            cor_menor = np.array([45, 100, 100])
            cor_maior = np.array([75, 255, 255])
            mescla = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        centro = (frame.shape[1]//2, frame.shape[0]//2)
        segmentado_cor = cv2.morphologyEx(mescla,cv2.MORPH_CLOSE,np.ones((7, 7)))	
   
        if self.creeper_values()[0][0]!=0 and self.get_ids()== self.get_idCreeper():
            self.h, self.w = frame.shape[:2]
            corte = int(self.get_corners())
            segmentado_cor[:,0:corte-120]=0
            segmentado_cor[:,corte+120:self.w]=0

        contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        maior_contorno = None
        maior_contorno_area = 0

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > maior_contorno_area:# and area > 150 and area<700:
                maior_contorno = cnt
                maior_contorno_area = area

        # Encontramos o centro do contorno fazendo a média de todos seus pontos.
        if not maior_contorno is None :
            cv2.drawContours(frame, [maior_contorno], -1, [255, 0, 0], 5)
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            self.cross(frame, centro, [255,0,0], 1, 17)
        else:
            media = (0, 0)
        return media, centro, maior_contorno_area,segmentado_cor

    # Inicilamente implementada apenas para filtro de cor:
    def roda_todo_frame(self, imagem):        
        "Função assincrona de rodagem da imagem"
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            creeper = self.cv_image.copy()
            self.aruco_image = self.cv_image.copy()
            self.mask,self.maiorcontorno = self.segmenta_linha(self.cv_image)
            self.centro, self.mediaCor, self.areaCor,segmentado = self.segmenta_creeper(creeper,self.cor_creeper)
            self.centro_de_massa()
            cv2.putText(self.cv_image,self.text, (50,50), cv2.QT_FONT_NORMAL, 0.8, (0,255,0))
            cv2.imshow("Camera", self.cv_image)
            #cv2.imshow("segmentado", segmentado)
            #cv2.imshow("Creeper", creeper)
            #cv2.imshow("Mascara", self.vision)
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def Subscriber(self):
        return self.subscriber 


if __name__=="__main__":
    print('Este script não deve ser usado diretamente')