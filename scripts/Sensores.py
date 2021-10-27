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
from std_msgs.msg import Header

from auxiliar import *

# Classes de sensores que serão chamadas pelo robô e pela máquina de estações.

# Classe do sensor laser, com algumas de suas funções principais:
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
    
    def get_dados(self):
    	return self.dados

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

    def recebe_odometria(self, data):
        "Função assincrona para registrar odometria"
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista))    

        if self.contador % self.pula == 0:
            pass
            #print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(self.x, self.y,angulos[2]))
        self.contador += 1

    def distancia_centro(self):
        "Retorna distancias relativas ao ponto de partida"
        return np.sqrt(self.x**2 + self.y**2)

    def positions(self):
        "Retorna posições via odometria"
        return (self.x,self.y)

    def Subscriber(self):
        return self.subscriber

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
        self.calib_path  = "aruco_assets/"
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
        self.estado_na_pista = 0           # Indica em qual estado da pista o robô está. Já virou a direita? Já completou a volta? (O valor dessa variável é atualizada pelo módulo actions)

        # Centraliza creeper:
        self.centro = []                    # Informações filtro de cor.
        self.mediaCor = []
        self.areaCor = 0.0

        self.cor_creeper = None    
        self.id_creeper = None

    #getters
    def get_corners(self):
        '''Retorna o primeiro valor de x do id identificado'''
        m  = 0
        if self.get_idCreeper() in self.get_ids(): 
            try:
                for i in self.corners[0]:
                    m += i[0][0]
                return m
            except:
                pass
        else:
            return 0

    def get_ids(self):
        "Retorno do primeiro id identificado"
        try:
            return self.ids[0]
        except:
            return None

    def aruco_distance(self):
        "Retorno da distancia da aruco ao robo"
        return self.dist_aruco

    def get_estado_na_pista(self):
        "Retorno do estado de pista"
        return self.estado_na_pista

    def get_valores(self):
        "Getter de valores para aproveitamento em ações executadas"
        return self.cx, self.cy, self.h, self.w

    def get_idCreeper(self):
        return self.id_creeper
    
    def get_creeperValues(self):
        '''Valores utilizados na centralização do creeper'''
        return (self.centro, self.mediaCor, self.areaCor)

    #setters
    def set_estado_na_pista(self,estado):
        "Permite modificação do estado"
        self.estado_na_pista = estado

    def set_cor(self,cor):
        "Permite da modificação da cor de segmentação"
        self.cor = cor

    def set_cor_crepper(self,cor):
        self.cor_creeper = cor

    def set_id_creeper(self,id):
        self.id_creeper = id

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
    
    def curva(self):
        '''Retorna True caso identificado id de curva'''
        return (self.get_ids()[0]==200)
    
    def creeper_correto():
        '''Retorna true caso o id do creeper recebido pela camera seja o id buscado'''
        return (self.get_ids()[0]== self.get_idCreeper())

    def faixa_imagem(self):
        "Recorta a faixa a altura de visão do robô para seguir linha, além de cortes para percorre-la"
        self.h, self.w = self.cv_image.shape[:2]
        
        search_top = 3*self.h//4 - 50
        search_bot = 3*self.h//4 + 20

        self.mask[0:search_top, 0:self.w] = 0           # Recorte na mascara utilizada para seguir linha
        self.mask[search_bot:self.h, 0:self.w] = 0
        
        esquerda = self.mask.copy()
        direita = self.mask.copy()
        
        if self.aruco_distance()<35 and curva and self.estado_na_pista == 0:
            print("Virando a esquerda!")
            esquerda[:,400:]=0
            self.vision = esquerda
        elif self.aruco_distance()<35 and curva and self.estado_na_pista == 1:
            print("Virando a Direita!")
            direita[:,:250]=0
            self.vision = direita
        else:
            print("Seguindo Linha!")    
            self.vision =  self.mask

       
    def centro_de_massa(self):
        "Define centro de massa da figura"
        self.aruco_markers()                         
        self.faixa_imagem()
        self.M = cv2.moments(self.vision)     # Mascara atuante no "momento"
        if self.M['m00'] > 0:
            self.cx = int(self.M['m10']/self.M['m00'])
            self.cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.cv_image, (self.cx, self.cy), 20, (255,0,0), -1)

    # Inicilamente implementada apenas para filtro de cor:
    def roda_todo_frame(self, imagem):        
        "Função assincrona de rodagem da imagem"
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            self.aruco_image = self.cv_image.copy()
            #creeper = self.cv_image.copy()
            self.mask = identifica_cor(self.cv_image,self.cor, True)
            self.mediaCor, self.centro, self.areaCor, self.mask = identifica_cor(self.cv_image,self.cor_creeper)
            
            self.centro_de_massa()

            cv2.imshow("Camera", self.cv_image)
            cv2.imshow("Mascara", self.vision)
            #cv2.imshow("Creeper", creeper)
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def Subscriber(self):
        return self.subscriber 


if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
