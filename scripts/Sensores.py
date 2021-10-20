# Imports:
from __future__ import print_function, division
import rospy
import numpy as np
import numpy
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
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

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

    def __init__(self, pula):
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.recebe_odometria)
        self.x = 0
        self.y = 0
        self.z = 0
        self.contador = 0
        self.pula = pula

    def recebe_odometria(self, data):
       
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista))    

        if self.contador % self.pula == 0:
            print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(self.x, self.y,angulos[2]))
        self.contador += 1

    def Subscriber(self):
        return self.subscriber

#  Classe do sensor Câmera, com algumas de suas funções principais:
class Camera:

    def __init__(self):

        self.topico_imagem = "/camera/image/compressed"
        self.subscriber = rospy.Subscriber(self.topico_imagem , CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        self.cor = ""
        self.mask = None
        self.bridge = CvBridge()   # Para compressed image
        self.cv_image = None
        self.centro = []           # Informações filtro de cor.
        self.mediaCor = []
        self.areaCor = 0.0
        self.M = None
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1

    def centro_de_massa(self):
        "Define centro de massa da figura"
        self.M = cv2.moments(self.mask)
        if self.M['m00'] > 0:
            self.cx = int(self.M['m10']/self.M['m00'])
            self.cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.cv_image, (self.cx, self.cy), 20, (0,255,255), -1)

    def faixa_imagem(self):
        "Recorta a faixa a altura de visão do robô"
        self.h, self.w = self.cv_image.shape[:2]
        search_top = 3*self.h//4 - 50
        search_bot = 3*self.h//4 + 20
        self.mask[0:search_top, 0:self.w] = 0
        self.mask[search_bot:self.h, 0:self.w] = 0

    def get_valores(self):
        "Getter de valores para aproveitamento em ações executadas"
        return self.cx, self.cy, self.h, self.w

    def set_cor(self,cor):
        self.cor = cor

    # Inicilamente implementada apenas para filtro de cor:
    def roda_todo_frame(self, imagem):                                          # NÃO SEI SE É VIÁVEL IMPLEMENTAR
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            self.mediaCor, self.centro, self.areaCor, self.mask = identifica_cor(self.cv_image,self.cor)
            self.centro_de_massa()
            self.faixa_imagem()
            cv2.imshow("Camera", self.cv_image)
            cv2.imshow("Mascara", self.mask)
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def Subscriber(self):
        return self.subscriber 

# A função a seguir é chamada sempre que chega um novo frame
if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
