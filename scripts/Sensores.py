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

    def __init__(self,cor1 = None, cor2 = None):

        self.topico_imagem = "/camera/image/compressed"
        self.subscriber = rospy.Subscriber(self.topico_imagem , CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        
        self.corLow = cor1         # Parâmetros para o filtro
        self.corHight = cor2

        self.bridge = CvBridge()   # Para compressed image

        self.centro = []           # Informações filtro de cor.
        self.mediaCor = []
        self.areaCor = 0.0

    # Inicilamente implementada apenas para filtro de cor:
    def roda_todo_frame(self):                                          # NÃO SEI SE É VIÁVEL IMPLEMENTAR
        print("Frame filtro")
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            cv2.imshow("Camera", cv_image)                               #Acho que já é chamado em corModule
            
            self.mediaCor, self.centro, self.areaCor = cormodule.identifica_cor(cv_image, self.corLow, self.corHight)  # VER COMO O LORRAN CHAMOU EM AUXILIAR

            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def Subscriber(self):
        return self.subscriber 

# A função a seguir é chamada sempre que chega um novo frame
if __name__=="__main__":
    print('Este script não deve ser usado diretamente')
