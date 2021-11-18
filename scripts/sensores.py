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

from sklearn.linear_model import LinearRegression
from sklearn import linear_model

__authors__ = ["Leticia Côelho","Lorran Caetano","Matheus Oliveira","Ykaro de Andrade"]

#================================CLASSES SENSORIAIS=================================#

# Classe do sensor laser, com algumas de suas funções principais:
class Laser:
    "Classe sensorial do  LAser, guia o Robô de acordo com distâncias a objetos do cenário"
    def __init__(self):
        #=====Atributos da callback==============#
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.distancia = None
        self.dados = None

    #================================Callback=================================#
    def scaneou(self, dado):
        "Função assincrona que registra dados de distância"
        self.dados = np.array(dado.ranges).round(decimals=2)

    #================================Funções de exportações=================================#
    def get_dados(self):
        "Exporta a distância relativa a frente do robô"
        if self.dados is not None:
    	    return self.dados[0]

    def Subscriber(self):
        return self.subscriber


#Classe do sensor Odometria, com algumas de suas funções principais:
class Odom:
    "Classe sensorial da Odometria, guia o Robô de acordo com posições e angulações para boa execução do objetivo"
    def __init__(self):
        #=====Atributos da callback==============#
        self.subscriber = rospy.Subscriber("/odom", Odometry, self.recebe_odometria)
        self.x = 0
        self.y = 0
        self.z = 0
        self.angulo = 0

    #================================Callback=================================#
    def recebe_odometria(self, data):
        "Função assincrona para registrar odometria, e guardar dados essencias para uso posterior"
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista))    
        self.angulo = angulos[2]
        #Conserta angulação
        if self.angulo > 360:
           self.angulo  -= 360
        elif self.angulo  < 0:
            self.angulo  += 360

    #================================Funções de exportações=================================#
    def distancia_centro(self):
        "Retorna distancias relativas ao ponto de partida"
        return np.sqrt(self.x**2 + self.y**2)

    def positions(self):
        "Retorna posições via odometria"
        return (self.x,self.y)

    def get_angulo(self):
        "Retorna angulação do robô"
        return self.angulo

    def Subscriber(self):
        return self.subscriber

 #Classe de obteção do node da estacao       
class Redeneural:
    "Classe sensorial da node Rede neural,que presta serviço a este, essencial para identificação e aproximação de estações"
    def __init__(self):
        #=====Atributos da callback==============#
        self.recebedor = rospy.Subscriber("/estacao", String, self.recebe_estacao)
        self.neural_position = rospy.Subscriber("/corner", Float64, self.recebe_posicao)
        self.estacao = None
        self.posicao = 0 
    #================================Callback=================================#
    def recebe_estacao(self,msg):
        "Recebe as informações oferecidas pelos nodes que prestam serviço e as estabelecem como atributo, em específico o label da estação"
        try:
            if msg.data == "boat":
                self.estacao = "horse"
            else:
                self.estacao = msg.data
        except:
            pass

    def recebe_posicao(self,msg):
        "Recebe as informações oferecidas pelos nodes que prestam serviço e as estabelecem como atributo, em específico a posição X da estação"
        try:
            self.posicao = msg.data
        except:
            pass

    #================================Getters=================================#
    def get_estacao(self):
        "Exporta estação identificada"
        return self.estacao
    
    def get_posicao(self):
        "Exporta posição identificada"
        return self.posicao

# Classe do sensor Câmera, com algumas de suas funções principais:
class Camera:
    "Classe sensorial da câmera, essencial para obter valores e sinais de segmentação de cores, tal qual guiar o percurso"
    def __init__(self):
        "Construtor da classe câmera"
        #=====Atributos da callback==============#
        self.topico_imagem = "/camera/image/compressed"
        self.subscriber = rospy.Subscriber(self.topico_imagem , CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        self.bridge = CvBridge()   # Para compressed image
        self.cv_image = None
        #=====Atributos para seguir linha==============#
        self.mask = None
        self.M = None
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        #=====Atributos para identificar aruco==============#
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
        #=====Atributos para identificar creeper==============#
        self.centro = []           # Informações filtro de cor.
        self.mediaCor = []
        self.areaCor = 0.0
        self.cor_creeper = None
        self.id_creeper = None
        #=====Atributos para realizar o retorno a pista==============#
        self.mascara_recorte = True
        self.maiorcontorno = None
        #=====Atributo Regressao Linear para controle de velocidade ==============#
        self.angulo_vertical = None
        #=====Atributos para printar na tela==============#
        self.text = None

    #================================getters=================================#
    def get_ids(self):
        "Retorno das ids aruco identificadas"
        try:
            return self.ids[0][0]
        except:
            return None

    def get_corners(self):
        "Retorna coordenadas x do aruco identificado do creeper para ajuste de centralização"
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
        "Getter dos valores essenciais ao creeper, como posição de cor, tela, e área medida"
        if self.get_idCreeper()==13:
            if self.areaCor >  50:
                return (self.centro, self.mediaCor, self.areaCor)
            return [0,0],[0,0],0
        else:
            if self.areaCor >  130:
                return (self.centro, self.mediaCor, self.areaCor)
            return [0,0],[0,0],0
    
    def get_idCreeper(self):
        "Retorna id do creeper setado  pelo usuário"
        return self.id_creeper

    def get_corCreeper(self):
        "Retorna cor do creeper setado pelo usuário"
        return self.cor_creeper
    
    def get_contorno(self):
        "Retorna o tamanho da área de contorno da pista para fins de volta a ela"
        if not self.mascara_recorte:
            return self.maiorcontorno
        return 0

    #================================setters=================================#
    def set_mascara_recorte(self,boolean):
        "Condição que faz o recorte da mascára da amarela da pista, permite identificação ou não de pedaços para sua volta"
        self.mascara_recorte = boolean

    def set_texto(self,texto):
        "Setta texto a ser printado na imagem do robô"
        self.text = texto

    def set_curva(self,direcao):
        "Permite modificação da curva, isto é sua direção no circuito"
        self.curva = direcao

    def set_cor_creeper(self, cor):
        "Permite modificar a cor do creeper buscado"
        self.cor_creeper = cor
    
    
    def set_id_creeper(self, id):
        "Permite modificar a id do creeper buscado"
        self.id_creeper = id


    #================================Funções relativas a detecção aruco=================================#
    def aruco_detection(self):
        "Detecta arucos pelo cenário"
        self.gray = cv2.cvtColor(self.aruco_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, _ = aruco.detectMarkers(self.gray, self.aruco_dict)

    def aruco_markers(self):
        "Faz marca dos arucos na tela vista do robô, e guarda valores essenciais para detecção"
        self.aruco_detection()
        try:
            self.ret = aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.camera_distortion)
            self.rvec, self.tvec = self.ret[0][0,0,:], self.ret[1][0,0,:]
            aruco.drawDetectedMarkers(self.cv_image, self.corners, self.ids) 
            aruco.drawAxis(self.cv_image, self.camera_matrix, self.camera_distortion, self.rvec, self.tvec, 1)
            self.dist_aruco = np.sqrt(self.tvec[0]**2 + self.tvec[1]**2 + self.tvec[2]**2)
        except:
            pass
    
    #================================Funções relativas a regressão linear =================================#
    def encontrar_centro_dos_segmento_de_linha(self, bgr, contornos):
        """
           Marca centros dos contornos na imagem recebida e retornaa todos seus pontos (X,Y) desses centros em listas.
        """

        img = bgr.copy()
        x_list = []
        y_list = []

        color = (0,0,255)

        for contorno in contornos:
            area = cv2.contourArea(contorno)
            
            if area>300:
            
                cX = int(contorno[:,:,0].mean())
                cY = int(contorno[:,:,1].mean())

                x_list.append(cX)
                y_list.append(cY)

        for x,y in zip(x_list,y_list):

            self.cross(img, (x,y), color , 2, 4)

        return img, x_list, y_list


    def regressao_linear(self, bgr, x_array, y_array):
        """
            bgr : imagem bgr

            x_list : lista de pontos contendo o Xcentro do contorno.
            y_list : lista de pontos contendo o Ycentro do contorno.
            
            Retorna a imagem com a melhor regressão encontrada.
            
        """
        img = bgr.copy()

        # Como é uma linha próxima da vertical, é mais produtivo escrever x por y:
        # Treinando modelo

        reg = LinearRegression()
        yr = y_array.reshape(-1,1)  # Entradas do modelo
        xr = x_array.reshape(-1,)   # saídas do modelo
        reg.fit(yr,xr)

        try:
            # RANSAC:
            ransac = linear_model.RANSACRegressor(reg)
            ransac.fit(yr, xr)
            reg = ransac.estimator_
            a, b = reg.coef_, reg.intercept_

            #Regressão:
            x = a*y_array + b

            #Buscando valores mínimos e máximos da lista obtida para pegarmos ponto inicial e final:
            y_min = int(min(y_array)) - 100
            y_max = int(max(y_array)) + 100

            x_min = int(a*y_min + b)
            x_max = int(a*y_max + b)  

            #Desenhando linha:  
            cv2.line(img, (x_min, y_min), (x_max, y_max), (255,0,0), thickness=3);  

            return img, reg
        except:
            return img, reg

    def calcular_angulo_com_vertical(self, lm):
        """
        lm : regressão linear.

        Função retorna o angulo com a vertical.

        """

        a = lm.coef_

        rad = math.atan(a)     # Em radianos
        teta = rad*180/math.pi # Em graus
        
        return teta
        
    #================================Funções relativas a segmentação de cores do cenário=================================#
    def faixa_imagem(self):
        "Recorta a faixa a altura de visão do robô para seguir linha, além de cortes para percorre-la, isto é força uma visão para robô seguir por uma direção na curva."
        self.h, self.w = self.cv_image.shape[:2]
        esquerda = self.mask.copy()
        direita = self.mask.copy()

        if self.aruco_distance()<35 and self.get_ids()==200 and self.curva =="esquerda":
            esquerda[:,400:]=0
            self.vision = esquerda
        elif self.aruco_distance()<35 and self.get_ids()==200 and self.curva == "direita":
            direita[:,:250]=0
            self.vision = direita
        else:
  
            self.vision =  self.mask
       
    def centro_de_massa(self):
        "Define centro de massa da figura segmentada"
        self.aruco_markers()
        self.faixa_imagem()
        self.M = cv2.moments(self.vision)
        if self.M['m00'] > 0:
            self.cx = int(self.M['m10']/self.M['m00'])
            self.cy = int(self.M['m01']/self.M['m00'])
            cv2.circle(self.cv_image, (self.cx, self.cy), 20, (255,0,0), -1)

    def cross(self,img_rgb, point, color, width,length):
        "Coloca uma cruz no centro da figura"
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 

    def segmenta_linha(self,frame):
        '''
        Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV). Neste caso em específico segmenta a cor amarelo da pista.
        '''
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cor_menor = (int(44//2), 180, 180)
        cor_maior = (int(64//2), 255, 255)
        mescla = cv2.inRange(frame_hsv, cor_menor, cor_maior)
        centro = (frame.shape[1]//2, frame.shape[0]//2)
        segmentado_cor = cv2.morphologyEx(mescla,cv2.MORPH_CLOSE,np.ones((7, 7)))	

        img_regressao = frame.copy()

        self.h, self.w = frame.shape[:2]
        if self.mascara_recorte:
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
        
        # Buscando centro dos contornos:
        img_regressao, x_list, y_list = self.encontrar_centro_dos_segmento_de_linha(img_regressao, contornos)
        self.textAngulo = ''

        if x_list and len(x_list)<5:
            x_array = np.array(x_list)
            y_array = np.array(y_list)
            # Regressao_linear
            img_regressao, regressao = self.regressao_linear(img_regressao, x_array , y_array)

            # Encontrando algulo com a vertical e guardando em uma variável:
            self.angulo_vertical = abs(self.calcular_angulo_com_vertical(regressao))
            self.textAngulo = f"Angulo: {self.angulo_vertical}"

        # Encontramos o centro do contorno fazendo a média de todos seus pontos.
        if not maior_contorno is None :
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            self.cross(frame, centro, [255,0,0], 1, 17)
        else:
            media = (0, 0)
        
        return segmentado_cor, img_regressao, maior_contorno_area
        

    def segmenta_creeper(self,frame, cor): 
        '''
        Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV). Neste caso em específico segmenta a cor escolhida do creeper.
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

    #================================Callback=================================#
    def roda_todo_frame(self, imagem):        
        "Função assincrona de rodagem da imagem, captando os valores de interesse"
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            creeper = self.cv_image.copy()
            self.aruco_image = self.cv_image.copy()
            self.mask,regressao,self.maiorcontorno = self.segmenta_linha(self.cv_image)
            self.centro, self.mediaCor, self.areaCor,segmentado = self.segmenta_creeper(creeper,self.cor_creeper)
            self.centro_de_massa()
            cv2.putText(self.cv_image,self.text, (50,50), cv2.QT_FONT_NORMAL, 0.8, (0,255,0))
            cv2.imshow("Camera", self.cv_image)
            cv2.putText(regressao,self.textAngulo, (50,50), cv2.QT_FONT_NORMAL, 0.8, (255,0,0))
            cv2.imshow("Regressao", regressao)
            #Caso deseje ver alguma mascara de segmentação, descomentar linhas abaixo
            #cv2.imshow("segmentado", segmentado)
            #cv2.imshow("Mascara", self.vision)
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def Subscriber(self):
        "Retorna o subscriber do node"
        return self.subscriber 


if __name__=="__main__":
    print('Este script não deve ser usado diretamente')