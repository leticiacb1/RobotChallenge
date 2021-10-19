#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import math
import numpy as np
import os
import rospy
import tf
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
from sklearn import linear_model
from sklearn.linear_model import LinearRegression


def segmenta_linha_amarela(bgr):
    """
        deve receber uma imagem bgr e retornar uma máscara com os segmentos amarelos do centro da pista em branco.
        Utiliza a função cv2.morphologyEx() para limpar ruidos na imagem
    """
    img2 = bgr.copy()
    img_hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
    # Segmenta apenas a cor amarela
    menor = (int(45/2), 50, 50)
    maior = (int(66/2), 255, 255)
    mask_amarelo = cv2.inRange(img_hsv, menor, maior)
    elemento_estrut = np.ones([3,3])
    # realiza a abertura
    mask_amarelo_abertura = cv2.morphologyEx(mask_amarelo, cv2.MORPH_OPEN, elemento_estrut)
    
    
    return mask_amarelo_abertura

def encontrar_contornos(mask):
    """
        deve receber uma imagem preta e branca e retornar todos os contornos encontrados
    """
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    
    return contornos

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    x = int(x)
    y = int(y)
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def encontrar_centro_dos_contornos(bgr, contornos):
    """
        deve receber uma lista de contornos e retornar, respectivamente,
        a imagem com uma cruz no centro de cada segmento e o centro de cada. 
        formato: img, x_list, y_list
    """

    img = bgr.copy()
    x_list = []
    y_list = []
    # loop over the contours
    for c in contornos:
        # compute the center of the contour
        M = cv2.moments(c)
        x_list.append(int(c[:,:,0].mean()))
        y_list.append(int(c[:,:,1].mean()))
        Xcentro = int(c[:,:,0].mean())
        Ycentro = int(c[:,:,1].mean())
        # draw the contour and center of the shape on the image
        cv2.drawContours(img, [c], -1, (0, 0, 255), 2)
        crosshair(img, (Xcentro, Ycentro), 5, (0,0,255))
    return img, x_list, y_list


def desenhar_linha_entre_pontos(bgr, X, Y, color):
    """
        deve receber uma lista de coordenadas XY, e retornar uma imagem com uma linha entre os centros EM SEQUENCIA do mais proximo.
    """
    img = bgr.copy()
    for i in range(1,len(X)):
        x1, x2, y1, y2 = X[i-1], X[i], Y[i-1], Y[i]
        cv2.line(img, (x1,y1), (x2,y2), color, 2)

    return img

def regressao_por_centro(bgr, x_array, y_array):
    """
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta e os parametros da reta

    """
    img = bgr.copy()
 
    reg = linear_model.LinearRegression()

    yr = y_array.reshape(-1,1) # Entradas do modelo
    xr = x_array.reshape(-1,) # saídas do modelo

    reg.fit(yr,xr)
    
    ransac = linear_model.RANSACRegressor(reg)
    ransac.fit(yr, xr)
    reg = ransac.estimator_
    coef_angular, coef_linear = reg.coef_, reg.intercept_
    x = coef_angular*y_array + coef_linear
    y_min = int(min(y_array)-250)
    y_max = int(max(y_array)+250)
    x_min = int(coef_angular*y_min + coef_linear)
    x_max = int(coef_angular*y_max + coef_linear) 
    cv2.line(img, (x_min, y_min), (x_max, y_max), (0,255,0), thickness=3);
    
    return img, reg

def calcular_angulo_com_vertical(img, lm):
    """
        deve receber uma imagem contendo uma reta, além da reggressão linear e determinar o ângulo da reta com a vertical, utilizando o metodo preferir.
    """
    angulo = math.degrees(math.atan(lm.coef_))
     # font
    font = cv2.FONT_HERSHEY_SIMPLEX
    # org
    org = (250, 50)
    # fontScale
    fontScale = 1
    # Blue color in BGR
    color = (255, 0, 0)
    # Line thickness of 2 px
    thickness = 2
    text = str(angulo)
    text = "Angle: " + text
    # Using cv2.putText() method
    img = cv2.putText(img, text, org, font, 
                       fontScale, color, thickness, cv2.LINE_AA)
    return angulo

def segmenta_linha_branca(bgr):
    """
        deve receber uma imagem e segmentar as faixas brancas
    """
     # Faz a conversão para o espaço HSV
    img_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Segmenta apenas a cor branca (qualquer valor de H, saturação baixa e altos intensidades luminosas)
    menor = (int(0), 0, 240)
    maior = (int(180), 50, 255)
    bgr = cv2.inRange(img_hsv, menor, maior)
    elemento_estrut = np.ones([3,3])
    bgr = cv2.morphologyEx(bgr, cv2.MORPH_CLOSE, elemento_estrut)

    
    return bgr

def estimar_linha_nas_faixas(img, mask):
    """
        deve receber uma imagem preta e branca e retorna dois pontos que formen APENAS uma linha em cada faixa. Desenhe cada uma dessas linhas na iamgem.
         formato: [[(x1,y1),(x2,y2)], [(x1,y1),(x2,y2)]]
    """
    lines = cv2.HoughLinesP(mask, 25, math.pi/180.0, threshold=500, minLineLength=100, maxLineGap=30)
    linhas1 = []
    linhas2 = []

    for i in range(2):
        # Faz uma linha ligando o ponto inicial ao ponto final, com a cor verde (BGR)
        cv2.line(img, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 255, 0), 5, cv2.LINE_AA)
        linhas1.append((lines[i][0][0], lines[i][0][1]))
        linhas2.append((lines[i][0][2], lines[i][0][3]))
    linhas = [(linhas1[0],linhas2[0]),(linhas1[1],linhas2[1])]
    return linhas

def calcular_equacao_das_retas(linhas):
    """
        deve receber dois pontos que estejam em cada uma das faixas e retornar a equacao das duas retas. Onde y = h + m * x. Formato: [(m1,h1), (m2,h2)]
    """
    x1_esquerda,y1_esquerda = linhas[0][0]
    x2_esquerda, y2_esquerda = linhas[0][1]
    m1 = (y2_esquerda-y1_esquerda)/(x2_esquerda-x1_esquerda)
    h1 = y1_esquerda - m1*x1_esquerda
    
    x1_direita,y1_direita = linhas[1][0]
    x2_direita, y2_direita = linhas[1][1]
    m2 = (y2_direita-y1_direita)/(x2_direita-x1_direita)
    h2 = y1_direita - m2*x1_direita
    return [(m1,h1), (m2,h2)]

def calcular_ponto_de_fuga(img, equacoes):
    """
        deve receber duas equacoes de retas e retornar o ponto de encontro entre elas. Desenhe esse ponto na imagem.
    """
    try:
        m1,h1 = equacoes[0]
        m2,h2  = equacoes[1]
        x = (h2-h1)/(m1-m2)
        y = m1*x + h1
        cv2.circle(img, (int(x), int(y)), radius=5, color=(0,255,0), thickness=-1) 
    except:
        x, y = 0,0
    return img, (x, y)

def load_mobilenet():
    """
        Carrega o modelo e os parametros da MobileNet. 
        Retorna a rede carregada.
    """
    proto = "./mobilenet_detection/MobileNetSSD_deploy.prototxt.txt" # descreve a arquitetura da rede
    model = "./mobilenet_detection/MobileNetSSD_deploy.caffemodel" # contém os pesos da rede em si
    net = cv2.dnn.readNetFromCaffe(proto, model)
    return net


def detect(net, frame, CONFIDENCE, COLORS, CLASSES):
    """
        Recebe:
            net - a rede carregada
            frame - uma imagem colorida BGR
            CONFIDENCE - o grau de confiabilidade mínima da detecção
            COLORS - as cores atribídas a cada classe
            CLASSES - o array de classes
        Devolve: 
            img - a imagem com os objetos encontrados
            resultados - os resultados da detecção
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence


        if confidence > CONFIDENCE:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))


    return image, results

def identifica_cor(frame, cor): #agora recebe o frame e uma string com a cor(ex.: "red")
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
    # vermelho puro (H=0) estão entre H=-8 e H=8. 
    # Precisamos dividir o inRange em duas partes para fazer a detecção 
    # do vermelho:
    # frame = cv2.flip(frame, -1) # flip 0: eixo x, 1: eixo y, -1: 2 eixos
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #selecionar a cor a ser segmentada
    if cor == "red":
        cor_menor = np.array([0, 50, 100])
        cor_maior = np.array([6, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

        cor_menor = np.array([174, 50, 100])
        cor_maior = np.array([180, 255, 255])

        # NOTA: so' precisamos de 2 ranges porque o vermelho da' 
        # a volta do 360 para o zero.
        # Para qualquer outra cor apenas um inRange resolve
        segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)
    elif cor == "blue":
        cor_menor = np.array([220/2, 50, 100])
        cor_maior = np.array([260/2, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    elif cor == "amarelo":
        cor_menor = (int(45/2), 50, 50)
        cor_maior = (int(66/2), 255, 255)
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    #podemos adiconar mais a medida que for surgindo necessidade
        

    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos inverter, porque 
    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 



    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
    # que um quadrado 7x7. É muito útil para juntar vários 
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    # Encontramos os contornos na máscara e selecionamos o de maior área
    #contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
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


if __name__ == "__main__":
    print('Este script não deve ser usado diretamente')

