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

def identifica_cor(frame, cor, segue_linha = False):    #Recebe o frame e uma string com a cor(ex.: "red") e se a mascará é para a linha ou para creeper
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''
    # frame = cv2.flip(frame, -1) # flip 0: eixo x, 1: eixo y, -1: 2 eixos
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #selecionar a cor a ser segmentada
    if cor == "orange":                     # Mascara creeper vermelho
        cor_menor = np.array([0, 50, 100])
        cor_maior = np.array([6, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

        cor_menor = np.array([174, 50, 100])
        cor_maior = np.array([180, 255, 255])

        segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)
    
    elif cor == "blue":                      # Marcara creeper azul
        cor_menor = np.array([220/2, 50, 100])
        cor_maior = np.array([260/2, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    
    elif cor == "green":                     # Mascara creeper verde 
        cor_menor = np.array([100/2, 50, 100])
        cor_maior = np.array([140/2, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    
    elif cor == "amarelo":                    # Mascara linha
        cor_menor = (int(45/2), 50, 50)
        cor_maior = (int(66/2), 255, 255)
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

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
        
        if not segue_linha:
        
            cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        #cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)

    else:
        media = (0, 0)

    if segue_linha:
        return segmentado_cor                         #Mascara da linhas
    else:
        return media, centro, maior_contorno_area     #Utilizado apenas para centralizar em creeper


if __name__ == "__main__":
    print('Este script não deve ser usado diretamente')

