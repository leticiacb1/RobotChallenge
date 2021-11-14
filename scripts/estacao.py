#! /usr/bin/env python3
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String,Float64
from cv_bridge import CvBridge, CvBridgeError



class Estacao:
    def __init__(self):
        rospy.init_node("estacao")
        self.topico_imagem = "/camera/image/compressed"
        self.subscriber = rospy.Subscriber(self.topico_imagem , CompressedImage, self.roda_todo_frame, queue_size=4, buff_size = 2**24)
        self.bridge = CvBridge()   # Para compressed image
        self.cv_image = None
        self.pub = rospy.Publisher("/estacao", String, queue_size=10)
        self.corner = rospy.Publisher("/corner", Float64, queue_size=1)
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "not",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]
        self.CONFIDENCE = 0.15
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))
        self.proto = "mobilenet_detection/MobileNetSSD_deploy.prototxt.txt" # descreve a arquitetura da rede
        self.model = "mobilenet_detection/MobileNetSSD_deploy.caffemodel" # contém os pesos da rede em si
        self.rede = None
        self.resultados = None
        self.label = None
        self.net = None
        self.alvos = ["cow","horse","boat","dog","car"]
        self.initial_x = None
        self.final_x= None
        self.centro = None
       

    def roda_todo_frame(self, imagem):        
        "Função assincrona de rodagem da imagem"
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            self.rede,self.resultados =  self.detect()
            self.centro = (self.cv_image.shape[1]//2,  self.cv_image.shape[0]//2)
            cv2.imshow("Rede Neural", self.rede)
            #cv2.imshow("Creeper", creeper)
            #cv2.imshow("Mascara", self.vision)
            cv2.waitKey(1)
        
        except CvBridgeError as e:
            print('ex', e)

    def load_mobilenet(self):
        """
        Carrega o modelo e os parametros da MobileNet. 
        """
        self.net = cv2.dnn.readNetFromCaffe(self.proto, self.model)

    def detect(self):
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
        self.load_mobilenet()
        self.rede = self.cv_image.copy()
        (h, w) = self.rede.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(self.rede, (300, 300)), 0.007843, (300, 300), 127.5)

        # pass the blob through the network and obtain the detections and
        # predictions
        print("[INFO] computing object detections...")
        self.net.setInput(blob)
        detections = self.net.forward()
        self.resultados = []
        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with the
            # prediction
            confidence = detections[0, 0, i, 2]
            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > self.CONFIDENCE:
                # extract the index of the class label from the `detections`,
                # then compute the (x, y)-coordinates of the bounding box for
                # the object
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                # display the prediction
                if self.CLASSES[idx] in self.alvos:
                    self.label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                    print("[INFO] {}".format(self.label))
                    cv2.rectangle(self.rede, (startX, startY), (endX, endY),
                        self.COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(self.rede, self.label, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)
                    self.resultados.append((self.CLASSES[idx], confidence*100, (startX, startY),(endX, endY)))
                    self.initial_x = startX
                    self.final_x = endX

        return self.rede, self.resultados

    def publica_estacao(self):
        try:
            while not rospy.is_shutdown():
                if self.resultados is not None:
                    try:
                    # Publicando caso exista qual o rotulo da estacao
                        self.pub.publish(String(self.resultados[0][0]))
                        position = (self.initial_x+ self.final_x)/2 - self.centro[0]
                        self.corner.publish(Float64(position))
                    except:
                        pass
                rospy.sleep(0.1)

        except rospy.ROSInterruptException:
            print("Ocorreu uma exceção com o rospy")


    

if __name__=="__main__":
    estacao = Estacao()
    estacao.publica_estacao()

	

