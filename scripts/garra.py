import rospy
from std_msgs.msg import Float64
# rode: roslaunch mybot_description mybot_control2.launch 
__authors__ = ["Leticia Côelho","Lorran Caetano","Matheus Oliveira","Ykaro de Andrade"]

#================================CLASSES DA GARRA=================================#

class Garra:
    "Classe que realiza o controle e publish das operações da garra"
    def __init__(self):
        "Construtor da função"
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
        self.ombro.publish(-1.0)  ## para baixo 
        self.garra.publish(0.0)  ## Fechado

    def inicio_garra(self):
        "Definição inicial da garra, para baixo e fechada"
        self.ombro.publish(-1.0)  ## para baixo 
        self.garra.publish(0.0)  ## Fechado

    def posiciona_garra(self):
        "Inicia a garra aberta e na altura do  pescoço do creeper"
        self.ombro.publish(-0.32) ## para cima, só um pouco (no 'pescoço')
        self.garra.publish(-1.0)  ## Aberta

    def fecha_garra(self):
        "Fecha a farra para agarra o creeper"
        self.garra.publish(0.0) ## Fechada
    
    def levanta_ombro(self):
        "Levanta a garra para tirar creeper da visão"
        self.ombro.publish(2)
    
    def abaixa_ombro(self):
        "Abaixo o ombro para posteriormente soltar creeper"
        self.ombro.publish(-1.0)
    
    def abre_garra(self):
        "Abre garra para soltar creeper"
        self.garra.publish(-1.0)  ## Aberta
