import rospy
from std_msgs.msg import Float64
# rode: roslaunch mybot_description mybot_control2.launch 

class Garra:

    def __init__(self):
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
        self.ombro.publish(-1.0)  ## para baixo 
        self.garra.publish(0.0)  ## Fechado

    def inicio_garra(self):
        self.ombro.publish(-1.0)  ## para baixo 
        self.garra.publish(0.0)  ## Fechado

    def posiciona_garra(self):
        "Inicia a garra aberta e na altura do  pescoço do creeper"
        self.ombro.publish(-0.32) ## para cima, só um pouco (no 'pescoço')
        self.garra.publish(-1.0)  ## Aberta

    def fecha_garra(self):
        self.garra.publish(0.0) ## Fechada
    
    def levanta_ombro(self):
        self.ombro.publish(2)
    
    def abaixa_ombro(self):
        self.ombro.publish(-1.0)
    
    def abre_garra(self):
        self.garra.publish(-1.0)  ## Aberta


    def largar_objeto(self, momento):
        '''
        Movimentação da garra para largar o objeto
        '''
        now = rospy.get_time()
        if now - momento < 0.5:
            self.ombro.publish(0.0)
        elif now - momento < 1.0:
            self.garra.publish(-1.0)
        elif now - momento < 1.5:
            self.ombro.publish(-1.0)
        elif now - momento < 2.0:
            self.garra.publish(0.0)
        else:
            return True
        return False