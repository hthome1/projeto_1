#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
import math
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import visao_module
import garra


bridge = CvBridge()
cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
viu_linhas=False
viu_linha1=False
viu_linha2=False
achou_obj=False

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"

tfl = 0

tf_buffer = tf2_ros.Buffer()

dist = []
def scaneou(dado):
	d = (np.array(dado.ranges)[0]).round(decimals=2)
	dist.append(d)

def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		#print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
	#	print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global maior_area
    global viu_linhas
    global viu_linha1
    global viu_linha2
    global resultados
    global achou_obj
    global centro_detect
    global x_objeto
    global area_objeto

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
      #  print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        media, centro, maior_area =  visao_module.identifica_cor(cv_image)

        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro_detect, imagem, resultados =  visao_module.processa(cv_image)  
        vc_temp = visao_module.find_circles(cv_image)
        if vc_temp==0:
            viu_linhas = True 
        elif vc_temp==1:
            viu_linha1=True
        elif vc_temp==2:
            viu_linha2=True     
        for r in resultados:
            if r[0]==visao_module.instrucoes["base"]:
                achou_obj=True
                x_objeto = (r[2][0] + r[3][0])/2
                area_objeto=(r[3][0] - r[2][0])*(r[3][1] - r[2][1])

            #print(r)
            # (CLASSES[idx], confidence*100, (startX, startY),(endX, endY)    
            pass

        depois = time.clock()
        
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor_frames = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
    tfl = tf2_ros.TransformListener(tf_buffer)

    tolerancia = 25
    tolX=0.63
    acabou = False
    encontrou=False
    encontrouX=False
    encontrouY=False
    na_pista=True
    pegou_creeper=False
    achou_base=False
    tutorial=garra.MoveGroupPythonIntefaceTutorial()


    try:
        while not rospy.is_shutdown():    
            if na_pista:
                if viu_linhas:
                    vel = Twist(Vector3(0.25,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    viu_linhas = False
                    continue

                elif viu_linha1:
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.12))
                    velocidade_saida.publish(vel)
                    viu_linha1 = False
                    continue

                elif viu_linha2:
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.12))
                    velocidade_saida.publish(vel)
                    viu_linha2 = False
                    continue

            print("Id: ", id)

            if len(media) !=0 and len(centro) != 0 and maior_area>=3000:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                if (not acabou and not pegou_creeper) or (not acabou and not achou_base):
                    if id ==visao_module.instrucoes["id"]:
                                p=len(dist)-1 
                                if p>0:
                                # ajusta a direcao ate encontrar o centro do objeto
                                # quando encontra comeca a ir para frente, mas o ajuste de direcao continua ocorrendo, para evitar possiveis desvios
                                        if (media[0] > (centro[0])):
                                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                                        if (media[0] < (centro[0])):
                                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))

                                        if (abs(media[0] - centro[0])<=10): 
                                            vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.0))
                                            encontrou=True
                                        velocidade_saida.publish(vel)
                                        rospy.sleep(0.1)

                                        if encontrou and not acabou:
                                        #scanner precisa haver detectado algo
                                            if dist[p] <= 1.5: #a partir desta distancia anda mais devagar e para de ajustar a direcao
                                                anda = Twist(Vector3(0.075, 0, 0), Vector3(0, 0, 0))
                                                velocidade_saida.publish(anda)
                                                rospy.sleep(0.1)
                                            if dist[p]<=0.28:#a partir deste ponto, o robo para e o loop nao recomeca
                                                if x>tolX and not encontrouX:
                                                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0.0))
                                                else:
                                                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.0))
                                                    #rospy.sleep(0.5)
                                                    encontrouX=True
                                                if encontrouX:
                                                    velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                                                    velocidade_saida.publish(velocidade)

                                                    raw_input()
                                                    tutorial.open_gripper()
                                                    raw_input()
                                                    tutorial.go_to_init_joint_state()
                                                    raw_input()
                                                    tutorial.go_to_zero_position_goal()
                                                    raw_input()
                                                    tutorial.close_gripper()
                                                    raw_input()
                                                    tutorial.go_to_home_position_goal()
                                                    raw_input()

                                                    velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0,- 0.2))
                                                    velocidade_saida.publish(velocidade)
                                                    rospy.sleep(7)
                                                    acabou=True

                                            if acabou:
                                                na_pista=True
                                                pegou_creeper=True
                                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                                                velocidade_saida.publish(velocidade)
                                                rospy.sleep(2)

            if achou_obj and not achou_base and pegou_creeper:
                    print(area_objeto)
                    na_pista=False

                    if x_objeto < (centro_detect[0] - tolerancia):
                        # Vira à esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.08))
                        achou_obj = False
                        print("ESQUERDA")
                        velocidade_saida.publish(vel)
                        continue                  

                    elif x_objeto > (centro_detect[0] + tolerancia):
                        # Vira à direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.08))  
                        velocidade_saida.publish(vel)
                        achou_obj = False
                        velocidade_saida.publish(vel)
                        print("DIREITA")
                        continue                  

                    elif (centro_detect[0]- tolerancia) < x_objeto < (centro[0] + tolerancia): # Gosto de usar a < b < c do Python. Não seria necessário neste caso
                        # Segue em frente
                        vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.02))
                        velocidade_saida.publish(vel)
                        achou_obj = False
                        print("FRENTE")

                        p=len(dist)-1 
                        if p>0: 
                            if dist[p]<=1:#a partir deste ponto, o robo para e o loop nao recomeca
                                    rospy.sleep(4)
                                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                                    achou_base=True
           
                            elif dist[p] <= 2: #a partir desta distancia anda mais devagar e para de ajustar a direcao
                                    vel = Twist(Vector3(0.075, 0, 0), Vector3(0, 0, 0))
                                    print("perto da base!!!")
                                
                        velocidade_saida.publish(vel)
                        continue

            if achou_base:
                        velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                        velocidade_saida.publish(velocidade)
                        raw_input()
                        tutorial.open_gripper()
                        raw_input()
                        tutorial.go_to_init_joint_state()
                        raw_input()
                        print("acabouuuuuuu!!!!")


    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
