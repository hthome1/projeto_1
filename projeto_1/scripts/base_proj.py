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


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
viu_linhas=False
viu_linha1=False
viu_linha2=False

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
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()


def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

def comp(a,b):
    if a[1] > b[1]:
        return -1
    elif a[1] == b[1]:
        return 0
    else:
        return 1


def eq_reta(P1, P2):
    x1=P1[0]
    y1=P1[1]
    x2=P2[0]
    y2=P2[1]
    dx=x2-x1
    dy=y2-y1
    #coeficiente angular
    if dx != 0:
        m=dy/dx
    else:
        m=0
    #coeficiente linear
    h=y1-(m*x1)
    return (m,h)

def intersecao_retas(reta1,reta2):
    m1=reta1[0]
    m2=reta2[0]
    h1=reta1[1]
    h2=reta2[1]

    xi=int((h2-h1)/(m1-m2))
    yi= int((m1*xi)+h1)

    return (xi,yi)


def calcula_m(x2,x1,y2,y1):
    dx=(x2-x1)
    dy=(y2-y1)    
                
    if dx!=0:
        return dy/dx

def calcula_h(x,y,m):
    return y - m * x


dist = []
def scaneou(dado):
	#global dist
	d = (np.array(dado.ranges)[0]).round(decimals=2)
	dist.append(d)


	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	print(dist)
	#print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))


def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
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
		print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



def find_circles(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)
    # Detect the edges present in the image
    bordas = auto_canny(blur)
    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)


    #mascaras para filtrar o branco
    masks = cv2.inRange(gray, 240, 255)
    nmask=cv2.bitwise_not(masks)


    imagem = cv2.bitwise_or(frame, frame, mask=masks)
    imgcon= cv2.bitwise_and(gray, gray, mask=nmask)
    
    imagem2 = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)


    #mascaras para filtrar o branco
    masks = cv2.inRange(gray, 240, 255)
    nmask=cv2.bitwise_not(masks)


    imagem = cv2.bitwise_or(frame, frame, mask=masks)
    imgcon= cv2.bitwise_and(gray, gray, mask=nmask)
    
    imagem2 = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
    circles = None

    lines=[]
    lines = cv2.HoughLines(imagem2, 1, np.pi/180, 200)
    
    m1 = 0
    h1 = 0
    m2 = 0
    h2 = 0

    linha1 = False
    linha2 = False

    if lines is not None:
        for linha in lines:
            for r,theta in linha: 
                a = np.cos(theta) 
                b = np.sin(theta)  
                x0 = a*r 
                y0 = b*r 
                x1 = int(x0 + 1000*(-b))  
                y1 = int(y0 + 1000*(a))  
                x2 = int(x0 - 1000*(-b))  
                y2 = int(y0 - 1000*(a)) 
                
                m=calcula_m(x2,x1,y2,y1)

                if m < -0.1 and m > -3.3:
                    
                    if not linha1:
                        linha1=True
                        m1 = m
                        h1=calcula_h(x1,y1,m)
                        cv2.line(imagem,(x1,y1), (x2,y2), (0,255,0),2)

                elif m > 0.1 and m < 3.3:
                    if not linha2:
                        linha2 = True
                        m2 = m
                        h2 =calcula_h(x1,y1,m)
                        cv2.line(imagem,(x1,y1), (x2,y2), (0,255,0),2)


        if (m1-m2)!=0:    
            intersecao=intersecao_retas((m1,h1),(m2,h2))                
            cv2.circle(imagem,(intersecao[0],intersecao[1]), 15, (100,50,0), -1)

    imagem_final=frame+imagem
    cv2.imshow('Final', imagem_final)
    cv2.waitKey(1)

    if linha1 and linha2:
        return 0
    elif linha1 and not linha2:
        return 1
    elif linha2 and not linha1:
        return 2
    else:
        return False





# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global viu_linhas
    global viu_linha1
    global viu_linha2
    global resultados

    categoria="dog"
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        media, centro, maior_area =  visao_module.identifica_cor(cv_image)

        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro_detect, imagem, resultados =  visao_module.processa(cv_image)  
        vc_temp = find_circles(cv_image)
        if vc_temp==0:
            viu_linhas = True 
        elif vc_temp==1:
            viu_linha1=True
        elif vc_temp==2:
            viu_linha2=True     
        for r in resultados:
            if r[0]==categoria:
                achou_obj=True
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        #cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebedor_frames = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]
    acabou = False
    encontrou=False
    na_pista=True
    pegou_creeper=False
    try:
        # Inicializando - por default gira no sentido anti-horário
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        
        while not rospy.is_shutdown():
            #for r in resultados:
                #print(r)
            #velocidade_saida.publish(vel)
            #rospy.sleep(0.01)
            if na_pista:
            
                if viu_linhas:
                    print(" duas linhassssssssssssssssssssssssss")
                    vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)

                    viu_linhas = False
                    continue

                elif viu_linha1:
                    print(" 1111111111111111111111111111111111111111111")

                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.08))

                    velocidade_saida.publish(vel)

                    viu_linha1 = False

                    continue

                elif viu_linha2:
                    print(" 2222222222222222222222222222222222222222")
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.08))
                    velocidade_saida.publish(vel)

                    viu_linha2 = False
                    continue



            if len(media) !=0 and len(centro) != 0:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

                #print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
                #print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))
                if not acabou and not pegou_creeper:
                   # na_pista=False
                   p=len(dist)-1 
                   if p>0: 
                       #if dist[p]<=3.5:

				# ajusta a direcao ate encontrar o centro do objeto
				# quando encontra comeca a ir para frente, mas o ajuste de direcao continua ocorrendo, para evitar possiveis desvios
                        if (media[0] > (centro[0])):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                        if (media[0] < (centro[0])):
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))

                        if (abs(media[0] - centro[0])<=10): 
                            vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.0))
                            encontrou=True
                        velocidade_saida.publish(vel)
                        rospy.sleep(0.1)

                        if encontrou and not acabou:
                            print("encontrouuuuuuuuuuuuuuuu and not acabouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")

                        #scanner precisa haver detectado algo
                          #  print(dist[p])

                            if dist[p] <= 1.5: #a partir desta distancia anda mais devagar e para de ajustar a direcao
                                anda = Twist(Vector3(0.075, 0, 0), Vector3(0, 0, 0))
                                velocidade_saida.publish(anda)
                                rospy.sleep(0.1)



                            if dist[p]<=0.3:#a partir deste ponto, o robo para e o loop nao recomeca
                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.4))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(4.5)
                                acabou=True
                                print("acabouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")

                            if acabou:
                                na_pista=True
                                pegou_creeper=True
                                print("vaiiii pra pistaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
                                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                                velocidade_saida.publish(velocidade)
                                rospy.sleep(2)




    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


