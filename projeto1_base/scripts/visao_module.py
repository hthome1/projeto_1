#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet_simples as mnet

global color
global base
instrucoes={
    "color": "blue",
    "base": "cat",
    "id": 11
}



def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged



def processa(frame):
    '''Use esta funcao para basear o processamento do seu robo'''

    result_frame, result_tuples = mnet.detect(frame)

    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - int(length/2), point[1]),  (point[0] + int(length/2), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - int(length/2)), (point[0], point[1] + int(length/2)),color ,width, length)

    cross(result_frame, centro, [255,0,0], 1, 17)

  #  cv2.imshow('video', result_frame)
 #   cv2.waitKey(1)

    return centro, result_frame, result_tuples




def cor_creeper(frame, color):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color == "green":
        cor_menor = np.array([57, 50, 50])
        cor_maior = np.array([67, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    elif color == "blue":
            cor_menor = np.array([94, 50, 50])
            cor_maior = np.array([104, 255, 255])
            segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    elif color == "pink":
            cor_menor = np.array([138, 50, 50])
            cor_maior = np.array([152, 255, 255])
            segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    return segmentado_cor

    

   




def identifica_cor(frame):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
    # vermelho puro (H=0) estão entre H=-8 e H=8. 
    # Precisamos dividir o inRange em duas partes para fazer a detecção 
    # do vermelho:
    # frame = cv2.flip(frame, -1) # flip 0: eixo x, 1: eixo y, -1: 2 eixos
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    
    segmentado_cor = cor_creeper(frame, instrucoes["color"])

   

   #cor_menor = np.array([172, 50, 50])
    #cor_maior = np.array([180, 255, 255])
    #segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)

    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos inverter, porque 
    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - int(length/2), point[1]),  (point[0] + int(length/2), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - int(length/2)), (point[0], point[1] + int(length/2)),color ,width, length)


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
  #  cv2.imshow('seg', segmentado_cor)
  #  cv2.waitKey(1)

    return media, centro, maior_contorno_area










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



