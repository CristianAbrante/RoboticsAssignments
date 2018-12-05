#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Rob�tica Computacional - Curso 2014/2015
# Grado en Ingenier�a Inform�tica (Cuarto)
# Pr�ctica 5:
#     Simulaci�n de robots m�viles holon�micos y no holon�micos.

import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
# ******************************************************************************
# Declaraci�n de funciones

def distancia(a,b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def mostrar(objetivos,ideal,trayectoria):
  # Mostrar objetivos y trayectoria:
  plt.ion() # modo interactivo
  # Fijar los bordes del gr�fico
  objT   = np.array(objetivos).T.tolist()
  trayT  = np.array(trayectoria).T.tolist()
  bordes = [min(trayT[0]+objT[0]),max(trayT[0]+objT[0]),
            min(trayT[1]+objT[1]),max(trayT[1]+objT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])*.75
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar objetivos y trayectoria
  idealT = np.array(ideal).T.tolist()
  plt.plot(idealT[0],idealT[1],'-g')
  plt.plot(trayectoria[0][0],trayectoria[0][1],'or')
  r = radio * .1
  for p in trayectoria:
    plt.plot([p[0],p[0]+r*cos(p[2])],[p[1],p[1]+r*sin(p[2])],'-r')
    #plt.plot(p[0],p[1],'or')
  objT   = np.array(objetivos).T.tolist()
  plt.plot(objT[0],objT[1],'-.o')
  plt.show()
  raw_input()

# ******************************************************************************

# Definici�n del robot:
P_INICIAL = [0.,4.,0.] # Pose inicial (posici�n y orientacion)
V_LINEAL  = .7         # Velocidad lineal    (m/s)
V_ANGULAR = 200.       # Velocidad angular   (�/s)
FPS       = 10.        # Resoluci�n temporal (fps)

HOLONOMICO = 1
GIROPARADO = 0
LONGITUD   = .2

# Definici�n de trayectorias:
trayectorias = [
    [[2+2*sin(.4*pi*i),2+2*cos(.4*pi*i)] for i in range(5)],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)],
    [[2+2*sin(1.2*pi*i),2+2*cos(1.2*pi*i)] for i in range(5)],
    [[2*(i+1),4*(1+cos(pi*i))] for i in range(6)],
    [[2+.2*(22-i)*sin(.1*pi*i),2+.2*(22-i)*cos(.1*pi*i)] for i in range(20)],
    [[2+(22-i)/5*sin(.1*pi*i),2+(22-i)/5*cos(.1*pi*i)] for i in range(20)]
    ]

# Definici�n de los puntos objetivo:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <�ndice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definici�n de constantes:
EPSILON = .01               # Umbral de distancia
V = V_LINEAL/FPS            # Metros por fotograma
W = V_ANGULAR*pi/(180*FPS)  # Radianes por fotograma
trayectoria = [P_INICIAL]   # Trayectoria percibida
trayectreal = [P_INICIAL]   # Trayectoria seguida

ideal = robot()
ideal.set(*P_INICIAL) # operador 'splat'

real = robot()
real.set_noise(0.01,0.1,0.1)   # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

tiempo  = 0.
espacio = 0.
#random.seed(0)
random.seed(datetime.now())
for punto in objetivos:
  while distancia(trayectoria[-1],punto) > EPSILON and len(trayectoria) <= 1000:
    pose = ideal.pose()

    w = angulo_rel(pose,punto)
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto)
    if (v > V): v = V
    if (v < 0): v = 0

    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:
        v = 0
      ideal.move(w,v)
      real.move(w,v)
    else:
      ideal.move_triciclo(w,v,LONGITUD)
      real.move_triciclo(w,v,LONGITUD)
    trayectoria.append(ideal.pose())
    trayectreal.append(real.pose())

    espacio += v
    tiempo  += 1

if len(trayectoria) > 1000:
  print "<!> Trayectoria muy larga - puede que no se haya alcanzado la posici�n final."
print "Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s"
print "Distancia real al objetivo: "+\
    str(round(distancia(trayectreal[-1],objetivos[-1]),3))+"m"
mostrar(objetivos,trayectoria,trayectreal)  # Representaci�n gr�fica
