#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - Curso 2014/2015
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure(1)
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
  raw_input()
  plt.clf()

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)

  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]
# resuelve la cinemática directa.
# a partir de theta y a devuelve
# un vector de orígenes x, y
def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# aray de tipos de articulaciones
# 0 -> articulación de rotación
# 1 -> articulación prismáPráctica
tipo = [0, 0, 1]

# límites articulares
limSup = [np.radians(90), np.radians(90), 5]
limInf = [np.radians(-90), np.radians(-90), 0]

# valores articulares arbitrarios para la cinemática directa inicial
th=[0.,0.,0.] # Ángulos en radianes.
a =[5.,5.,0.]
L = sum(a) # variable para representación gráfica
EPSILON = .01

plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
if len(sys.argv) != 3:
  sys.exit("python " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]

# guardamos una lista de lista de puntos.
# para poder pintar las diferentes iteraciones.
O=range(len(th)+1) # Reservamos estructura en memoria
O[0]=cin_dir(th,a) # Calculamos la posicion inicial
print "- Posicion inicial:"
muestra_origenes(O[0])
dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  # Para cada combinación de articulaciones:
  for i in range(len(th)):
    # cálculo de la cinemática inversa
    index = len(th) - i - 1

    efector = O[i][len(th)]
    current_art = O[i][index]

    # articulacion de rotación
    if (tipo[index] == 0):
      v1 = np.subtract(objetivo, current_art)
      v2 = np.subtract(efector, current_art)
      a1 = np.arctan2([v2[0]], [v2[1]])
      a2 = np.arctan2([v1[0]], [v1[1]])
      angle = a1 - a2
      th[index] = th[index] + angle
      if (th[index] > np.pi):
        th[index] -= 2 * np.pi
      elif (th[index] < -np.pi):
        th[index] += 2 * np.pi

      if (th[index] > limSup[index]):
        th[index] = limSup[index]
      elif (th[index] < limInf[index]):
        th[index] = limInf[index]
    # articulación prismática
    else:
        w = np.sum(th[0 : index])
        d = np.dot(
            [np.cos(w), np.sin(w)],
            np.subtract(objetivo, efector)
        )
        a[index] = a[index] + d
        if (a[index] > limSup[index]):
            a[index] = limSup[index]
        elif (a[index] < limInf[index]):
            a[index] = limInf[index]

    O[i+1] = cin_dir(th,a)

  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print "\n- Iteracion " + str(iteracion) + ':'
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print "Distancia al objetivo = " + str(round(dist,5))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print "\n" + str(iteracion) + " iteraciones para converger."
else:
  print "\nNo hay convergencia tras " + str(iteracion) + " iteraciones."
print "- Umbral de convergencia epsilon: " + str(EPSILON)
print "- Distancia al objetivo:          " + str(round(dist,5))
print "- Valores finales de las articulaciones:"
for i in range(len(th)):
  print "  theta" + str(i+1) + " = " + str(round(th[i],3))
for i in range(len(th)):
  print "  L" + str(i+1) + "     = " + str(round(a[i],3))
