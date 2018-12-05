#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - Curso 2014/2015
# Grado en Ingeniería Informática (Cuarto)
# Clase robot

from math import *
import random
import numpy as np
import copy

class robot:
  def __init__(self):
    # Inicializacion de pose y parámetros de ruído
    self.x             = 0.
    self.y             = 0.
    self.orientation   = 0.
    self.forward_noise = 0.
    self.turn_noise    = 0.
    self.sense_noise   = 0.
    self.weight        = 1.
    self.old_weight    = 1.
    self.size          = 1.

  def copy(self):
    # Constructor de copia
    return copy.deepcopy(self)

  def set(self, new_x, new_y, new_orientation):
    # Modificar la pose
    self.x = float(new_x)
    self.y = float(new_y)
    self.orientation = float(new_orientation)
    while self.orientation >  pi: self.orientation -= 2*pi
    while self.orientation < -pi: self.orientation += 2*pi

  def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
    # Modificar los parámetros de ruído
    self.forward_noise = float(new_f_noise);
    self.turn_noise    = float(new_t_noise);
    self.sense_noise   = float(new_s_noise);

  def pose(self):
    # Obtener pose actual
    return [self.x, self.y, self.orientation]

  def sense1(self,landmark,noise):
    # Calcular la distancia a una de las balizas
    return np.linalg.norm(np.subtract([self.x,self.y],landmark)) \
                                        + random.gauss(0.,noise)

  def sense(self, landmarks):
    # Calcular las distancias a cada una de las balizas
    d = [self.sense1(l,self.sense_noise) for l in landmarks]
    d.append(self.orientation + random.gauss(0.,self.sense_noise))
    return d

  def move(self, turn, forward):
    # Modificar pose del robot (holonómico)
    self.orientation += float(turn) + random.gauss(0., self.turn_noise)
    while self.orientation >  pi: self.orientation -= 2*pi
    while self.orientation < -pi: self.orientation += 2*pi
    dist = float(forward) + random.gauss(0., self.forward_noise)
    self.x += cos(self.orientation) * dist
    self.y += sin(self.orientation) * dist

  def move_triciclo(self, turn, forward, largo):
    # Modificar pose del robot (Ackermann)
    dist = float(forward) + random.gauss(0., self.forward_noise)
    self.orientation += dist * tan(float(turn)) / largo\
              + random.gauss(0.0, self.turn_noise)
    while self.orientation >  pi: self.orientation -= 2*pi
    while self.orientation < -pi: self.orientation += 2*pi
    self.x += cos(self.orientation) * dist
    self.y += sin(self.orientation) * dist

  def Gaussian(self, mu, sigma, x):
    # Calcular la probabilidad de 'x' para una distribución normal
    # de media 'mu' y desviación típica 'sigma'
    if sigma:
      return exp(-(((mu-x)/sigma)**2)/2)/(sigma*sqrt(2*pi))
    else:
      return 0

  def measurement_prob(self, measurements, landmarks):
    # Calcular la probabilidad de una medida.
    self.weight = 1.
    for i in range(len(measurements)-1):
      self.weight *= self.Gaussian(self.sense1(landmarks[i],0),\
                            self.sense_noise, measurements[i])
    diff = self.orientation - measurements[-1]
    while diff >  pi: diff -= 2*pi
    while diff < -pi: diff += 2*pi
    self.weight *= self.Gaussian(0, self.sense_noise, diff)
    return self.weight

  def __repr__(self):
    # Representación de la clase robot
    return '[x=%.6s y=%.6s orient=%.6s]' % \
            (str(self.x), str(self.y), str(self.orientation))

