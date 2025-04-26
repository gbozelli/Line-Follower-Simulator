# -*- coding: utf-8 -*-
import math
import time
import os
import pygame 
import numpy as np
from matplotlib import image
from pygame.draw import line
import time 

black = (0, 0, 0)
blue = (0, 0, 255)
red = (255, 0, 0)
white = (255,255,255)
fps_sim = 50
fps_draw = 20
fps_draw_ratio = math.ceil(fps_sim/fps_draw)
fps_text_ratio = 2

MAX_SPEED =255
MIN_SPEED =-MAX_SPEED
P_TERM = 8 #3.5
I_TERM = 0.1
D_TERM = 57 #12
fator_erro = 1.0
erroAnterior = 0
somaErro = 0
valorPID = 0
somaValorPID = 0

def pid_control_task(sensor_values):
    global erroAnterior, somaErro, valorPID
    
    # Certifique-se de que temos exatamente 5 valores de sensor
    if len(sensor_values) != 5:
        return MAX_SPEED, MAX_SPEED
    
    # Normalizar os valores dos sensores (0 a 1)
    sensor_norm = [max(0, min(1, (0.5 - val) / 0.5)) for val in sensor_values]
    
    # Cálculo do erro analógico usando centro de massa
    total = sum(sensor_norm)
    if total > 0:
        # Posição ponderada dos sensores (-2, -1, 0, 1, 2)
        posicao = (-2*sensor_norm[0] -1*sensor_norm[1] +0*sensor_norm[2] 
                  +1*sensor_norm[3] +2*sensor_norm[4]) / total
        erro = posicao * 3  # Fator de escala (ajuste conforme necessário)
    else:
        # Nenhum sensor ativo, mantém o último erro válido
        erro = erroAnterior
    
    erro = fator_erro * erro
    somaErro += erro
    
    # Cálculo do PID (forma contínua)
    valorPID = P_TERM * erro + D_TERM * (erro - erroAnterior) + I_TERM * somaErro
    erroAnterior = erro
    
    # Cálculo das velocidades
    if valorPID >= 0:
        velocidadeEsq = MAX_SPEED
        velocidadeDir = MAX_SPEED - valorPID
    else:
        velocidadeEsq = MAX_SPEED + valorPID
        velocidadeDir = MAX_SPEED
    
    # Limites de velocidade
    velocidadeEsq = max(min(velocidadeEsq, MAX_SPEED), MIN_SPEED)
    velocidadeDir = max(min(velocidadeDir, MAX_SPEED), MIN_SPEED)
    
    return velocidadeEsq, velocidadeDir

def deg_to_rad(theta):
  return (theta%360)/180*math.pi
def rad_to_deg(theta):
  return ( theta%(2*math.pi))/math.pi*180


class Robot:
  # Constructor
  def __init__(self, x, y, R, yaw, scale, sensor_dist=0.85, sensor_gap=10):
    self.px = x 	
    self.py = y
    self.radius = R*scale
    self.yaw = deg_to_rad(yaw)
    self.scale = scale
    self.wheel_w = self.radius/8
    self.wheel_r = 3*self.wheel_w
    self.sensor_dist = sensor_dist*self.radius
    self.sensor_r = scale
    self.sensor_gap = sensor_gap  # in degree
    self.dist = 0
    self.vl = 0
    self.vr = 0
    self.sensor_pos_list = []
    for angle in range(-self.sensor_gap*2,self.sensor_gap*2+1,self.sensor_gap):
      self.sensor_pos_list.append([self.px+self.sensor_dist,self.py])

  def rotate_points(self,points_list,theta):
    for p in points_list:
      p[0],p[1] = self.px+math.cos(theta)*(p[0]-self.px) + math.sin(theta)*(p[1]-self.py),self.py-math.sin(theta)*(p[0]-self.px) + math.cos(theta)*(p[1]-self.py)
    return points_list
  
  def draw(self, screen):
    pygame.draw.circle(screen,blue,(self.px, self.py),self.radius,2)
    pygame.draw.circle(screen,blue,(self.px, self.py),self.sensor_dist,1)
    # pygame.draw.rect(screen, blue, [ ( self.px-self.wheel_r, self.py-self.radius-self.wheel_w ), ( 2*self.wheel_r, self.wheel_w) ], 1)
    # pygame.draw.rect(screen, blue, [ ( self.px-self.wheel_r, self.py+self.radius), ( 2*self.wheel_r, self.wheel_w) ], 1)
    pygame.draw.polygon(screen, blue, self.rotate_points([[self.px-self.wheel_r, self.py+self.radius], [self.px+self.wheel_r, self.py+self.radius], [self.px+self.wheel_r, self.py+self.radius+self.wheel_w], [self.px-self.wheel_r, self.py+self.radius+self.wheel_w]],self.yaw),2)
    pygame.draw.polygon(screen, blue, self.rotate_points([[self.px-self.wheel_r, self.py-self.radius-self.wheel_w], [self.px+self.wheel_r, self.py-self.radius-self.wheel_w], [self.px+self.wheel_r, self.py-self.radius], [self.px-self.wheel_r, self.py-self.radius]],self.yaw),2)
    for sensor in self.sensor_pos_list:
      pygame.draw.circle(screen,red,sensor,self.sensor_r,0)

  def update(self):
    v  = (self.vr + self.vl) / 2
    L = 2 * (self.radius + self.wheel_w)
    if L != 0:
      omega = (self.vr - self.vl) / L
    else:
      omega = 0
      
    delta_time = 1/fps_sim
    self.px += v * math.cos(self.yaw) * delta_time
    self.py -= v * math.sin(self.yaw) * delta_time  # Eixo Y invertido no Pygame
    self.yaw += omega * delta_time
    
    # Manter o yaw dentro de 0-2π
    self.yaw %= 2*math.pi
    
    # Atualizar distância percorrida (para estatísticas)
    self.dist += abs(v) * delta_time
    
    # Atualizar posições dos sensores (geometria direta)
    for i, angle in enumerate(range(-self.sensor_gap*2, self.sensor_gap*2+1, self.sensor_gap)):
      sensor_angle = self.yaw + math.radians(angle)
      self.sensor_pos_list[i] = [
        self.px + self.sensor_dist * math.cos(sensor_angle),
        self.py - self.sensor_dist * math.sin(sensor_angle)
        ]

  def manual_control(self,event):
    if (event.type == pygame.KEYDOWN):  
      if (event.key==pygame.K_UP):
        self.vr += MAX_SPEED/2
        self.vl += MAX_SPEED/2
      if (event.key==pygame.K_RIGHT):
        self.vr += MAX_SPEED/2
        self.vl -= MAX_SPEED/2
      if (event.key==pygame.K_LEFT):
        self.vr -= MAX_SPEED/2
        self.vl += MAX_SPEED/2
      if (event.key==pygame.K_DOWN):
        self.vr -= MAX_SPEED/2
        self.vl -= MAX_SPEED/2
    if (event.type == pygame.KEYUP):  
      if (event.key==pygame.K_UP):
        self.vr -= MAX_SPEED/2
        self.vl -= MAX_SPEED/2
      if (event.key==pygame.K_RIGHT):
        self.vr -= MAX_SPEED/2
        self.vl += MAX_SPEED/2
      if (event.key==pygame.K_LEFT):
        self.vr += MAX_SPEED/2
        self.vl -= MAX_SPEED/2
      if (event.key==pygame.K_DOWN):
        self.vr += MAX_SPEED/2
        self.vl += MAX_SPEED/2

    
  def automatic_control(self,sensors_values):
    self.vl,self.vr = pid_control_task(sensors_values)
    if abs(valorPID) > 1000:
      self.px = 200
      self.py = 180
      self.radius = 30 
      somaValorPID = 0

def main():
  pygame.init()
  clock = pygame.time.Clock()
  screen = pygame.display.set_mode((1080, 720), pygame.HWSURFACE)
  base_dir = os.path.dirname(__file__)
  filename = os.path.join(base_dir, 'circuit_1.png')
  image = pygame.image.load(filename).convert_alpha()
  img_arr = pygame.surfarray.array2d(image)
  img_arr[img_arr < -1] = 1
  img_arr = np.maximum(img_arr, 0)
  screen = pygame.display.set_mode(img_arr.shape, pygame.HWSURFACE)
  font = pygame.font.SysFont(None, 24)
  screen.blit(image, (0, 0))
  pygame.display.flip()
  initial_positions = {
    'circuit_1': (200, 180, 30, 20, 1),
    'circuit_2': (230, 480, 30, 90, 1)
  }

  line_follower = Robot(200, 180, 30, 20, 1)  # initial position for circuit_1
  # line_follower = Robot(230, 480, 30, 90, 1)  # initial position for circuit_2
  max_shift = math.ceil(MAX_SPEED/fps_draw)+line_follower.radius+line_follower.wheel_w+2
  rect_update_list = [pygame.Rect(line_follower.px-max_shift,line_follower.py-max_shift,2*max_shift,2*max_shift),pygame.Rect(10,10,90,55)]
  
  cnt_draw = 0
  cnt_text = 0
  done = False
  def pos_2_value(s):
    
    try:
      return img_arr[int(s[0]), int(s[1])]
    except IndexError:
      # Resetar posição do robô quando sair dos limites
      line_follower.__init__(*initial_positions['circuit_1'])
      end_time = time.time()
      somaValorPID = 0
      return 0  # Retorna valor padrão

  while done == False:
    sensor_list = list(map(pos_2_value,line_follower.sensor_pos_list))
    # print(list(sensor_list),line_follower.px,line_follower.py,rad_to_deg(line_follower.yaw),line_follower.vl,line_follower.vr)
    line_follower.automatic_control(sensor_list)  # Comment for manual control
    line_follower.update()
    
    if (cnt_draw%fps_draw_ratio==0):
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          done=True				
        # line_follower.manual_control(event)	 # Comment for automatic control
            
      screen.blit(image, (0, 0))			
      line_follower.draw(screen)
      rect_update_list[0].update(line_follower.px-max_shift,line_follower.py-max_shift,2*max_shift,2*max_shift)
      if (cnt_text%fps_text_ratio==0):
        screen.blit(font.render("left: "+str(line_follower.vl), True, white), (10,10))
        screen.blit(font.render("right: "+str(line_follower.vr), True, white), (10,25))
        screen.blit(font.render("dist: "+str(int(valorPID)), True, white), (10,40))
        cnt_text = 0
        pygame.display.update(rect_update_list)				
      else:
        pygame.display.update(rect_update_list[0])
      cnt_text += 1		
      cnt_draw = 0
    cnt_draw += 1
    clock.tick(fps_sim)  # Comment to accelerate simulation
  pygame.quit() 

if __name__ == "__main__":
  main()