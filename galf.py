import numpy as np
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.core.problem import Problem
from pymoo.optimize import minimize
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.termination import get_termination
import pygame
import math
import os
# -*- coding: utf-8 -*-
import math
import time
import os
import pygame 
import numpy as np
from matplotlib import image
from pygame.draw import line
from pymoo.core.problem import ElementwiseProblem
from pymoo.optimize import minimize
import numpy as np

black = (0, 0, 0)
blue = (0, 0, 255)
red = (255, 0, 0)
white = (255,255,255)
fps_sim = 50
fps_draw = 20
fps_draw_ratio = math.ceil(fps_sim/fps_draw)
fps_text_ratio = 2

MAX_SPEED =100
MIN_SPEED =-MAX_SPEED
P_TERM = 30 #3.5
I_TERM = 0
D_TERM = 15 #12
fator_erro = 1.0
erroAnterior = 0
somaErro = 0

def pid_control_task(sensor_values):
    global erroAnterior, somaErro
    
    # Certifique-se de que temos exatamente 5 valores de sensor
    if len(sensor_values) != 5:
        return MAX_SPEED, MAX_SPEED  # Retorna valores padrão se não houver 5 sensores
    
    # Converter valores dos sensores para digital (threshold 0.5)
    sensorDigital = [1 if val <= 0.5 else 0 for val in sensor_values]
    
    # Cálculo do erro com sistema de pesos
    erro = 0
    
    # Padrões dos sensores (para 5 sensores)
    if sensorDigital == [1, 0, 0, 0, 0]:
        erro = 6
    elif sensorDigital == [1, 1, 0, 0, 0]:
        erro = 4.5
    elif sensorDigital == [0, 1, 0, 0, 0]:
        erro = 3
    elif sensorDigital == [0, 1, 1, 0, 0]:
        erro = 1.5
    elif sensorDigital == [0, 0, 1, 0, 0]:
        erro = 0
    elif sensorDigital == [0, 0, 1, 1, 0]:
        erro = -1.5
    elif sensorDigital == [0, 0, 0, 1, 0]:
        erro = -3
    elif sensorDigital == [0, 0, 0, 1, 1]:
        erro = -4.5
    elif sensorDigital == [0, 0, 0, 0, 1]:
        erro = -6
    else:
        erro = erroAnterior  # Mantém o último erro válido
    
    erro = fator_erro * erro
    somaErro += erro
    
    # Cálculo do PID
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



# Configurações do simulador (versão simplificada para avaliação)
class LineFollowerSimulator:
    def __init__(self):
        pygame.init()
        clock = pygame.time.Clock()
        screen = pygame.display.set_mode((1080, 720), pygame.HWSURFACE)
        base_dir = os.path.dirname(__file__)
        filename = os.path.join(base_dir, 'circuit_1.png')
        self.image = pygame.image.load(filename).convert_alpha()
        self.img_arr = pygame.surfarray.array2d(self.image)
        self.img_arr[self.img_arr < -1] = 1
        self.img_arr = np.maximum(self.img_arr, 0)
        
    def evaluate_pid(self, kp, ki, kd, max_steps=1000):
        # Configuração do robô
        robot = Robot(200, 180, 30, 20, 1)
        
        # Configura os parâmetros PID
        global P_TERM, I_TERM, D_TERM
        P_TERM, I_TERM, D_TERM = kp, ki, kd
        
        total_error = 0
        steps_on_track = 0
        max_speed = MAX_SPEED
        
        for step in range(max_steps):
            # Obtém valores dos sensores
            sensor_values = [self.img_arr[int(s[0]), int(s[1])] for s in robot.sensor_pos_list]
            
            # Se todos os sensores estiverem fora da linha (branco), penaliza
            if all(v > 0.5 for v in sensor_values):
                total_error += 100  # Grande penalização por sair da linha
                break
            
            # Calcula o erro
            sensorDigital = [1 if val <= 0.5 else 0 for val in sensor_values]
            erro = 0
            
            # Padrões dos sensores (para 5 sensores)
            if sensorDigital == [1, 0, 0, 0, 0]:
              erro = 6
            elif sensorDigital == [1, 1, 0, 0, 0]:
              erro = 4.5
            elif sensorDigital == [0, 1, 0, 0, 0]:
              erro = 3
            elif sensorDigital == [0, 1, 1, 0, 0]:
              erro = 1.5
            elif sensorDigital == [0, 0, 1, 0, 0]:
              erro = 0
            elif sensorDigital == [0, 0, 1, 1, 0]:
              erro = -1.5
            elif sensorDigital == [0, 0, 0, 1, 0]:
              erro = -3
            elif sensorDigital == [0, 0, 0, 1, 1]:
              erro = -4.5
            elif sensorDigital == [0, 0, 0, 0, 1]:
              erro = -6
            else:

                # Se múltiplos sensores ativos ou nenhum, mantém último erro
              pass
            
            total_error += abs(erro)
            steps_on_track += 1
            
            # Atualiza o robô
            robot.automatic_control(sensor_values)
            robot.update()
            
            # Verifica se saiu do mapa
            if (robot.px < 0 or robot.px >= self.img_arr.shape[0] or 
                robot.py < 0 or robot.py >= self.img_arr.shape[1]):
                total_error += 1000  # Grande penalização por sair do mapa
                break
        
        # Fitness: combinação de erro total e distância percorrida
        fitness = total_error / steps_on_track if steps_on_track > 0 else 1e6
        return fitness



# Função de simulação simplificada (você pode adaptar com mais realismo)
def run_simulation(kp, ki, kd, max_steps=500):
    global erroAnterior, somaErro, P_TERM, I_TERM, D_TERM
    erroAnterior = 0
    somaErro = 0
    P_TERM = kp
    I_TERM = ki
    D_TERM = kd

    # Reuse a estrutura do seu main aqui, mas com passos limitados e sem pygame
    # Aqui só ilustramos acumulando erro com sensores fictícios
    erro_total = 0
    robot = Robot(200, 180, 30, 20, 1)
    
    for _ in range(max_steps):
        # Sensores simulados (ideal: rodar na imagem como no seu main)
        sensor_vals = np.random.rand(5)  # Substitua por sensores reais ou simulados na linha

        robot.automatic_control(sensor_vals)
        robot.update()

        # Acumula o erro absoluto
        erro_total += abs(erroAnterior)
    
    return erro_total

# Classe de problema para pymoo
class PIDTuningProblem(ElementwiseProblem):
    def __init__(self):
        super().__init__(n_var=3,
                         n_obj=1,
                         n_constr=0,
                         xl=np.array([0.0, 0.0, 0.0]),  # limites inferiores: kp, ki, kd
                         xu=np.array([100.0, 100.0, 100.0]))  # limites superiores

    def _evaluate(self, x, out, *args, **kwargs):
        kp, ki, kd = x
        error = run_simulation(kp, ki, kd)
        out["F"] = error  # menor erro é melhor

# Algoritmo genético
problem = PIDTuningProblem()

algorithm = GA(
    pop_size=20,
    eliminate_duplicates=True
)

res = minimize(problem,
               algorithm,
               termination=('n_gen', 20),
               seed=1,
               save_history=True,
               verbose=True)

print("Melhores valores encontrados:")
print("Kp:", res.X[0], "Ki:", res.X[1], "Kd:", res.X[2])
