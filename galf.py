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
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.termination import get_termination
from pymoo.operators.selection.tournament import TournamentSelection
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

MAX_SPEED =255
MIN_SPEED =-MAX_SPEED
P_TERM = 30 #3.5
I_TERM = 0
D_TERM = 15 #12
fator_erro = 1.0
erroAnterior = 0
somaErro = 0
valorPID = 0
last_time = time.time()

def pid_control_task(sensor_values):
    global erroAnterior, somaErro, valorPID, last_time
    current_time = time.time()
    delta_time = current_time - last_time
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
    valorPID = P_TERM * erro + D_TERM * (erro - erroAnterior)/delta_time + I_TERM * somaErro
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
    self.effective_dist = 0
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
      
    self.effective_dist += calculate_effective_distance(self)

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

def calculate_effective_distance(robot):
    # Calcula a velocidade linear (módulo da velocidade de translação)
    linear_speed = math.sqrt((robot.vr + robot.vl)**2) / 2  # Módulo da velocidade média
    
    # Se a velocidade linear for muito baixa, consideramos que o robô está girando no lugar
    if linear_speed < 5:  # Limiar pequeno para considerar movimento insignificante
        return 0
    
    # Calcula o deslocamento neste passo de tempo
    delta_time = 1/fps_sim
    displacement = linear_speed * delta_time
    
    return displacement

# Função de simulação simplificada (você pode adaptar com mais realismo)
# Classe de problema para pymoo
# ... (código anterior permanece o mesmo)

def run_simulation(kp, ki, kd):
    # Configura os parâmetros PID
    global P_TERM, I_TERM, D_TERM, somaErro, erroAnterior
    P_TERM, I_TERM, D_TERM = kp, ki, kd
    somaErro = 0
    erroAnterior = 0

    # Inicializa o pygame (se ainda não estiver inicializado)
    pygame.init()
    screen = pygame.display.set_mode((1080, 720), pygame.HWSURFACE)
    base_dir = os.path.dirname(__file__)
    filename = os.path.join(base_dir, 'circuit_1.png')
    image = pygame.image.load(filename).convert_alpha()
    img_arr = pygame.surfarray.array2d(image)
    img_arr[img_arr < -1] = 1
    img_arr = np.maximum(img_arr, 0)

    # Configuração do robô
    line_follower = Robot(260, 290, 30, 90, 1) 
    max_shift = math.ceil(MAX_SPEED / fps_draw) + line_follower.radius + line_follower.wheel_w + 2
    rect_update_list = [pygame.Rect(line_follower.px - max_shift, line_follower.py - max_shift, 2 * max_shift, 2 * max_shift),
                        pygame.Rect(10, 10, 90, 55)]
    initial_positions = {
      'circuit_1': (260, 290, 30, 90, 1),
      'circuit_2': (230, 480, 30, 90, 1)
    }

  
    start_time = time.time()
    max_steps = 1000  # Número máximo de passos
    
    def pos_2_value(s):
        try:
            return img_arr[int(s[0]), int(s[1])]
        except IndexError:
            # Resetar posição do robô quando sair dos limites
            line_follower.__init__(*initial_positions['circuit_1'])
            ValorPID = 10000
            return 0  # Retorna valor padrão
            
    for _ in range(max_steps):
        sensor_list = list(map(pos_2_value, line_follower.sensor_pos_list))
        line_follower.automatic_control(sensor_list)
        line_follower.update()

        # Critério de parada (se o robô sair da pista)
        if abs(valorPID) > 1000:
            break
            
    end_time = time.time()
    elapsed_time = end_time - start_time
    
    # Retorna uma tupla com: (tempo, erro acumulado, deslocamento)
    # Queremos maximizar o tempo, minimizar o erro acumulado e maximizar o deslocamento
    return elapsed_time, line_follower.effective_dist

class PIDTuningProblem(ElementwiseProblem):
    def __init__(self):
        super().__init__(n_var=3,
                         n_obj=2,  # Agora temos três objetivos
                         n_constr=0,
                         xl=np.array([10.0, 0.0, 10.0]),  # limites inferiores: kp, ki, kd
                         xu=np.array([20.0, 1.0, 40.0]))  # limites superiores

    def _evaluate(self, x, out, *args, **kwargs):
        kp, ki, kd = x
        time, distance = run_simulation(kp, ki, kd)
        print(10*time)
        print('kp:',kp)
        print('ki:',ki)
        print('kd:',kd)
        out["F"] = [-time,-distance]

def run_optimization():
    problem = PIDTuningProblem()
    
    # Usamos NSGA2 para otimização multiobjetivo
    from pymoo.algorithms.moo.nsga2 import NSGA2
    algorithm = NSGA2(
        pop_size=100,
        eliminate_duplicates=True,
        crossover=SBX(prob=0.9, eta=10),
        mutation=PM(eta=5)
    )
    
    # Configuramos critérios de parada
    termination = get_termination("n_gen", 10)
    
    res = minimize(problem,
                   algorithm,
                   termination,
                   seed=1,
                   save_history=True,
                   verbose=True)
    
    # Encontramos a solução com melhor compromisso entre os três objetivos
    from pymoo.decomposition.asf import ASF
    decomp = ASF()
    
    # Pesos: ajuste conforme a importância de cada objetivo
    # [tempo, erro, deslocamento]
    weights = np.array([0.2, 0.3])  # Dando mais peso ao erro acumulado
    
    # Normaliza os objetivos para mesma escala
    F = res.F
    F_normalized = (F - F.min(axis=0)) / (F.max(axis=0) - F.min(axis=0) + 1e-6)
    
    # Encontra o índice da melhor solução
    best_idx = np.argmin(decomp.do(F_normalized, weights))
    
    print("\nMelhores valores encontrados:")
    print(f"Kp: {res.X[best_idx][0]:.2f}, Ki: {res.X[best_idx][1]:.5f}, Kd: {res.X[best_idx][2]:.2f}")
    print(f"Desempenho:")
    print(f"- Tempo na pista: {-res.F[best_idx][0]:.5f} segundos")

    
    return res.X[best_idx]  # Retorna os melhores parâmetros encontrados

# ... (restante do código permanece o mesmo)
import threading

def main():
    # Thread para a otimização (algoritmo genético)
    optimization_thread = threading.Thread(target=run_optimization)
    optimization_thread.daemon = True  # Encerra quando a main terminar
    optimization_thread.start()

    # Thread para a simulação gráfica
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.daemon = True
    simulation_thread.start()

    # Mantém o programa rodando
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()