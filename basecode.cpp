/*
 * Código do Robô Seguidor de Linha Xerapó
 * 
 * Funcionalidades:
 * - Controle PID baseado em 8 sensores infravermelhos
 * - Seguimento de linha eficiente
 * - Preparado para implementações futuras:
 * * Mapeamento da pista
 * * Detecção de volta completa (parada automática)
 * * Curvas de 90° otimizadas
 * 
 * Autor: Paulo Henrique do Amaral
 * Data: 05/2025
 */

#include <Arduino.h>
#include <QTRSensors.h>
#include <driver/ledc.h>
#include "BluetoothSerial.h"

// =============================================
// ============ CONFIGURAÇÕES ==================
// =============================================

// Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
int val, cnt = 0, v[3];
bool onoff = false;
bool debugBluetooth = true; // Habilita logs detalhados

// --- Sensores ---
QTRSensors qtr; // Objeto para os sensores QTR
const uint8_t SENSOR_COUNT = 8; 
uint16_t sensorValues[SENSOR_COUNT]; 
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {35, 32, 33, 25, 26, 27, 14, 13};
int sensorDigital[SENSOR_COUNT]; 

// Sensores especiais
const int SENSOR_PARADA = 20; // Sensor de final de percurso
const int SENSOR_CURVA = 21; // Sensor de curva 90°

// --- Controle dos Motores ---
// Configuração PWM
const int PWM_PINS[] = {2, 21}; // Pinos PWM
//const int PWM_CHANNELS[] = {0, 1}; // Canais PWM 
//const int PWM_FREQ = 1000; // Frequência (Hz)
//const int PWM_RESOLUTION = 8; // Resolução (bits)

// Pinos de controle dos motores
const int MOTOR_PINS[] = {4, 5, 18, 19}; // [IN1, IN2, IN3, IN4]

// Velocidades máximas (ajustar conforme necessário)
int VEL_MAX_ESQ = 50; // Motor esquerdo
int VEL_MAX_DIR = 50; // Motor direito //vel_max permitida é 2^(pwm_resolution)

// --- Controle PID ---
// Constantes (ajustar durante testes)
float KP = 10; // Ganho proporcional
float KD = 10; // Ganho derivativo
float KI = 0; // Ganho integral

// Variáveis do PID
int erro = 0;
int erroAnterior = 0;
int somaErro = 0;
int valorPID = 0;
float fator_erro = 1;

// Velocidades calculadas
int velocidadeEsq = 0;
int velocidadeDir = 0;

// =============================================
// ============ FUNÇÕES PRINCIPAIS =============
// =============================================

void setup() {
  // Inicialização serial para debug
  Serial.begin(115200);
  Serial.println("Iniciando robô Xerapó...");
  SerialBT.begin();
  Serial.println("Bluetooth Started! Ready to pair...");

  // Configura sensores de parada e curva
  pinMode(SENSOR_PARADA, INPUT);
  pinMode(SENSOR_CURVA, INPUT);

  // Configura sensores QTR
  qtr.setTypeRC();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT); // Declarando os pinos dos sensores qtr

  // Configura PWM
  for (int i = 0; i < 2; i++) {
   //ledcAttachChannel(PWM_PINS[i], PWM_FREQ, PWM_RESOLUTION, PWM_CHANNELS[i]); // configurando os pinos e os canis PWM
    pinMode(PWM_PINS[i], OUTPUT);
  }

  // Configura pinos dos motores
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
  }

  Serial.println("Configuração concluída!");
}

void loop() {
  // Verifica Bluetooth
  if (SerialBT.available()) {
    le_valores_bluetooth();
    atribui_valores_bluetooth();
  }

  // Controle do robô
  if (onoff) {
    lerSensores();
    calcularErro();
    calcularPID();
    movimentarRobo();
    
  } 
  else {
    pararMotores();
  }
  
  delay(1);
}

// =============================================
// ============ FUNÇÕES AUXILIARES =============
// =============================================

void lerSensores() {
  // Lê valores analógicos e converte para digital
  qtr.read(sensorValues);
  
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    // Debug: exibe valores no serial


    // Threshold para detecção da linha (ajustar conforme necessário)
    sensorDigital[i] = (sensorValues[i] <= 1500) ? 1 : 0;
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorDigital[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void calcularErro() {
/* 
  * Calcula o erro com base na posição da linha
  * Erros negativos: linha à direita
  * Erros positivos: linha à esquerda
  * 
  * Sistema de pesos:
  * -7 a +7 conforme a posição da linha
*/
  
    erroAnterior = erro; // Armazena erro anterior para cálculo derivativo

      // Verifica padrão dos sensores para determinar o erro
    if (sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==0 && 
            sensorDigital[4]==0 && sensorDigital[5]==0 && sensorDigital[6]==1) erro = -7.5;

    else if (sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==0 && 
            sensorDigital[4]==0 && sensorDigital[5]==1 && sensorDigital[6]==1) erro = -6;

    else if (sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==0 && 
            sensorDigital[4]==0 && sensorDigital[5]==1 && sensorDigital[6]==0) erro = -4.5;

    else if (sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==0 && 
            sensorDigital[4]==1 && sensorDigital[5]==1 && sensorDigital[6]==0) erro = -3;

    else if ( sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==0 && 
            sensorDigital[4]==1 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = -1.5;

    else if (sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==1 && 
            sensorDigital[4]==1 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = 0;

    else if (sensorDigital[1]==0 && sensorDigital[2]==0 && sensorDigital[3]==1 && 
            sensorDigital[4]==0 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = 1.5;

    else if (sensorDigital[1]==0 && sensorDigital[2]==1 && sensorDigital[3]==1 && 
            sensorDigital[4]==0 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = 3; // CENTRO

    else if (sensorDigital[1]==0 && sensorDigital[2]==1 && sensorDigital[3]==0 && 
            sensorDigital[4]==0 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = 4.5;

    else if (sensorDigital[1]==1 && sensorDigital[2]==1 && sensorDigital[3]==1 && 
            sensorDigital[4]==0 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = 6;

    else if (sensorDigital[1]==1 && sensorDigital[2]==0 && sensorDigital[3]==0 && 
            sensorDigital[4]==0 && sensorDigital[5]==0 && sensorDigital[6]==0) erro = 7.5;

    // Caso inválido (múltiplos sensores ativos ou linha perdida)
    else erro = erro; // Mantém o último erro válido
}

void calcularPID() {
  erro = fator_erro*erro;
  // Fórmula PID: PID = Kp*erro + Kd*(erro - erro_anterior) + Ki*soma_erro
  valorPID = KP * erro + KD * (erro - erroAnterior) + KI * somaErro;
}

void movimentarRobo() {
  // Ajusta velocidades com base no PID
  if (valorPID >= 0) {
    // Linha à esquerda - reduz motor direito
    velocidadeEsq = VEL_MAX_ESQ;
    velocidadeDir = VEL_MAX_DIR - valorPID;
  } else {
    // Linha à direita - reduz motor esquerdo
    velocidadeEsq = VEL_MAX_ESQ + valorPID; // PID negativo => soma reduz velocidade
    velocidadeDir = VEL_MAX_DIR;
  }

  // Garante que as velocidades estão dentro dos limites
  velocidadeEsq = constrain(velocidadeEsq, 0, VEL_MAX_ESQ);
  velocidadeDir = constrain(velocidadeDir, 0, VEL_MAX_DIR);

  // Aplica os comandos aos motores
  digitalWrite(MOTOR_PINS[0], LOW); // Motor A horário
  digitalWrite(MOTOR_PINS[1], HIGH); // Motor A anti-horário
  digitalWrite(MOTOR_PINS[2], LOW); // Motor B horário
  digitalWrite(MOTOR_PINS[3], HIGH); // Motor B anti-horário

  //ledcWrite(PWM_CHANNELS[0], velocidadeEsq); // Canal 0 = Motor Esquerdo
  //ledcWrite(PWM_CHANNELS[1], velocidadeDir); // Canal 1 = Motor Direito
  analogWrite(PWM_PINS[0], velocidadeEsq); // Canal 0 = Motor Esquerdo
  analogWrite(PWM_PINS[1], velocidadeDir); // Canal 1 = Motor Direito
}

void pararMotores() {
  // Para ambos os motores
  digitalWrite(MOTOR_PINS[0], LOW);
  digitalWrite(MOTOR_PINS[1], LOW);
  digitalWrite(MOTOR_PINS[2], LOW);
  digitalWrite(MOTOR_PINS[3], LOW);
  
  //ledcWrite(PWM_CHANNELS[0], 0); // Canal 0 = Motor Esquerdo
  //ledcWrite(PWM_CHANNELS[1], 0); // Canal 1 = Motor Direito
  analogWrite(PWM_PINS[0], velocidadeEsq); // Canal 0 = Motor Esquerdo
  analogWrite(PWM_PINS[1], velocidadeDir); // Canal 1 = Motor Direito
}

//In this void the the 2 read values are assigned.
void atribui_valores_bluetooth() {
  if (cnt == 2) { // Só processa se tiver par completo
    Serial.print("Atribuindo: ");
    Serial.print(v[1]);
    Serial.print(" = ");
    Serial.println(v[2]);
    
    switch(v[1]) {
      case 1:
        KP = constrain(v[2], 0, 100); // Limita entre 0-100
        Serial.print("KP atualizado para: ");
        Serial.println(KP);
        SerialBT.print("KP="); // Confirma via Bluetooth
        SerialBT.println(KP);
        break;
        
      case 2:
        KI = constrain(v[2], 0, 100);
        Serial.print("KI atualizado para: ");
        Serial.println(KI);
        SerialBT.print("KI=");
        SerialBT.println(KI);
        break;
        
      case 3:
        KD = constrain(v[2], 0, 100);
        Serial.print("KD atualizado para: ");
        Serial.println(KD);
        SerialBT.print("KD=");
        SerialBT.println(KD);
        break;
        
      case 4:
        fator_erro = constrain(v[2], 0, 10);
        Serial.print("Fator erro atualizado para: ");
        Serial.println(fator_erro);
        SerialBT.print("Fator erro = ");
        SerialBT.println(fator_erro);
        break;
        
      case 5:
        VEL_MAX_DIR = constrain(v[2], 0, 255);
        Serial.print("Vel. Max Dir. atualizada para: ");
        Serial.println(VEL_MAX_DIR);
        SerialBT.print("VEL_MAX_DIR = ");
        SerialBT.println(VEL_MAX_DIR;
        break;
        
      case 6:
        VEL_MAX_ESQ = constrain(v[2], 0, 255);
        Serial.print("Vel. Max Esq. atualizada para: ");
        Serial.println(VEL_MAX_ESQ);
        SerialBT.print("VEL_MAX_ESQ = ");
        SerialBT.println(VEL_MAX_ESQ;
        break;
        
      case 7:
        onoff = (v[2] != 0);
        Serial.print("Estado atualizado para: ");
        Serial.println(onoff ? "LIGADO" : "DESLIGADO");
        SerialBT.print("Estado: ");
        SerialBT.println(onoff);
        break;
      
      default:
        Serial.println("Comando desconhecido");
        SerialBT.println("ERRO: Comando desconhecido");
    }
    cnt = 0; // Reseta o contador
  }
}

void le_valores_bluetooth() {
  static String buffer = ""; // Buffer para acumular dados
  
  while (SerialBT.available()) {
    char c = SerialBT.read();
    
    if (c == '\n' || c == '\r') { // Final de comando
      if (buffer.length() > 0) {
        int commaPos = buffer.indexOf(',');
        if (commaPos > 0) {
          v[1] = buffer.substring(0, commaPos).toInt();
          v[2] = buffer.substring(commaPos+1).toInt();
          cnt = 2; // Marca como pronto para processar
          
          // Debug: mostra no Serial Monitor
          Serial.print("Recebido: ");
          Serial.print(v[1]);
          Serial.print(",");
          Serial.println(v[2]);
        }
        buffer = ""; // Limpa o buffer
      }
    } else {
      buffer += c; // Acumula caracteres
    }
  }
}

// =============================================
// ========= FUNÇÕES PARA IMPLEMENTAR ==========
// =============================================

/*void volta_para_pista() {
  if (erro_anterior < 0){
    velocidadeEsq = 0;
    velocidadeDir = VEL_MAX;
    digitalWrite(MOTOR_PINS[0], LOW); // Motor A horário
    digitalWrite(MOTOR_PINS[1], HIGH); // Motor A anti-horário
    digitalWrite(MOTOR_PINS[2], LOW); // Motor B horário
    digitalWrite(MOTOR_PINS[3], HIGH); // Motor B anti-horário

    ledcWrite(PWM_CHANNELS[0], velocidadeEsq); // Canal 0 = Motor Esquerdo
    ledcWrite(PWM_CHANNELS[1], velocidadeDir); // Canal 1 = Motor Direito
  }
} */