/** 
 * Universidade de Brasília
 * Laboratório de Automação e Robótica - LARA
 *
 * Programa: Recebe diversos comandos eviados do Matlab e as executa enviando 
 *           comandos via serial para o manipulador Cyton Alpha 7D1G
 *
 * @authors De Hong Jung , Rafael Lima
 */

#include <SSC32.h>

#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>
#include <serstream>
#include <sstream>
#include <string>
#include <vector>
#include <iterator>

#include <elapsedMillis.h>

using namespace std;

//=========================== Variaveis =================================

float value = 0.0;
float erro = 0.0, servoPosicaoMap = 0.0;
int count, k, duration;
vector <float> angles_ref;
vector <float> angles_feedback;
vector <float> angles_cmd;

// Minimo e Maximo em Volts
float min_array[] = {2.72, 0.7, 2.7, 4.06, 2.7, 2.7, 2.75, 2.69};
float max_array[] = {0.75, 4.26, 0.75, 0.45, 0.63, 0.63, 0.6, 0.69};

// Minimo e Maximo do Servo
float min_array2[] = {2330, 2320, 2350, 800, 2370, 2300, 2370};
float max_array2[] = {720, 920, 730, 2100, 610, 600, 580};
float min_degree[] = {-90,-86, -90, -83, -90, -90,-90};
float max_degree[] = {90, 65, 90, 62, 90, 90, 90};

// Inputs
int servoPin = 6, servoPosicao, Input;

float Setpoint, Output;

// Time
elapsedMillis timeElapsed;

// Servo Motors Controler
SSC32 myssc = SSC32();

//============= Classes ============================================

namespace std {
  ohserialstream cout(Serial);
}

//=============== Funções =========================================

// Funcao de Mapaeamento de variavel float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
 
}

// Leitura de String da Serial
String readSerial() {
  String inData = "";
  if (Serial.available() > 0) {
    int h = Serial.available();
    for (int i = 0; i < h; i++) {
      inData += (char)Serial.read();
    }
    return inData;
  }
  else {
    return "No connection";
  }
}

// Leitura de posicao dos servos
vector <float> Feedback(){
  vector <float> angles;
  
  for (int i = 0; i < 7; i++){
    value = analogRead(i)*(5.0/1023.0);
    value = mapfloat(value, min_array[i], max_array[i], -90, 90);
    angles.push_back(value);
  }
  return angles;
}

// Funcao de move todos os servos a partir de um vetor de angulos (em graus)
void setMultServos(vector <float> angles, int dt){
  
  myssc.beginGroupCommand(SSC32_CMDGRP_TYPE_SERVO_MOVEMENT);
  
  value = mapfloat(angles[0], min_degree[0], max_degree[0], min_array2[0], max_array2[0]);
  myssc.servoMoveTime(0, value, dt);
  value = mapfloat(angles[1], min_degree[1], max_degree[1], min_array2[1], max_array2[1]);
  myssc.servoMoveTime(1, value, dt);
  value = mapfloat(angles[2], min_degree[2], max_degree[2], min_array2[2], max_array2[2]);
  myssc.servoMoveTime(2, value, dt);
  value = mapfloat(angles[3], min_degree[3], max_degree[3], min_array2[3], max_array2[3]);
  myssc.servoMoveTime(3, value, dt);
  value = mapfloat(angles[4], min_degree[4], max_degree[4], min_array2[4], max_array2[4]);
  myssc.servoMoveTime(4, value, dt);
  value = mapfloat(angles[5], min_degree[5], max_degree[5], min_array2[5], max_array2[5]);
  myssc.servoMoveTime(5, value, dt);
  value = mapfloat(angles[6], min_degree[6], max_degree[6], min_array2[6], max_array2[6]);
  myssc.servoMoveTime(6, value, dt);
  
  myssc.endGroupCommand();

  delay(100);
}


// Função que lê a string de angulos da serial e a tranforma em vetor de angulos float
vector <float> ReadAngles(){
  
  vector <float> angles_float;

  angles_float.clear();
  if (Serial.available()){
    for (int i = 0; i < 7; i++)
      angles_float.push_back(Serial.readStringUntil(' ').toFloat());  
  }
  return angles_float;
}

// Executa a Forward Kinematics a partir dos angulos das juntas enviadas do Matlab
vector <float> FKM(){
  
  vector <float> angles_float;

  angles_float = ReadAngles();
  if (angles_float[0] == 999)
    return angles_float;
  else{
    duration = 700;
    setMultServos(angles_float, duration);
    return angles_float;
  }
}

// Funcao que inicializa todos os servos para posição 0 graus
void SetInitialPosition(){
  
  duration = 2000;

  myssc.beginGroupCommand(SSC32_CMDGRP_TYPE_SERVO_MOVEMENT);
  value = mapfloat(0, min_degree[0], max_degree[0], min_array2[0], max_array2[0]);
  myssc.servoMoveTime(0, value, duration);
  value = mapfloat(0, min_degree[1], max_degree[1], min_array2[1], max_array2[1]);
  myssc.servoMoveTime(1, value, duration);
  value = mapfloat(0, min_degree[2], max_degree[2], min_array2[2], max_array2[2]);
  myssc.servoMoveTime(2, value, duration);
  value = mapfloat(0, min_degree[3], max_degree[3], min_array2[3], max_array2[3]);
  myssc.servoMoveTime(3, value, duration);
  value = mapfloat(0, min_degree[4], max_degree[4], min_array2[4], max_array2[4]);
  myssc.servoMoveTime(4, value, duration);
  value = mapfloat(0, min_degree[5], max_degree[5], min_array2[5], max_array2[5]);
  myssc.servoMoveTime(5, value, duration);
  value = mapfloat(0, min_degree[6], max_degree[6], min_array2[6], max_array2[6]);
  myssc.servoMoveTime(6, value, duration);
//  value = mapfloat(0, min_array[7], max_array[7], -90, 90);
//  myssc.servoMoveTime(7,value,3000);
  myssc.endGroupCommand();
  
  delay(2000);

}

//============== Setup ===========================================

void setup() {
  // Start Comunication 
  myssc.begin(9600);  // Servo Motors (Serial1)
  Serial.begin(9600); // Computer
  
  analogReference(EXTERNAL);  // Ligar porta AREF em 3.3V -> Aumenta resolução
  
  // Set angles to zero
  for (k = 0; k < 7; k++)
    angles_cmd[k] = 0;

  // Move Gently Robot to Zero Position
  SetInitialPosition();

  // Set time to zero
  timeElapsed = 0;
}

void printAngles(){

}

//=============== Loop ============================================

void loop() {

  /////// Read Input ///////
  
  // Feedback
  angles_feedback = Feedback();

  // Print Time Stamp
  Serial.print('R ');
  Serial.print(timeElapsed);
  Serial.print(' ');
  // Print Current Joint Angles
  for (k = 0; k < 6; k++){
    Serial.print(angles_feedback[k]);
    Serial.print(' ');
  }
  Serial.println(angles_feedback[k]);

  //////// Command /////////

  // Reset timer
  timeElapsed = 0;

  // Set Angles
  for (k = 0; k < 7; k++)
    angles_cmd[k] = 0;

  // Print Command Angles
  Serial.print('C');
  Serial.print(timeElapsed);
  Serial.print(' ');
  for (k = 0; k < 6; k++){
    Serial.print(angles_cmd[k]);
    Serial.print(' ');
  }
  Serial.println(angles_cmd[k]);

  //////// Actuation /////////

  // Send Servo command
  //setMultServos(angles_cmd,1000);
  //delay(1000);
  SetInitialPosition();
}

