/** 
 * Universidade de Brasília
 * Laboratório de Automação e Robótica
 * @authors De Hong Jung , Rafael Lima
 * Programa: Recebe diversos comandos eviados do Matlab e as executa enviando 
 *           comandos via serial para o manipulador Cyton Alpha 7D1G
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

using namespace std;

//=========================== Variaveis =================================

float value = 0.0;
float erro = 0.0, servoPosicaoMap = 0.0;
int count, k, duration;
vector <float> angles_ref;
vector <float> angles_feedback;
vector <float> ang0;vector <float> ang1;vector <float> ang2;vector <float> ang3;vector <float> ang4;vector <float> ang5;vector <float> ang6;
vector <float> ang;

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

//============= Classes ============================================

class PID{
public:
  
  float error;
  float sample;
  float lastSample;
  float kP, kI, kD;      
  float P, I, D;
  float pid;
  
  float setPoint;
  float lastProcess;
  
  PID(float _kP, float _kI, float _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(float _sample){
    sample = _sample;
  }
  
  void setSetPoint(float _setPoint){
    setPoint = _setPoint;
  }
  
  float process(){
    // Implementação P ID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

PID servoPID0(-0.1, 0.005, 0.0005);PID servoPID1(-0.1, 0.005, 0.0005);PID servoPID2(-0.1, 0.005, 0.0005);
PID servoPID3(-0.1, 0.005, 0.0005);PID servoPID4(-0.1, 0.005, 0.0005);PID servoPID5(-0.1, 0.005, 0.0005);
PID servoPID6(-0.1, 0.005, 0.0005);

namespace std {
  ohserialstream cout(Serial);
}


SSC32 myssc = SSC32();

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

// Controle PID
void PIDControl(vector <float> angles_float, vector <float> feedback){ 
  vector <float> pidOut;  
  
  // Setpoint
  servoPID0.setSetPoint(angles_float[0]);servoPID1.setSetPoint(angles_float[1]);servoPID2.setSetPoint(angles_float[2]);
  servoPID3.setSetPoint(angles_float[3]);servoPID4.setSetPoint(angles_float[4]);servoPID5.setSetPoint(angles_float[5]);
  servoPID6.setSetPoint(angles_float[6]);

  // Sample
  servoPID0.addNewSample(feedback[0]);servoPID1.addNewSample(feedback[1]);servoPID2.addNewSample(feedback[2]);
  servoPID3.addNewSample(feedback[3]);servoPID4.addNewSample(feedback[4]);servoPID5.addNewSample(feedback[5]);
  servoPID6.addNewSample(feedback[6]);

  // Process
  pidOut.push_back(servoPID0.process());pidOut.push_back(servoPID1.process());pidOut.push_back(servoPID2.process());
  pidOut.push_back(servoPID3.process());pidOut.push_back(servoPID4.process());pidOut.push_back(servoPID5.process());
  pidOut.push_back(servoPID6.process());

  // Compensation
  duration = 700;
  setMultServos(pidOut, duration);
    
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

void Replay(){
    vector <float> angles;
    angles.push_back(0.0);
    
    for (int i = 0; i < ang0.size(); i++){
      angles.clear();
      angles.push_back(ang0[i]);angles.push_back(ang1[i]);angles.push_back(ang2[i]);
      angles.push_back(ang3[i]);angles.push_back(ang4[i]);angles.push_back(ang5[i]);
      angles.push_back(ang6[i]);

      duration = 500;
      setMultServos(angles, duration);
      delay(400);
    }
}

void Record(){
    int dt = 400;
    ang0.clear();ang1.clear();ang2.clear();
    ang3.clear();ang4.clear();ang5.clear();
    ang6.clear();
    
    while (!Serial.available()){
        angles_ref.clear();
        angles_ref = Feedback();
        ang0.push_back(angles_ref[0]);ang1.push_back(angles_ref[1]);ang2.push_back(angles_ref[2]);
        ang3.push_back(angles_ref[3]);ang4.push_back(angles_ref[4]);ang5.push_back(angles_ref[5]);
        ang6.push_back(angles_ref[6]);
        Serial.print(angles_ref[0]); Serial.print(' ');Serial.print(angles_ref[1]); Serial.print(' ');Serial.print(angles_ref[2]); Serial.print(' ');Serial.print(angles_ref[3]); Serial.print(' ');
        Serial.print(angles_ref[4]); Serial.print(' ');Serial.print(angles_ref[5]); Serial.print(' ');Serial.println(angles_ref[6]);
        delay(dt);
    }
    Input = Serial.parseInt();

    delay(1000);
}

//============== Setup ===========================================

void setup() {
  // Start Comunication with Servo Motors
  myssc.begin(9600);
  Serial.begin(9600);
  
  analogReference(EXTERNAL);  // Ligar porta AREF em 3.3V -> Aumenta resolução

  // Set all Angles to zero
  ang0.push_back(999);ang1.push_back(0.0);ang2.push_back(0.0);
  ang3.push_back(0.0);ang4.push_back(0.0);ang5.push_back(0.0);
  ang6.push_back(0.0);

  // Move Robot to Zero Position
  SetInitialPosition();
}

//=============== Loop ============================================

void loop() {
  // Feedback
  angles_feedback = Feedback();

  // Print Joint Angles
  for (k = 0; k < 7; k++){
     if (k == 6){
        Serial.println(angles_feedback[k]);
     }else{
        Serial.print(angles_feedback[k]);
        Serial.print(' ');
     }
  }

  delay(10);
}

