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

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#define LED 13
#define TIME_BLINK 200

using namespace std;

namespace std {
  ohserialstream cout(Serial);
}

//=========================== Variaveis =================================

float value = 0.0;
float erro = 0.0, servoPosicaoMap = 0.0;
int count, k, duration;
vector <float> angles_ref;
vector <float> angles_feedback;
double joint_angles[7];
vector <float> ang0;vector <float> ang1;vector <float> ang2;vector <float> ang3;vector <float> ang4;vector <float> ang5;vector <float> ang6;
vector <float> ang;

// Minimo e Maximo em Volts
const float min_array[] = {2.72, 0.7, 2.7, 4.06, 2.7, 2.7, 2.75, 2.69};
const float max_array[] = {0.75, 4.26, 0.75, 0.45, 0.63, 0.63, 0.6, 0.69};

// Minimo e Maximo do Servo
const float min_array2[] = {2330, 2320, 2350, 800, 2370, 2300, 2370};
const float max_array2[] = {720, 920, 730, 2100, 610, 600, 580};
const float min_degree[] = {-90,-86, -90, -83, -90, -90,-90};
const float max_degree[] = {90, 65, 90, 62, 90, 90, 90};

// ROS
ros::NodeHandle  nh;
sensor_msgs::JointState joint_msg;
ros::Publisher arduinoControler("/Cython/jointAngles", &joint_msg);

// Servo Motors Controler
SSC32 myssc = SSC32();

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

//=============== Funções =========================================

// Funcao de Mapaeamento de variavel float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
 
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

  myssc.endGroupCommand();
  
  delay(2000);

}

/**
 * Debug Function - Blink Led 'n' times
 * @author Rafael
 */
void blink(int n){
  for(k=(n>=0)?0:n;k<n;k++){
    digitalWrite(LED,HIGH);
    delay(TIME_BLINK);
    digitalWrite(LED,LOW);
    delay(TIME_BLINK);
  }
}

//============== Setup ===========================================

void setup() {
  // Set LED for debug
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
  
  // Start Comunication with Servo Motors
  myssc.begin(9600);
  Serial1.setTimeout(4000);
  
  //analogReference(EXTERNAL);  // Ligar porta AREF em 3.3V -> Aumenta resolução

  // Move Robot to Zero Position
  SetInitialPosition();
  angles_feedback = Feedback();

  blink(2);

  // Start ROS Node
  nh.initNode();
  nh.advertise(arduinoControler);
  nh.spinOnce();

  joint_msg.position_length = 7;
  joint_msg.st_position = 0;
  joint_msg.position = &angles_feedback[0];

  blink(3);
}

//=============== Loop ============================================

void loop() {
  // Feedback
  angles_feedback = Feedback();

  joint_msg.position = &angles_feedback[0];

  // Send Msg to ROS
  arduinoControler.publish( &joint_msg );
  nh.spinOnce();

  delay(10);
}

