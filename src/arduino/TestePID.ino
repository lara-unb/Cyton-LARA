/* Universidade de Brasília
 * Laboratório de Automação e Robótica
 * Autor: De Hong Jung
 * Programa: Controle PID da posicao dos servos do manipulador Cyton Alpha 7D1G
 */
 
#include <SSC32.h>

#include <FilterDerivative.h>
#include <FilterOnePole.h>
#include <Filters.h>
#include <FilterTwoPole.h>
#include <FloatDefine.h>
#include <RunningStatistics.h>

SSC32 myssc = SSC32();

float value = 0.0;
float erro = 0.0, servoPosicaoMap = 0.0;

// Minimo e Maximo em Volts
float min_array[] = {2.72, 0.7, 2.7, 4.06, 2.7, 2.7, 2.75, 2.69};
float max_array[] = {0.75, 4.26, 0.75, 0.45, 0.63, 0.63, 0.6, 0.69};

// Minimo e Maximo do Servo
float min_array2[] = {2330, 2320, 2350, 800, 2370, 2300, 2370};
float max_array2[] = {720, 920, 730, 2100, 610, 600, 580};
float min_degree[] = {-90, -86, -90, -83, -90, -90,-90};
float max_degree[] = {90, 65, 90, 62, 90, 90, 90};

// Inputs
int servoPin = 1, servoPosicao;

float Setpoint, Input, Output;



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

PID myPID(-0.1, 0.005, 0.0005);

// filters out changes faster that 5 Hz.
float filterFrequency = 15.0;
// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );


void setup() {
  myssc.begin(9600);
  analogReference(EXTERNAL);  // Ligar porta AREF em 3.3V -> Aumenta resolução
  myPID.setSetPoint(0.0);

}

void loop() {
  // Leitura da posição do Servo
  value = analogRead(servoPin)*(5.0/1023.0);
  value = mapfloat(value, min_array[servoPin], max_array[servoPin], -90, 90);
  value = lowpassFilter.input( value );
  
  myPID.addNewSample(value);

  Output = myPID.process();

  // Comando de movimentacao de Servo
  servoPosicaoMap = mapfloat(Output, min_degree[servoPin], max_degree[servoPin], min_array2[servoPin], max_array2[servoPin]);
  myssc.servoMoveTime(servoPin, servoPosicaoMap, 700);

  Serial.print("Input: "); Serial.println(value); //Serial.print("  Erro: "); Serial.println(Output);

  delay(5);

}

// Funcao de Mapaeamento de variavel float
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
