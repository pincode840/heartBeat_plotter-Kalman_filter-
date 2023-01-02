#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;  // 센서와 상호 작용할 MAX30105 객체 생성

// Kalman 필터 객체 생성
float Q_angle = 0.01;  // 각도에 대한 공정 잡음 
float Q_bias = 0.03;  // 편향에 대한 프로세스 잡음 분산
float R_measure = 0.03;  // 측정 잡음 분산
float angle = 0;  // Kalman 필터에 의해 계산된 각도
float bias = 0;  // Kalman 필터에 의해 계산된 치우침
float rate;  // 필터링되지 각도 변화율
float P[2][2];  // 오차 공분산 행렬
float dt;


void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  // 센서 초기화
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    // 센서를 못찾으면 프로그램 중지
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  // 센서 설정
  byte ledBrightness = 0x1F;  // LED current: 0=Off to 255=50mA
  byte sampleAverage = 8;  // Sample average: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;  // LED mode: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 3200;  // Sample rate: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;  // Pulse width: 69, 118, 215, 411
  int adcRange = 4096;  // ADC range: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  dt = 0.01;
  // 초기 angle 및 bias 값으로 Kalman 필터를 초기화합니다.
  angle = particleSensor.getIR();
  bias = 0;
  float a = 0;


  // 오차 공분산 행렬 초기화
  P[0][0] = 0;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 0;
}

void loop() {
  // 센서에서 필터안된 IR 광도값을 읽음
  float sensorValue = particleSensor.getIR();

  // 각도의 변화율 계산
  rate = sensorValue - angle;

  // Kalman 필터를 사용하여 angle 및 bias 추정치를 업데이트
  angle += dt * rate;
  bias += dt * (rate - bias);

  // 새 오류 공분산 계산
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Kalman gain 계산
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // 측정된 값을 이용하여 angle 및 bias 추정치를 업데이트
  float y = sensorValue - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  // 오류 공분산 행렬 업데이트
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;



  float a = (angle + sensorValue)/2;
  

  // 필터링된 값과 필터링 안한 값 확인
  Serial.print(angle);
  Serial.print(",");
  //Serial.print(a);
  //Serial.print(",");
  Serial.println(sensorValue);
}
