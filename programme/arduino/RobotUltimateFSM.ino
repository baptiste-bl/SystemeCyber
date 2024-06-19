#include "MeMegaPi.h"
#include <Wire.h>
#include <TimerFive.h>

#define BELT_PITCH 2.032 // distance between teeth
#define NTEETH 90.0 // number of teeth 

#define RPM_2_MMS BELT_PITCH*NTEETH/60.0 // conversion of the speed from rpm to mm/s

#define VOLTS_2_PWM 255.0/12.0 // conversion voltage to pwm [-255 255]
#define MAX_VOLTAGE 9 // max allowed voltage in V
#define PI 3.1415926535897932384626433832795
#define KT 0.2

#define Ts 5000 // Sampling rate in µs

float Te = Ts / 1000000.0; // Sampling time in seconds
float taux = 0.016;
float k = 52.98;
float a = -1 / taux;
float b = k / taux;
float Lbis[3] = {237.5, 15117, 300.8};
float Kb = 100;
float kI = -1.769535673839024;
float kx = 0.028312570781425;
float e1 = 0;
float e2 = 0;
float x =0;
float y=0;
float d01 = 0;
float v01 = 0;
float p01 = 0;
float d02 = 0;
float v02 = 0;
float p02 = 0;
float xi01 = 0;
float xi02 = 0;

volatile float angle = 0;  // actual heading in deg
volatile float speed1 = 0; // actual speed in mm/s
volatile float speed2 = 0;
volatile float position1 = 0; // actual position in mm
volatile float position2 = 0;
volatile float refAngle = 0;
volatile float angle_point = 0;

volatile float ref1 = 0; // actual references from serial
volatile float ref2 = 0;
volatile float refX = 0; // actual references from serial
volatile float refY = 0;
volatile float posX = 0; // actual references from serial
volatile float posY = 0;
float u1 = 0; // control signals in V
float u2 = 0;
volatile float vxref = 0; // actual references from serial
volatile float vyref = 0;
volatile long compTime = 0; // actual computation time of the critical loop
volatile short overrun = 0;

MeGyro gyro; // gyroscope object instanciation
MeEncoderOnBoard Encoder_1(SLOT1); // motor with encoder object instanciation
MeEncoderOnBoard Encoder_2(SLOT2);

void Update5ms()
{
  UpdateSensors();
  UpdateControl();
  UpdateActuators();
}

void UpdateSensors() {
  float angle_anc=angle;
  gyro.update(); // update the gyroscope state
  angle = gyro.getAngleZ(); // get the estimated heading in deg
  angle_point=(angle-angle_anc)/Te;
  Encoder_1.loop(); // update the encoders state
  Encoder_2.loop();
  
  speed1 = Encoder_1.getCurrentSpeed() * RPM_2_MMS; // compute the speed in mm/s
  speed2 = Encoder_2.getCurrentSpeed() * RPM_2_MMS;  

  position1 = Encoder_1.getCurPos() * BELT_PITCH * NTEETH / 360.0f; // compute the position in mm
  position2 = Encoder_2.getCurPos() * BELT_PITCH * NTEETH / 360.0f;  
}

float calcU(float u) {
  if (u > MAX_VOLTAGE)
    return MAX_VOLTAGE;
  else if (u < -MAX_VOLTAGE)
    return -MAX_VOLTAGE;
  return u;
}

void UpdateControl()
{
  float vxref_pres=vxref;
  float vyref_pres=vyref;

  vxref = KT * (refX -x );
  vyref = KT * (refY -y );

  x += (vxref+vxref_pres)/2 *Te;
  y += (vyref+vyref_pres)/2 *Te;



  float l = 175/2 * 85 /100;
  float L = 140;
  float refSpeedChenille =  cos(angle * PI / 180)*vxref + sin(angle * PI / 180)*vyref;
  refAngle = (-sin(angle * PI / 180)/L)*vxref + (cos(angle * PI / 180)/L)*vyref;
  
  // Calcul de la différence de vitesse de référence pour les chenilles
  float refSpeedDifference = (refAngle) * l;
  
  // Mise à jour des références de vitesse pour les moteurs
  ref1 = refSpeedChenille - refSpeedDifference;
  ref2 = refSpeedChenille + refSpeedDifference;

  // Update FSM for motor 1
  e1 = position1 - d01;
  float d01temp = d01 + Te * (v01 + e1 * Lbis[0]);
  float v01temp = v01 + Te * (a * v01 + b * u1 + b * p01 + Lbis[1] * e1);
  float p01temp = p01 + Te * Lbis[2] * e1;

  d01 = d01temp;
  v01 = v01temp;
  p01 = p01temp;

  // Update FSM for motor 2
  e2 = position2 - d02;
  float d02temp = d02 + Te * (v02 + e2 * Lbis[0]);
  float v02temp = v02 + Te * (a * v02 + b * u2 + b * p02 + Lbis[1] * e2);
  float p02temp = p02 + Te * Lbis[2] * e2;

  d02 = d02temp;
  v02 = v02temp;
  p02 = p02temp;

  // Update integrators and control signals
  float xi01temp = xi01 + Te * (-ref1 - v01 + Kb * (calcU(u1) - u1));
  float u10 = -kI * xi01 - kx * v01;

  float xi02temp = xi02 + Te * (ref2 - v02 + Kb * (calcU(u2) - u2));
  float u20 = -kI * xi02 - kx * v02;

  xi01 = xi01temp;
  xi02 = xi02temp;
  u1 = u10;
  u2 = u20;
}

void UpdateActuators() {
  setMotorsVoltage(u1, u2); // set the voltages
}

void Timer5ISR() {
  static char executing = 0; // set to 1 when the update function is running
  if (executing) {
    overrun = 1;
    return;
  } else executing = 1; // if already running => overrun

  interrupts(); // enable the interrupts during the execution of this loop
  long startTime = micros();
  
  Update5ms();
  
  compTime = micros() - startTime;
  executing = 0;
}

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
  }
}

void setupMotors() {
  Encoder_1.setPulse(8);
  Encoder_1.setRatio(46);
  Encoder_2.setPulse(8);
  Encoder_2.setRatio(46);
  
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  // Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void setMotorsVoltage(float voltage1, float voltage2) {
  Encoder_1.setMotorPwm(constrain(voltage1, -MAX_VOLTAGE, MAX_VOLTAGE) * VOLTS_2_PWM);
  Encoder_2.setMotorPwm(constrain(voltage2, -MAX_VOLTAGE, MAX_VOLTAGE) * VOLTS_2_PWM);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  gyro.begin();
  Wire.setClock(400000);
  setupMotors();
  Timer5.initialize(Ts);
  Timer5.attachInterrupt(Timer5ISR);  
}

void loop() {
  static float posX = 3000;
  static float posY = 2000;
  noInterrupts();
  refX = posX;
  refY = posY;
  float angleCopy = angle;
  float angle_pointCopy = angle_point;
  float speed1Copy = speed1;
  float v0Copy = v02;
  float speed2Copy = speed2;
  float position1Copy = position1;
  float position2Copy = position2;
  long compTimeCopy = compTime;
  interrupts();
  
  // Serial.print(" compTime: ");
  // Serial.print(compTimeCopy);

  // Serial.print(" overrun: ");
  // Serial.print(overrun);
  
  // Serial.print(" speed1: ");
  // Serial.print(speed1Copy, 2);

  // Serial.print(" v0: ");
  // Serial.print(v0Copy, 2);

  // Serial.print(" speed2: ");
  // Serial.print(speed2Copy, 2);
  
  // Serial.print(" position1: ");
  // Serial.print(position1Copy, 2);
  
  // Serial.print(" position2: ");
  // Serial.print(position2Copy, 2);

  //Serial.print(" angle: ");
  //Serial.print(angleCopy, 2);

 //Serial.print(" angle_point: ");
  //Serial.print(angle_pointCopy, 2);

  // Serial.print(" ref1: ");
  // Serial.print(ref1, 2);

  // Serial.print(" ref2: ");
  // Serial.print(ref2, 2);
  
  Serial.println();

  if (Serial.available()) {
    posX = Serial.parseFloat();
    posY = Serial.parseFloat();
  }
  delay(10);
}
