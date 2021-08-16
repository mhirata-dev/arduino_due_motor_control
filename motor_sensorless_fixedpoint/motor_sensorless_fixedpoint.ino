// Sensorless control by fixed-point arithmetic
// created 2021-08-16
// Copyright (C) 2021 by Mitsuo Hirata

/* Connection
 * Arduino -> X-NUCLEO-IHM07M1 board
 * D08  -> CN10 22 (LED)
 * D09  -> CN10 13 (EN)
 * D11  -> CN7 1 (EN1)
 * D12  -> CN7 2 (EN2)
 * D13  -> CN7 3 (EN3)
 * D35  -> CN10 23 (IN1 PWM U)
 * D37  -> CN10 21 (IN2 PWM V)
 * D39  -> CN10 33 (IN3 PWM W)
 * A0   -> CN7 28 (Curr_fdbk1)
 * A1   -> CN7 36 (Curr_fdbk2)
 * A2   -> CN7 38 (Curr_fdbk3)
 * A6   -> VR1 (Control ON/OFF)
 * A7   -> VR2 (Speed)
 * 3.3V -> CN7 16 (3.3V)
 * GND  -> CN7 20 (GND)
 */

#include "mh_motorlib.h"
#include <math.h>

#define VCC 12
#define POLE_PAIR  7
#define AOFFSET (177*32768/360) // 177deg
#define IV_SCALE  1024
#define SC_SCALE  32768
#define rewrap(x) (((x + 16384)&0x7fff)-16384)
#define DUTY2VOLT (VCC*IV_SCALE/MAX_PWM_DUTY)
#define NUM_CURRENT_OFFSET  1000
#define PI_INT  16384
#define FS  20000
#define FF_MAX_RPM    2000
#define FF_ACCEL      100
#define FF_MAX_SPEED  (FF_MAX_RPM*POLE_PAIR*SC_SCALE/60/FS*FF_ACCEL)
#define FF_CURRENT    1500

// angle sensor
const int slaveSelectPin = 10; // D10

// serial buffer
char  buff[256];
const int sqrt23 = (int)(sqrt(2.0/3.0)*SC_SCALE);
const int cos23z = (int)(cos(2.0/3.0*PI)*sqrt(2.0/3.0)*SC_SCALE);
const int cos43z = (int)(cos(4.0/3.0*PI)*sqrt(2.0/3.0)*SC_SCALE);
const int sin23z = (int)(sin(2.0/3.0*PI)*sqrt(2.0/3.0)*SC_SCALE);
const int sin43z = (int)(sin(4.0/3.0*PI)*sqrt(2.0/3.0)*SC_SCALE);
const int adgain_and_ivscale = (int)(0.33*1.56*4096/3.3*SC_SCALE/IV_SCALE);
volatile int sinv, cosv;
volatile int theta = 0, theta_in, theta_delta, theta_err;
volatile int omega, omega_ref=0;
// Potentio meter
volatile int pota, potb;
// Current
volatile int ia, ib, ic, ia0, ib0, ic0, ia0_, ib0_, ic0_;
volatile int iu, iv, iw;
volatile int Ia, Ib;
volatile int Id, Iq, Id_ref, Iq_ref;
// Voltage
volatile int Va, Vb, Vd, Vq;
volatile int du, dv, dw;
// State of integrator
volatile int xid = 0, xiq = 0, xiw = 0;
// Variables for sensorless control
volatile int Ea, Eb, Vra, Vrb, theta_est;
// Index
volatile int i_count = 0;
volatile int k = 0;
// Data logging
#define N_DATA  500
volatile short data[N_DATA][7];
volatile int f_record = 0, mode = 0, nn = 0;
// Model parameter
const int Ra = (int)(1.0160*SC_SCALE);
const int La = (int)(2.7e-5*SC_SCALE); // 0.018*(3/2)
const int u_offset = (int)(0.4625*IV_SCALE);
// PLL Controller gain
const int iKp_pll = 28;
const int iKi_pll = 1611;
// Current control gain
const int  Ki_curr_d = 12000;
const int  Ki_curr_q = 12000;
const int iKp_curr = 3;
// Velocity control gain
const int Kp_velo = 5;

void setup() {
  // Angle sensor
  setupAngleSensor(slaveSelectPin);
  analogWriteResolution(12);
  // PWM and ADC
  setup_pwm();
  setup_adc();
  // Enabler
  pinMode(11, OUTPUT); // EN1
  pinMode(12, OUTPUT); // EN2
  pinMode(13, OUTPUT); // EN3
  pinMode(9, OUTPUT);  // Enabler (ON/OFF)
  pinMode(8, OUTPUT);  // LED
  pinMode(7, OUTPUT);  // Monitor
  digitalWrite(11, HIGH); // EN1
  digitalWrite(12, HIGH); // EN2
  digitalWrite(13, HIGH); // EN3
  digitalWrite(9, LOW); // Enarbler (ON/OFF)
  digitalWrite(8, LOW); // LED
  digitalWrite(7, LOW); // Monitor
  // Serial
  Serial.begin(115200);
}

void loop() {
  int i;
  if (f_record == 2){
    for (i=0; i<N_DATA; i++){
      sprintf(buff,"%d,%d,%d,%d,%d,%d,%d",data[i][0],data[i][1],data[i][2],data[i][3],data[i][4]-1000,data[i][5]-1000,data[i][6]*1000/191);
      Serial.println(buff);
    }
    f_record = 1;
    nn = 0;
  }
}

void updateControl()
{
  sincos16_t sc;

  digitalWrite(7,HIGH); // start control
  pota = getPot1Voltage(); // 12bit
  potb = getPot2Voltage(); // 12bit
  // Ident current offset
  if (mode == 0){
    // Control OFF
    digitalWrite(9, LOW); // Run
    digitalWrite(8, LOW); // LED
    f_record = 0;
    k = 0;
    xid = 0, xiq = 0, xiw = 0; // reset integrator
    if ( i_count < NUM_CURRENT_OFFSET ) {
      ia0_ += getIU();
      ib0_ += getIV();
      ic0_ += getIW();
      i_count++;
    } else if ( i_count == NUM_CURRENT_OFFSET ) {
      ia0 = ia0_ / NUM_CURRENT_OFFSET;
      ib0 = ib0_ / NUM_CURRENT_OFFSET;
      ic0 = ic0_ / NUM_CURRENT_OFFSET;
      i_count++;
    } else {
      ia0_ = 0; ib0_ = 0; ic0_ = 0;
      i_count = 0;
    }
    // Control ON
    if (pota > 200){
      mode = 1;
      k = 0;
    }
  }
  if ( mode > 0) {
    // Control ON
    digitalWrite(9, HIGH); // Run
    digitalWrite(8, HIGH); // LED
    // Control OFF
    if ( pota < 100 ){
      mode = 0;
    }
  }
  // Get angle (readAngle return 14bit (2^14 = 0x3fff = 16384 value)
  // （モニター用のため制御には使用していない）
  theta_in = rewrap(readAngle(slaveSelectPin)*2*7 + AOFFSET);
  // Mode 1 (Accerelate)
  if ( mode == 1 ){
    k++;
    theta_delta = k/FF_ACCEL;
    theta += theta_delta;
    if ( k > FF_MAX_SPEED){
      mode = 2;
      f_record = 1;
      nn = 0;
      xiw = 0;
      // センサレスへ移行するときのオフセットを取り除く
      // theta -= 8192; // (-90deg)
    }
    Id_ref = 0;
    Iq_ref = FF_CURRENT;
  }
  // Mode 2 (Constant speed)
  if ( mode == 2 ){
    // PLL
    theta_err = rewrap(theta_est - theta);
    // Integrator
    xiw += theta_err;
    // PI controll
    theta_delta = xiw / iKi_pll + theta_err / iKp_pll;
    theta += theta_delta + FF_MAX_SPEED/FF_ACCEL;
    // velocity FB
    omega_ref = (potb - 45);
    if (omega_ref > 2800){
      omega_ref = 2800; // 約15000rpm (max速度はこれにFF_MAX_SPEED/FF_ACCELが加わる)
    }
    // P control
    // theta_deltaに対する目標値になっているので注意(2000rpm基準)
    Iq_ref = (omega_ref - theta_delta)*Kp_velo;
    Id_ref = 0;
  }
  // sin and cos
  theta = rewrap(theta);
  sc   = sincost(theta);
  sinv = sc.sinv;
  cosv = sc.cosv;
  // Get currents
  ia = getIU() - ia0; // 12bit (4096)
  ib = getIV() - ib0; // 12bit (4096)
  ic = getIW() - ic0; // 12bit (4096)
  //
  if (du < dv && du < dw) {
    iu =  ib + ic;
    iv = -ib;
    iw = -ic;
  } else if (dv < du && dv < dw) {
    iu = -ia;
    iv =  ia + ic;
    iw = -ic;
  } else {
    iu = -ia;
    iv = -ib;
    iw =  ia + ib;
  }
  // iu,iv,iw -> Ia, Ib
  Ia = (sqrt23 * iu + cos23z * iv + cos43z * iw) / adgain_and_ivscale;
  Ib = (              sin23z * iv + sin43z * iw) / adgain_and_ivscale;  
  // Ia,Ib -> Id, Iq
  Id = ( cosv*Ia + sinv*Ib) / SC_SCALE;
  Iq = (-sinv*Ia + cosv*Ib) / SC_SCALE;
  // Id controller
  xid += Id_ref - Id;
  Vd = Ki_curr_d*xid/FS + (Id_ref - Id)/iKp_curr;
  // Iq controller
  xiq += Iq_ref - Iq;
  Vq = Ki_curr_q*xiq/FS + (Iq_ref - Iq)/iKp_curr;
  //
  Va = (cosv * Vd - sinv * Vq) / SC_SCALE;
  Vb = (sinv * Vd + cosv * Vq) / SC_SCALE;
  //
  du = -(Va * sqrt23              ) / SC_SCALE / DUTY2VOLT;
  dv = -(Va * cos23z + Vb * sin23z) / SC_SCALE / DUTY2VOLT;
  dw = -(Va * cos43z + Vb * sin43z) / SC_SCALE / DUTY2VOLT;
  //
  pwmOUT(du,dv,dw);

  // For sensorless control
  Vra = Ra*Ia/SC_SCALE;
  Vrb = Ra*Ib/SC_SCALE;
  // Induced voltage
  Ea = -(Va - Vra);
  Eb =  (Vb - Vrb);
  // Estimated angle
  theta_est = arctan(Ea,Eb);
  // logger
  if (f_record == 1){
    data[nn][0] = Id_ref;
    data[nn][1] = Id;
    data[nn][2] = Iq_ref;
    data[nn][3] = Iq;
    data[nn][4] = theta/32;
    data[nn][5] = theta_in/32;
    data[nn][6] = (theta_delta + FF_MAX_SPEED/FF_ACCEL);
    nn++;
    if (nn >= N_DATA){
      nn = 0;
      f_record = 2;
    }
  }
  digitalWrite(7,LOW);
}
// EOF of motor_sensorless_fixedpoint_*.cpp
