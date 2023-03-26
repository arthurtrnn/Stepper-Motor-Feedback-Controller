#include <AccelStepper.h>
#define encA 2
#define encB 3
#define pul 4
#define dir 5
AccelStepper hairRobot(AccelStepper::DRIVER ,pul,dir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
long baudrate = 115200; 
void setup() {  
  Serial.begin(baudrate);
  // Motor
   hairRobot.setAcceleration(1000);
   hairRobot.setMaxSpeed(10000);
   hairRobot.setSpeed(20000);    
   // Encoder
   pinMode(encA, INPUT_PULLUP);
   pinMode(encB, INPUT_PULLUP);
   attachInterrupt(0,checkdirB,RISING); // Whenever a 0->5V change occurs, run checkDirB function
}

volatile signed long p, pp = 0;
volatile float theta, thetap, w, wp, sps, e, esum, ep = 0; // motor angle, velocity, error, and previous
unsigned long newTime;
unsigned long oldTime, dt = 0;
float ppr = 600; // pulse per revolution (encoder)
float spr = 200; // steps per revolution (motor)

void checkdirB() {
  if (digitalRead(encB)==HIGH) {
    p++; // Consider as forward
  }
  else {
    p--;
  }
}
float Kp = 0.182;
float Ki =  0.000000000000029;
//float Ki = 0.00000000000001;
float Kd = 100;
float spsD = 1000; //steps per second
float w_pid = spsD;

void velPID(){
  e = spsD - sps;
  esum += e;
  w_pid = Kp*e + Ki*esum*dt*1000 + Kd*(e-ep)/(dt*1000); //ms to s conversion
  hairRobot.setSpeed(w_pid);
  ep = e;
}

void sensorUpdate() {
  theta = p*360/ppr;        // Get the angle(degrees) from the number of pulses
  w = (theta - thetap)/dt;   // Instant velocity from previous measurement (deg/ms)
  sps = w*ppr/360*1000;           // Steps per second conversion
  //Serial.print(p); Serial.print("  "); Serial.print(theta); Serial.print(" "); Serial.print(thetap);
  //Serial.print(w); Serial.print("  "); 
  Serial.println(sps);
  thetap = theta;
}
void loop(){ 
  velPID();
  hairRobot.runSpeed();
  dt = millis() - oldTime;
  if (millis() - oldTime >= 100) { //Sampling Time - Every 100 ms
    sensorUpdate();
    oldTime = millis();
  }
  //e = wd - wp;
  //esum += e;
  //w_pid = Kp*e + Ki*esum*dt*1000; // + Kd*(e-ep)/(dt*1000); //ms to s conversion
  //Serial.print("e:" + (String)e); Serial.print("pid: " + (String)w_pid);
  //Serial.print("w:" + (String)w); Serial.println("wpid: " + (String)(w_pid));

  //if (pp!=p) {
   // newTime = millis();
   // theta = p*2*PI/ppr;
   // dt = newTime - oldTime;
   // w = (theta-thetap)/dt*360/(2*PI)/1.8;
    //Serial.print("theta:" + (String)theta); Serial.println("wpid: " + (String)(w_pid));
   //pp = p;
   //thetap = theta;
   //oldTime = newTime;
   //wp = w;   
   //Serial.print(p," ");
   //Serial.print(dt," ");
   //Serial.println(wp," "); 
   
   //}
}
