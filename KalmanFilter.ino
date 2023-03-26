#define encA 2
#define encB 3
#define pul 4
#define dir 5
// Encoder: https://www.amazon.com/gp/product/B01MZ4V1XP/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1
volatile signed long pulse, prev_pulse = 0; 
volatile float theta, thetap, w, wp, e, ep = 0;  //theta, angular velocity, and error in terms of wp
float ang_vel;
unsigned long newTime;
unsigned long oldTime = 0;
unsigned long dt = 0;
float ppr = 600.00;

// Motor
float steptheta = 1.8;
const int SPR = 200; // steps per rotation
float motorspeed = 1000; // change speed here (revo/min)
float pulserate = motorspeed/steptheta*360/60; // rev/360 degree, minute to second conversion
float pwm = 1/pulserate*1000000; //second to micros conversion


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Encoder
  pinMode(encA,INPUT_PULLUP);
  pinMode(encB,INPUT_PULLUP);
  attachInterrupt(0,checkdirB,RISING);
  //attachInterrupt(1,checkdirA,RISING);
  Serial.println(pulse);
  // Motor
  pinMode(dir, OUTPUT);
  pinMode(pul, OUTPUT);
  digitalWrite(dir, HIGH); // initializes direction
}

void checkdirB() {
  if (digitalRead(encB)==HIGH) {
    pulse++; // Consider as forward
  }
  else {
    pulse--;
  }
}
void checkdirA() {
  if (digitalRead(encA)==HIGH) {
    pulse--; // Consider as forward
  }
  else {
    pulse++;
  }
}

// Initialize Paramters for Kalman Filter
float thetaM = pulse;
float Pm = 0.1;
float thetaP = thetaM;
float varV = 1;
float varW = 40;
float Pp = Pm + varW;

long kalmanFilter() {
  thetaP = thetaM;
  Pp = Pm + varW;
  thetaM = thetaP + Pp/(Pp+varW)*(pulse-thetaP);
  Pm = Pp - Pp*Pp/(Pp+varW);
  return thetaM; // this is actually the pulse
}

float Kp = 5;
float Ki = 1;
float Kd = 1;
int tidx = 100; // time interval for PID


void loop() {
  // put your main code here, to run repeatedly:
  if (prev_pulse != pulse) {
    theta = pulse*2*PI/ppr;
    newTime = millis();
    dt = newTime - oldTime;
    ang_vel = (theta-thetap)/dt;
    //thetaM = kalmanFilter();
    Serial.print("Pulse: "+(String)pulse);
    Serial.print(" theta: " +(String)theta);
    Serial.println(" AngVel: " +(String)ang_vel);

    }
  prev_pulse = pulse;
  thetap = theta;
  oldTime = newTime;
}
