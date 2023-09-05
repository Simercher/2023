#define R_A 3
#define R_B 2
#define L_A 21
#define L_B 20
//#define L_PWM 4
//#define R_PWM 5
//#define R_HIGH 22
//#define R_LOW 24
//#define L_HIGH 26
//#define L_LOW 28
#define TRIG 52
#define ECHO 50

int R_encoder = 0;
int L_encoder = 0;
int pre_time, current_time = millis();
int d_time = 50;
double r_error = 0, l_error = 0;
double r_pre_error = 0, l_pre_error = 0;
double r_kp = 0.5, l_kp = 0.5;
double r_ki = 0, l_ki = 0;
double r_kd = 80, l_kd = 80;
double P, I, D, PID, r_pwm = 0, l_pwm = 0;
const double MAX_SPEED = 75;//1700 / (1000 / d_time);
double r_velocity = 75, l_velocity = 75;
String msg = "";

int countEncoder();
int R_mutiSignal();
//int R_calPID();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(TRIG, OUTPUT);        // 定義輸入及輸出 
  pinMode(ECHO, INPUT);
  pinMode(R_A, INPUT);
  pinMode(R_B, INPUT);
  pinMode(L_A, INPUT);
  pinMode(L_B, INPUT);
}

void loop() {
  current_time = millis();
  countEncoder();
  if (current_time - pre_time > d_time) {
    msg = String(R_encoder) + " " + String(L_encoder) + "\n";
    Serial.print(msg);
    R_encoder = 0;
    L_encoder = 0;
    pre_time = current_time;
  }
} 

void setRV() {
  PID = map(PID, 0, MAX_SPEED, 0, 255);
  r_pwm += PID;
  if (r_pwm > 255) {
    r_pwm = 255;
  } else if (r_pwm < 0) {
    r_pwm = 0;
  }
}

void setLV() {
  PID = map(PID, 0, MAX_SPEED, 0, 255);
  l_pwm += PID;
  if (l_pwm > 255) {
    l_pwm = 255;
  } else if (l_pwm < 0) {
    l_pwm = 0;
  }
}

int R_calPID() {
  r_error = r_velocity - R_encoder;  //Current position - target position (or setpoint)
  Serial.print("R_velocity: ");
  Serial.print(r_velocity);
  Serial.print(" R_Encoder ");
  Serial.println(R_encoder);
//  Serial.print(" R_PWM: ");
//  Serial.println(r_pwm);
  P = r_kp * r_error;
  I += r_ki * (r_error * d_time);
  D = r_kd * ((r_error - r_pre_error) / d_time);
  r_pre_error = r_error;

  return P + I + D;
}

int L_calPID() {
  l_error = l_velocity - L_encoder;
//  Serial.println(L_encoder);
//  Serial.print("L_velocity: ");
//  Serial.print(l_velocity);
//  Serial.print(" L_Encoder ");
//  Serial.println(L_encoder);
//  Serial.print(" L_PWM: ");
//  Serial.println(l_pwm);
  P = l_kp * l_error;
//  Serial.print(l_error);
  I += l_ki * (l_error * d_time);
  D = l_kd * ((l_error - l_pre_error) / d_time);
  l_pre_error = l_error;

  return P + I + D;
}

int countEncoder() {
  attachInterrupt(digitalPinToInterrupt(R_B), R_mutiSignal, RISING);
  attachInterrupt(digitalPinToInterrupt(L_B), L_mutiSignal, RISING);
  return R_encoder;
}

int R_mutiSignal() {
//  Serial.println("test");
  if (digitalRead(R_B) * digitalRead(R_A) == 0) {
    R_encoder++;
  } else {
    R_encoder--;
  }
}

int L_mutiSignal() {
  if (digitalRead(L_B) * digitalRead(L_A) == 0) {
    L_encoder++;
  } else {
    L_encoder--;
  }
}
