//Nhóm 3
//Nguyễn Huyền Trang
//Phàn Huyền Trang
//Lê Thị Trang
//Trương Thị Huyền Mai

//Servo
#include <Servo.h>
Servo myservo;

//siêu âm
const int trig = 2;
const int echo = 12;

int speed;


void setup() {
  Serial.begin(9600);
  
  pinMode (8, OUTPUT);      //direction   // left
  pinMode (3, OUTPUT);      //PWM
  
  pinMode (7, OUTPUT);      //direction   // right
  pinMode (6, OUTPUT);      //PWM



  pinMode (trig, OUTPUT);   //Siêu âm
  pinMode (echo, INPUT);

  myservo.attach(A1);       //Servo
}

void left_wheel_stop()
{
  digitalWrite (7, LOW);
  digitalWrite (6, LOW);
}

void left_wheel_control(int speed)
{
  if (speed > 0){
    digitalWrite(7, HIGH);
    analogWrite (6, speed);
  }
  else{
    digitalWrite(7, LOW);
    analogWrite (6, - speed);
  }
}

void right_wheel_stop()
{
  digitalWrite (8, LOW);
  digitalWrite (3, LOW);
}

void right_wheel_control(int speed)
{
  if (speed > 0){
    digitalWrite(8, HIGH);
    analogWrite (3, speed);
  }
  else
  {
    digitalWrite(8, LOW);
    analogWrite (3, - speed);
  }
}

void KC_stop()
{
  left_wheel_stop();
  right_wheel_stop();
}

void KC_turn(int dir)
{ 
    if (dir == 1)
    {
      left_wheel_control(100);
      right_wheel_stop();
    }
    if (dir == -1)
    {
      left_wheel_stop();
      right_wheel_control(100);
    }
  delay(500);
}


float prev_distanceCm = 0;
float get_distance()
{
  long duration;
  float distanceCm;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  duration = pulseIn(echo, HIGH, 5000);
 
  // tính khoảng cách
  distanceCm = (float)duration / 29.1 / 2;
  if(distanceCm == 0)
  {
    if (prev_distanceCm == 0) 
    prev_distanceCm = 20; 
    distanceCm = prev_distanceCm;
  } 
  prev_distanceCm = distanceCm;
  return distanceCm;
}

//PID
const int setpoint = 20;
float sum_err = 0;
float prev_err = setpoint - get_distance();
const int original_speed = 100;
void PID()
{
  
  int Kp = 15;
  int Ki = 0.5;
  int Kd = 0.8;
  float dt = 0.01;

  float err = setpoint - get_distance();

  sum_err += err * dt; 
  float P = Kp * err;
  float I = Ki * sum_err;
  float D = Kd * (err - prev_err)/dt;
  prev_err = err;

  float w = P + I + D;
  int v_left = original_speed + (int)w;
  int v_right = original_speed - (int)w;
  
  if(v_right > 255) v_right = 255;
  if(v_right < -255) v_right = -255;
  if(v_left > 255) v_left = 255;
  if(v_left < -255) v_left = -255;
  
  wheel_control(fabs(v_left), fabs(v_right));
  
  delay(10);
}

void wheel_control(int v_left, int v_right)
{
  if (get_distance - setpoint > 0) // bám tường phải , nếu khoảng cách > setpoint , thì xe quay trái
  {
    digitalWrite(8, HIGH);
    analogWrite (3, v_left);
    digitalWrite(7, HIGH);
    analogWrite (6, v_right);
  }
  else
  {
    digitalWrite(8, HIGH);
    analogWrite (3, v_right);
    digitalWrite(7, HIGH);
    analogWrite (6, v_left);
  }
}

int write_servo()
{
  float d0, d1, d2;
    myservo.write(0);
    d1 = get_distance();
  delay(700);
    myservo.write(180);
    d2 = get_distance();
  delay(700);
    myservo.write(90);
    d0 = get_distance();
  delay(700);
  if(d1<40)
  {
    myservo.write(0);
    return 0;
  }
  if(d2<40)
  {
    myservo.write(180);
    return 180;
  }
  if(d0<30)
  {
    return 90;
  }
}

int t = 1, c = 1;
void loop()
{
    int pos = write_servo();

      myservo.write(pos);
      delay(100);
    
      PID();
      delay(1200);  
   
      KC_stop();
      myservo.write(90);
      int d0 = get_distance();
      delay(100);
    if (d0<28)
    { 
      if (pos == 180)
        KC_turn(-1);
      else if (pos == 0)
        KC_turn(1); 
    }
}
