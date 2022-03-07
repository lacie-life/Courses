#define in1 8          // phải
#define en1 3          //
#define in2 7         // trái
#define en2 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                 //

#define echo 12
#define trig 2
#include <Servo.h>
Servo my;

unsigned long cTime, pTime;

void setup()
{
  pinMode(in1, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en2, OUTPUT);
  my.attach(A1);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  my.write(0);
  Serial.begin(9600);
  pTime = millis();
}

int tinhkhoangcach()
{
  digitalWrite(trig, 0);
  delayMicroseconds(2);
  digitalWrite(trig, 1);
  delayMicroseconds(10);
  digitalWrite(trig, 0);
  unsigned long duration = pulseIn(echo, 1);
  return int(duration / 2.0 / 29.412);
}



void  dichuyen(int vantoc1, int vantoc2)
{
 
  digitalWrite(in1, HIGH);
  analogWrite(en1, 255 - vantoc1);
  digitalWrite(in2, HIGH);
  analogWrite(en2, 255 - vantoc2);
}


const double kp = 10;
const double ki = 0.025;
const double kd = 5;
const double target = 20;
const double delta=0.005;
int positionx = tinhkhoangcach();
double integral = 0;
double lastError = 0;
int controlvalue()
{
  double error = positionx - target;
  positionx = tinhkhoangcach();
  double convalue; 
  integral = integral + error*delta;
   convalue = kp * error + ki * integral ;
            
            convalue += kd * (error-lastError)/delta;
            lastError = error;
            //Serial.println(convalue);

  return int(convalue);
}

void bamvat()
{
  int vp = 50;
  int vt = 50;
  int convalue = controlvalue()-20;
  if (tinhkhoangcach() <=17 && tinhkhoangcach()>=13)
  {
    //vt = 50;
    dichuyen(50, 50);
  }
  if ( convalue > 0)
  {
  
    vp += convalue;
    
    if (vp > 255)
    {
      
      vp = 255;
      //v = constrain(v,75, 255);
    }
    dichuyen(vp, vt);
//    Serial.println(v);
  }
  else 
  {
    vt += abs(convalue);
    if (vt > 255)
    {
      vt = 255;
      
    }
    dichuyen(vp, vt);
    //Serial.println(v);
  }
  
}
unsigned long time_stop = 0;
unsigned long time_servo = 0;
  void loop()
  {
   cTime = millis();
   if(cTime - pTime >= 3000)
   {
    pTime = cTime;
    time_stop = millis();
    while (millis() - time_stop < 200)
    stops();
        //time_servo = millis();
    for (int k=0; k<=90; k++)
    
    {
    my.write(90);
    delay(5);
    }
    int distance = tinhkhoangcach(); 
    Serial.println(distance);
    if(distance >= 40)
    {
      turnleft();
      delay(800);
    }
    my.write(0);
    
   }
    bamvat();
    //dichuyen(LOW,160);
  }
  void stops()
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(en1,LOW);
    digitalWrite(en2,LOW); 
  }
  void turnleft()
  {
    digitalWrite(in2, HIGH);
    digitalWrite(en2, LOW);
    digitalWrite(in1, 0);
    digitalWrite(en1, 0);
  }
