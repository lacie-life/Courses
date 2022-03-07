#define in1 8 // chân in động cơ 1 high xoay phải, low xoay trái //DC phải
#define en1 3 // chân en động cơ 1, tốc độ
#define in2 7 // chân in động cơ 2 high xoay trái, low xoay phải
#define en2 6 // chân en động cơ 2, tốc độ

#define trig A2 //chân phát sóng siêu âm
#define echo A3 //chân thu sóng siêu âm

#include <Servo.h>
Servo myservo;

void setup()
{
  pinMode(in1, OUTPUT); //set output điều hướng motor 1
  pinMode(en1, OUTPUT); //set output chỉnh tốc độ mortor 1
  pinMode(in2, OUTPUT); //set output điều hướng motor 2
  pinMode(en2, OUTPUT); //set output chỉnh tốc độ mortor 2

  pinMode(trig, OUTPUT); //set output chân phát
  pinMode(echo, INPUT);  //set input chân thu

  myservo.attach(A1); //set servo
  myservo.write(180); //quay servo về góc 180

  //  Serial.begin(9600);
}

void move_(int in, int x, int en, int y) //truyền hướng và vận tốc cho từng bánh
{
  digitalWrite(in, x);
  analogWrite(en, 255 - y);
}

void moveto(int in, int en, int v) //điều hướng cho bánh
{
  move_(in, HIGH, en, v);

}

void dichuyen(double v1, double v2) //di chuyển bánh
{
  ktV(&v1, &v2); //kiểm tra chặn xung
  moveto(in1, en1, (int)v1);
  moveto(in2, en2, (int)(v2));
}

double distanceCm = 20; //khởi tạo giá trị khoảng cách
int khoang_cach()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH);

  if (duration / 2.0 / 29.412 != 0) //khi kc!=0, nhận giá trị khoảng cách(kc) mới
  { // ngược lại sẽ giữ nguyên giá trị kc cũ
    distanceCm = duration / 2.0 / 29.412;
  }

  return distanceCm;
}

const double _target = 20;   // giá trị mong muốn, cm
double _position = 0;        // giá trị thực, cm
const double deltaT = 0.005; // độ trễ code, second
int kt_w = 1;                //lưu giá trị kiểm ktra 0:bỏ qua, 1:lần đầu

double PID_(double error, double *integral, double *lastError, double Kp, double Ki, double Kd, int *kt) //PID tổng quát
{
  double P = Kp * error;

  *integral += error * deltaT;
  double I = Ki * *integral;

  double D;
  if (*kt != 1) //trong lần thực hiện đầu tiên, bộ D không hoạt động
  {
    D = Kd * (error - *lastError) / deltaT;
    *lastError = error;
  }
  *kt = 0;

  return P + I + D;
}

double PID_w() //PID vật tốc góc
{
  double Kp = 10;
  double Ki = 0.005;
  double Kd = 5;
  double error = _position - _target; //lỗi
  static double integral;             //tổng lỗi
  if (kt_w)
  {
    integral = 0;
  }
  static double lastError; //lỗi cũ

  return PID_(error, &integral, &lastError, Kp, Ki, Kd, &kt_w);
}

void ktraV(double *v) //đưa v về giới hạn xung, 75<=v<=255
{
  int dau = *v / abs(*v);           //dấu của v, -1 or 1
  *v = constrain(abs(*v), 80, 255); //|v|>=80 để đảm bảo xe luôn đủ xung di chuyển
  *v = dau * *v;                    //trả lại dấu cho v
}

void ktV(double *v1, double *v2)
{
  ktraV(v1);
  ktraV(v2);
}

void ktra_vuong()
{
  dichuyen(0, 0);    //di chuyển bánh
  delay(50);
  myservo.write(90); //quay servo về góc 90
  delay(1000);
  double _position_v = khoang_cach();
  if (_position_v <= 20)
  {
    dichuyen(-200, 200); //di chuyển bánh
    delay(400);
  }
}

int time_t = millis();
void loop()
{
  double v1 = 50;
  double v2 = 50;
  _position = khoang_cach(); //tính khoảng cách thực
  int pid = PID_w() - 20;

  if (_position >= 17 && _position <= 23)
  {
    dichuyen(v1, v2);
  }

  if (pid > 0)
  {
    v1 += pid;
    dichuyen(v1, v2); //di chuyển bánh
  }
  else
  {
    v2 += abs(pid);
    dichuyen(v1, v2); //di chuyển bánh
  }

  if (millis() - time_t > 4000)
  {
    ktra_vuong();
    time_t = millis();
    dichuyen(150, 150);     //di chuyển bánh
    delay(50);
    myservo.write(180); //quay servo về góc 180
    delay(1000);
  }
}
