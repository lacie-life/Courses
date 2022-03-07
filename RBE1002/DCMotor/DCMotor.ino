#define in1 8 // chân in động cơ 1 high xoay phải, low xoay trái //DC phải
#define en1 3 // chân en động cơ 1, tốc độ
#define in2 7 // chân in động cơ 2 high xoay trái, low xoay phải
#define en2 6 // chân en động cơ 2, tốc độ  

int speedMotor = 100;     

void setup() {
  Serial.begin(9600);  
   
  pinMode(in1, OUTPUT); //set output điều hướng motor 1
  pinMode(en1, OUTPUT); //set output chỉnh tốc độ mortor 1
  pinMode(in2, OUTPUT); //set output điều hướng motor 2
  pinMode(en2, OUTPUT); //set output chỉnh tốc độ mortor 2
}

void loop() {

  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(en1,speedMotor);  
  analogWrite(en2,speedMotor); 

  Serial.println("forward");
                                      
  delay(1000);

  analogWrite(en1,0);  
  analogWrite(en2,0); 
  
  Serial.println("stop");

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(en1,speedMotor);  
  analogWrite(en2,speedMotor); 

  Serial.println("backward");

  delay(1000);

  analogWrite(en1,0);  
  analogWrite(en2,0); 
  
  Serial.println("stop");

  delay(1000);
}
