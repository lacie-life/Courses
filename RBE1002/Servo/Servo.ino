#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  myservo.attach(A1);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  myservo.write(0);                  // sets the servo position according to the scaled value 
  delay(2000);                           // waits for the servo to get there 
  myservo.write(180);
  delay(2000);                           // waits for the servo to get there
}
