#include <Servo.h>

Servo servo_1;
Servo servo_2;
int val = 0;       // variable to store the data from the serial port

void setup() {
  servo_1.attach(10);  // asignamos el pin 9 al servo.
  servo_2.attach(9);
  Serial.begin(9600);
  servo_2.write(90);  // enviamos el valor escalado al servo.
  servo_1.write(90);  // enviamos el valor escalado al servo.
}

void loop() {
  val = Serial.read();      // read theq serial port
  int n=0;
  if (val>0 && val<=90) {
      n=2*val;
      Serial.print("1 ");
      Serial.println(n);
      servo_2.write(n);
      servo_2.write(n);
      }
    else if(val>90 && val<=180){
      n=2*val-180;
      Serial.print("2 ");
      Serial.println(n);
      servo_1.write(180-n);
      servo_1.write(180-n);
    } 
    delay(5);
}
