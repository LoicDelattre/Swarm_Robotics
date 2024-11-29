#include <MeMCore.h>

MeDCMotor motor1(M1);  // Motor on port M1
MeDCMotor motor2(M2);  // Motor on port M2


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  //(Serial.println("Skibidiing forward"); motor1.run(-200); motor2.run(200); delay(1000);  motor1.run(0);  motor2.run(0); delay(2000);)

  Serial.println("Skibidiing left");
  motor1.run(100);
  motor2.run(100);
  delay(1000);
  motor1.run(0);
  motor2.run(0);
  delay(2000);

  Serial.println("Skibidiing right");
  motor1.run(-101.250); // the difference of 2 % is made to balance out the fact that the robot turns more easily in one direction when compared to the other one
  motor2.run(-101.250);
  delay(1000);
  motor1.run(0);
  motor2.run(0);
  delay(2000);

  //Serial.println("Skibidiing Backward");motor1.run(100);motor2.run(-100);delay(1000);motor1.run(0);motor2.run(0);delay(2000);
}
