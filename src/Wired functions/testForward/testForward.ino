#include <MeMCore.h>

MeDCMotor motor1(M1);  // Motor on port M1
MeDCMotor motor2(M2);  // Motor on port M2
int i = 0 ;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(i=0;i<2;i++){
  Serial.println("moving forward");
  motor1.run(-200);  // Full speed forward for motor1  NEEDS TROUBLESHOOTING
  motor2.run(200);  // Full speed forward for motor2 
  i=i+1;

  }
 
}
