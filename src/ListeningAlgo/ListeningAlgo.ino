#include <MeMCore.h>  // Include the mBot's motor control library

MeDCMotor motor1(M1);  // Motor on port M1
MeDCMotor motor2(M2);  // Motor on port M2

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud
}

void loop() {
  if (Serial.available()) {  // Check if data is available to read
    char command = Serial.read();  // Read the incoming byte
    
    switch(command) {
      case 'F':  // Move forward
        motor1.run(-200);  // Full speed forward for motor1
        motor2.run(200);  // Full speed forward for motor2
        break;
      case 'B':  // Move backward
        motor1.run(200);  // Full speed backward for motor1
        motor2.run(-200);  // Full speed backward for motor2
        break;
      case 'L':  // Turn left
        motor1.run(0);  // Reverse motor1 to turn left
        motor2.run(255);   // Forward motor2 to turn left
        break;
      case 'R':  // Turn right
        motor1.run(-255);   // Forward motor1 to turn right
        motor2.run(0);  // Reverse motor2 to turn right
        break;
      case 'S':  // Stop
        motor1.run(0);  // Stop motor1
        motor2.run(0);  // Stop motor2
        break;
      default:
        break;
    }
  }
}