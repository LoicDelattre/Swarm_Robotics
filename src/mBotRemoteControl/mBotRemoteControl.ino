#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMCore.h>

#define GET 1
#define RUN 2
#define RESET 4
#define START 5

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// declare gyro
MeGyro _gyro;
double _Kp_angle = -5.0;

// robot characteristics
double _wheelDiameter = 6.0; //cm
double _distanceBetweenWheels = 11.5; //cm

// declare motor speeds
int _leftMotorSpeed = 0;
int _rightMotorSpeed = 0;

int _leftMotorDirection = 0;
int _rightMotorDirection = 0;

int baseSpeed = 180;
float baseTurnRatio = 0.05;
float currentTurnRatio = 0; //positive is right when going forward ie left when backwards)

float movingThreshold = 0.01;

  //move flags for nextx instructions
  bool xMoveFlag = false;
  bool yMoveFlag = false;
  bool moveFlag = false;
  bool forwardFlag = false;

char buffer[52];
char serialRead;
boolean isAvailable = false;
boolean isStart = false;
unsigned char prevc=0;
byte index = 0;
byte dataLen;
uint8_t command_index = 0;

int getSign(float value) {
  if (value > 0) {
    return 1;  // Positive
  } else if (value < 0) {
    return -1; // Negative
  } else {
    return 0;  // Zero
  }
}

void move(int leftMotorSpeed, int rightMotorSpeed) //, int duration)
{
  _leftMotorSpeed = leftMotorSpeed;
  _rightMotorSpeed = rightMotorSpeed;
  
  _leftMotor.run((9)==M1?-(_leftMotorSpeed):(_leftMotorSpeed));
  _rightMotor.run((10)==M1?-(_rightMotorSpeed):(_rightMotorSpeed));
}

void spinToAngle(float targetAngle)
{
    // retrieve new values from gyro
    _gyro.update();
    // get the theta angle (Rz)
    float thetaBaseline = _gyro.getAngleZ();
    targetAngle += thetaBaseline;
    float errorAngle = targetAngle - thetaBaseline;

    bool isTargetAngleReached = false;
    while (!isTargetAngleReached)
    {
      // determine the command (proportionnal controller) i.e. angular velocity
      double v = 0.0;
      double w = _Kp_angle * errorAngle;
    
      // compute the command
      double leftWheelSpeed  =  v / (_wheelDiameter/2) - _distanceBetweenWheels * w / (_wheelDiameter) ;
      double rightWheelSpeed  = v / (_wheelDiameter/2) + _distanceBetweenWheels * w / (_wheelDiameter) ;
    
      // crop wheel speed
      if (leftWheelSpeed > 250)
        leftWheelSpeed = 250;
      if (rightWheelSpeed > 250)
        rightWheelSpeed = 250;
    
      // send the commands
      move(rightWheelSpeed, leftWheelSpeed);
    
      // retrieve new values from gyro
      _gyro.update();
    
      // get the theta angle (Rz)
      float thetaCurrent = _gyro.getAngleZ();
      errorAngle = targetAngle - thetaCurrent;

      if (abs(errorAngle) < 4)
        isTargetAngleReached = true;
    }
    Stop();

}

void callOK()
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

void Stop()
{
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));
}

float normVector(float u, float v)
{
  return sqrt(u*u+v*v);
}

float scalarProduct(float u1, float v1, float u2, float v2)
{
  return u1*u2+v1*v2;
}

float getVectorAngle(float u1, float v1, float u2, float v2)
{
  float referenceVectorNorm = normVector(u1, v1);
  float targetVectorNorm = normVector(u2, v2);
  float angle = acos( scalarProduct(u1, v1, u2, v2) / (referenceVectorNorm*targetVectorNorm) ) / M_PI * 180.0f;
  return angle;
}

float getVectorAngleSign(float u1, float v1, float u2, float v2)
{
  float angleSign = u1*v2 - u2*v1;
  return angleSign;
}

void parseData()
{
  isStart = false;

  // gets the command index
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;

  // retrieves values from hid device
  int xCurrentSign = readBuffer(4); int xCurrentPosition = readBuffer(5);
  int yCurrentSign = readBuffer(6); int yCurrentPosition = readBuffer(7);
  int xTargetSign = readBuffer(8); int xTargetPosition = readBuffer(9);
  int yTargetSign = readBuffer(10); int yTargetPosition = readBuffer(11);
  int currentAngleSign = readBuffer(12); int currentAngle = readBuffer(13);

  // applies sign
  if (xCurrentSign == 1) xCurrentPosition *=-1;
  if (yCurrentSign == 1) yCurrentPosition *=-1;
  if (xTargetSign == 1) xTargetPosition *=-1;
  if (yTargetSign == 1) yTargetPosition *=-1; 
  if (currentAngleSign == 1) currentAngle *=-1;

  bool stopFlag = false;
  if (yTargetSign == 0) stopFlag = true;

  // determines the target vector coordinates
  float uTargetVector = xTargetPosition - xCurrentPosition;
  float vTargetVector = yTargetPosition - yCurrentPosition;

  // specifies the reference vector coordinates
  //float uReferenceVector = 0.0;
  //float vReferenceVector = 1.0*getSign(vTargetVector);

  float val = 1.0f;
  float uReferenceVector = 0.0f;
  float vReferenceVector = val;

  // determines the target angle
  float targetAngle = getVectorAngle(uReferenceVector, vReferenceVector, uTargetVector, vTargetVector);
  
  // determines its sign
  float targetAngleSign = getVectorAngleSign(uReferenceVector, vReferenceVector, uTargetVector, vTargetVector);
  if (targetAngleSign < 0)
    targetAngle *= -1;
  
  //subtract the current angle
  targetAngle -= currentAngle;

  //spinToAngle(targetAngle);

  int angleToSend = 0;
  if (targetAngle < 0)
  {
    angleToSend = floor(abs(targetAngle));
    targetAngleSign = 1;
  }
  else
  {
    angleToSend = floor(targetAngle);
    targetAngleSign = 0;
  }


  writeHead();
    writeSerial(idx);
    writeSerial(targetAngleSign);
    writeSerial(angleToSend);
  writeEnd();  
  
  if (abs(uTargetVector) > movingThreshold)
  {
    xMoveFlag = true;
  }
  if (abs(vTargetVector) > movingThreshold){
    yMoveFlag = true;
  }
  if (xMoveFlag and yMoveFlag){
    moveFlag = true;
  }

  //move purely forward or backwards directions
  if (vTargetVector > 0){
    _leftMotorDirection = 1;
    _rightMotorDirection = 1;
    forwardFlag = true;
  }
  else{
    _leftMotorDirection = -1;
    _rightMotorDirection = -1;
    forwardFlag = false;
  }

  //only apply speed if we have a diff postion from cur positon
  if (yMoveFlag){
    _leftMotorSpeed = baseSpeed*_leftMotorDirection;
    _rightMotorSpeed = baseSpeed*_rightMotorDirection;
  }
  if (xMoveFlag){
    int targetSignRotation = 1;
    if (xTargetSign == 1) targetSignRotation =-1;
    if (forwardFlag){
        updateTurnRatio(targetSignRotation);
    }
    else{
        updateTurnRatio(-targetSignRotation);
    }
    _leftMotorSpeed *= (1+currentTurnRatio);
    _rightMotorSpeed *= (1-currentTurnRatio);
  }
  if (stopFlag){
    _leftMotorSpeed = 0;
    _rightMotorSpeed = 0;
  }


  move(_leftMotorSpeed, _rightMotorSpeed);
  /*
  writeHead();
  writeSerial(idx);
  writeSerial(_leftMotorSpeed);
  writeSerial(_rightMotorSpeed);
  writeEnd();  
  */
}

void updateTurnRatio(int direction){
  if (direction == 1) //negative means left rotation
  {
    currentTurnRatio -= baseTurnRatio;
  }  

  else{//right rotation
      currentTurnRatio += baseTurnRatio;
    }
}

void writeBuffer(int16_t index, unsigned char c)
{
  buffer[index]=c;
}


void writeSerial(unsigned char c)
{
  Serial.write(c);
}

void writeHead()
{
  writeSerial(0xff);
  writeSerial(0x55);
}

void writeEnd()
{
  Serial.println(); 
}

unsigned char readBuffer(int16_t index)
{
  return buffer[index]; 
}

void readSerial()
{
  isAvailable = false;
  if(Serial.available() > 0)
  {
    isAvailable = true;
    serialRead = Serial.read();
    // uncomment to send back every single received byte
    //Serial.write(serialRead);
  }
}

void serialHandle()
{
  readSerial();
  if(isAvailable)
  {
    unsigned char c = serialRead & 0xff;
    if((c == 0x55) && (isStart == false))
    {
      if(prevc == 0xff)
      {
        index=1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if(isStart)
      {
        if(index == 2)
        {
          dataLen = c; 
          
        }
        else if(index > 2)
        {
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
   
    /*
    writeHead();
    Serial.write(index);
    Serial.write(prevc);
    Serial.write(c);
    Serial.write(dataLen);
    writeEnd();
    */

    index++;
    
    if(index > 51)
    {
      index=0; 
      isStart = false;
    }
    if(isStart && (dataLen == 0) && (index > 3))
    { 
      isStart = false;
      parseData(); 
      index=0;
    }
  }
}

void setup()
{
  delay(5);
  Stop();
  Serial.begin(115200);
  //Serial.print(index);

  // initialize the gyro
  _gyro.begin();

  // wait for 2 seconds
  delay(2000);

  // retrieve new values from gyro
  _gyro.update();


}


void loop()
{
  index = 0x00;
  
  while(1)
  {
    serialHandle();
  }
}
