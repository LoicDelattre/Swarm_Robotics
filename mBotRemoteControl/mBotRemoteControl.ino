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


char buffer[52];
char serialRead;
boolean isAvailable = false;
boolean isStart = false;
unsigned char prevc=0;
byte index = 0;
byte dataLen;
uint8_t command_index = 0;


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

  //yTargetPosition *=-1;

  // determines the target vector coordinates
  float uTargetVector = xTargetPosition - xCurrentPosition;
  float vTargetVector = yTargetPosition - yCurrentPosition;

  // specifies the reference vector coordinates
  float uReferenceVector = 1.0f;
  float vReferenceVector = 0.0f;

  // determines the target angle
  float targetAngle = getVectorAngle(uReferenceVector, vReferenceVector, uTargetVector, vTargetVector);
  
  // determines its sign
  float targetAngleSign = getVectorAngleSign(uReferenceVector, vReferenceVector, uTargetVector, vTargetVector);
  if (targetAngleSign < 0)
    targetAngle *= -1;
  
  //subtract the current angle
  targetAngle -= currentAngle;

  spinToAngle(targetAngle);

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

 
  /*_leftMotorDirection = readBuffer(4);
  _leftMotorSpeed = readBuffer(5);
  _rightMotorDirection = readBuffer(6);
  _rightMotorSpeed = readBuffer(7);

  if (_leftMotorDirection == 0)
    _leftMotorSpeed *=-1;

  if (_rightMotorDirection == 0)
    _rightMotorSpeed *=-1;
    
  move(_leftMotorSpeed, _rightMotorSpeed);
  
  writeHead();
  writeSerial(idx);
  writeSerial(_leftMotorSpeed);
  writeSerial(_rightMotorSpeed);
  writeEnd();  
  */
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
