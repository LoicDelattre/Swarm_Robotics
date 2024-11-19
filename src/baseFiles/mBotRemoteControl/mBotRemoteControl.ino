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

// declare motor speeds
int _leftMotorSpeed = 0;
int _rightMotorSpeed = 0;

int _leftMotorDirection = 0;
int _rightMotorDirection = 0;

// serial port
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
  /*delay(duration);
  _leftMotor.stop();
  _rightMotor.stop();
  */
  
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

void parseData()
{
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  _leftMotorDirection = readBuffer(4);
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
    //delay(10);
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
    
     writeHead();
    Serial.write(index);
    Serial.write(prevc);
    Serial.write(c);
    Serial.write(dataLen);
    writeEnd();
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
  Serial.print(index);
}


void loop()
{
  index = 0x00;
  writeHead();
  Serial.write(index);
  writeEnd();
  
  while(1)
  {
    serialHandle();
  }
}
