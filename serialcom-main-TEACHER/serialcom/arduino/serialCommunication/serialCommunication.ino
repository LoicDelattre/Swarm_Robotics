// More information about serial communication can be found here:
// https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialEvent
// and here:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/


// baud rate of the serial port
const int _portBaud = 9600; 

String _inputString = "";         // a String to hold incoming data
bool _stringComplete = false;  // whether the string is complete

void setup() 
{
  // initializes the serial port
  Serial.begin(_portBaud);

  Serial.setTimeout(1);
}

void loop() 
{
  if (_stringComplete) 
  {
    // if the input String has the following form "Position:<xCoord>;<yCoord>\n"
    // where <xCoord> and <yCoord> are float values
    // then we can retrieve xCoord, yCoord doing the following commands
    float xCoord = 0.0;
    float yCoord = 0.0;
    int headerEndIndex = _inputString.indexOf(':');
    int semiColonIndex = _inputString.indexOf(';');
    int eolIndex = _inputString.indexOf('\n');
    xCoord = (_inputString.substring(headerEndIndex+1, semiColonIndex)).toFloat();
    yCoord = (_inputString.substring(semiColonIndex+1, eolIndex)).toFloat();

    if (headerEndIndex != -1 && semiColonIndex!= -1 && eolIndex != -1)
    {
      Serial.print("(x, y) = ("); 
      Serial.print(xCoord); Serial.print(", "); Serial.print(yCoord);Serial.print(")\n");
    }
     
    // clears the string:
    _inputString = "";
    _stringComplete = false;
    return;
  }
}


/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() 
{
  while (Serial.available()) 
  {
    // gets the new byte:
    char inChar = (char)Serial.read();
    // adds it to the _inputString:
    _inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // does something about it:
    if (inChar == '\n') 
    {
      _stringComplete = true;
    }
  }
}
