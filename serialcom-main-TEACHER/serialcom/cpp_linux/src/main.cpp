#include <iostream>

#include "SerialClass.h"

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "[ERROR] No serial port was provided!" << std::endl;
		std::cout << "[INFO] use ./main /dev/ttyACM0 for instance" << std::endl;
		return 1;
	}
	
	// connects to the serial port 
	std::cout<< "[INFO] Connecting to port " <<  argv[1] << std::endl;
	Serial serialPort(argv[1]);
	
	// checks if it is open
	if (serialPort.isConnected())
	{
		std::cout<< "[INFO] Connected to the serial port! " << std::endl;
		std::cout << "===========" <<  std::endl;
	}
	else
	{
		std::cout<< "... FAILED! " << std::endl;
		std::cout<< "[ERROR] Cannot connect to port " << argv[1] << "!" << std::endl;
		std::cout << "===========" <<  std::endl;
		return 1;
	}
	
	sleep(2); //wait 2s while the Arduino resets

	int counter = 0;
	int nbIterations = 10;
	float x = -0.1;
	float y = 0.1;
	
	while (serialPort.isConnected() && counter < nbIterations)
	{
		// creates a string to send
		std::string str2send = "CurrentPosition:" + std::to_string(x) + "; " + std::to_string(y) + "\n";
		
		// sends a string 
		serialPort.writeData(str2send);
		
		// waits a while (100 ms)
		usleep(100000); 
		
		// reads a string back
		std::string receivedStr;
		serialPort.readData(receivedStr);
		
		// displays the results
		std::cout << "Iteration: " <<  counter+1 << "/" << nbIterations << std::endl ;
		std::cout << "[INFO] Message sent: " <<  str2send ;
		std::cout << "[INFO] Message received: " <<  receivedStr;
		std::cout << "-----------------------" <<  std::endl;
		
		// increments the counter
		counter++;
		x += -0.1;
		y += 0.1;
	}
	
	return 0;
}
