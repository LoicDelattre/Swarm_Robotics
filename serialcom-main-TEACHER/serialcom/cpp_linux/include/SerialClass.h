// inspired from https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://github.com/gbmhunter/CppLinuxSerial/tree/master
#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED

#define ARDUINO_WAIT_TIME 2000

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#define DEFAULT_READ_BUFFER_SIZE 255

class Serial
{
	private:
		bool m_bIsConnected; //Connection status
		int m_i32SerialPortIndex; // Serial port index
		struct termios m_oTty; // termios structure
		std::vector<char> m_vReadBuffer;
		unsigned char m_cReadBufferSize; 	

	public:
		//Initialize Serial communication with the given COM port
		Serial(char *portName);
		//Close the connection
		//NOTA: for some reason you can't connect again before exiting
		//the program and running it again
		~Serial();
		//Read data in a buffer, if nbChar is greater than the
		//maximum number of bytes available, it will return only the
		//bytes available. The function return -1 when nothing could
		//be read, the number of bytes actually read.
		int readData(std::string& readBuffer);
		//Writes data from a buffer through the Serial connection
		//return true on success.
		bool writeData(const std::string& writeBuffer);
		//Check if we are actually connected
		bool isConnected();

};

#endif // SERIALCLASS_H_INCLUDED