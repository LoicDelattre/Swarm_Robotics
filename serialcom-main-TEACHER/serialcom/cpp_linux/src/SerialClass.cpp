// From https://github.com/Gmatarrubia/LibreriasTutoriales/tree/master
#include "SerialClass.h"

Serial::Serial(char* portName): m_bIsConnected(false), m_i32SerialPortIndex(0), m_cReadBufferSize(DEFAULT_READ_BUFFER_SIZE)
{
	// Open the serial port
	m_i32SerialPortIndex = open(portName, O_RDWR );

	if (m_i32SerialPortIndex < 0)
	{
		printf("[ERROR] %i from open: %s\n", errno, strerror(errno));
	}
	else
	{
		m_bIsConnected = true;
		std::cout << "\t[INFO] Serial port opened successfully!" <<std::endl;
	}
	
	// Create the termios
	if(tcgetattr(m_i32SerialPortIndex, &m_oTty) !=  0) 
	{
		m_bIsConnected = false;
		printf("\t[ERROR] %i from tcgetattr: %s\n", errno, strerror(errno));
	}
	else
		std::cout << "\t[INFO] Termios structure  created successfully!" <<std::endl;
	
	// Fill the termios structure
	m_oTty.c_cflag &= ~PARENB; // disabling parity
	m_oTty.c_cflag &= ~CSTOPB; //only one stop bit used in comunication
	m_oTty.c_cflag |= CS8; // 8 bits per byte
	m_oTty.c_cflag &= ~CRTSCTS; //disabling RTS/CTS hardware flow control
	m_oTty.c_cflag |= CREAD | CLOCAL;  //turn on READ & ignore ctrl lines (CLOCAL = 1)

	m_oTty.c_lflag &= ~ICANON; // disable canonical mode (\ are treated specially if enable)
	m_oTty.c_lflag &= ~ECHO; // disable echo
	m_oTty.c_lflag &= ~ECHOE; //disable erasure
	m_oTty.c_lflag &= ~ECHONL; // disable new line echo
	m_oTty.c_lflag &= ~ISIG; // disable interpretation of INTR, QUIT and SUSP

	m_oTty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow control
	m_oTty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // disable any special handling of received bytes

	m_oTty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (e.g. new line character)
	m_oTty.c_oflag &= ~ONLCR; // prevent conversion of new line to carriage retunr/line feed

	m_oTty.c_cc[VTIME] = 10; //blocking read until either any amount of data is available or timeout (1 decisecond = 100 ms) occurs 
	m_oTty.c_cc[VMIN] =0;

	// set in/out baud rate to 9600
	cfsetispeed(&m_oTty, B9600);
	cfsetospeed(&m_oTty, B9600);
	
	if (tcsetattr(m_i32SerialPortIndex, TCSANOW, &m_oTty) != 0)
	{
		m_bIsConnected = false;
		printf("\t[ERROR] %i from tcsetattr: %s\n", errno, strerror(errno));
	}
	else
		std::cout << "\t[INFO] Termios structure saved successfully!" << std::endl;
	
	// flsuh RX buffer
	if (tcflush(m_i32SerialPortIndex, TCIFLUSH < 0))
	{
		m_bIsConnected = false;
		close(m_i32SerialPortIndex);
		std::cout << "\t[ERROR] TCould not flush the RX buffer, closing the port!" << std::endl;
	}
	
	// reserve space for reading
	m_vReadBuffer.reserve(DEFAULT_READ_BUFFER_SIZE);
}

Serial::~Serial()
{
	// Check if we are connected before trying to disconnect
	if(m_bIsConnected)
	{
		//We're no longer connected
		m_bIsConnected = false;
		//Close the serial handler
		close(m_i32SerialPortIndex);
		std::cout << "[INFO] Serial port is closed!" << std::endl;
	}
	else
		std::cout << "[INFO] Serial port was already closed, no need to close it again!" << std::endl;
}

int Serial::readData(std::string& readBuffer)
{
	int nbReadBytes  = 0;
	
	if(m_bIsConnected)
	{
		nbReadBytes = read(m_i32SerialPortIndex, &m_vReadBuffer[0], m_cReadBufferSize);
		
		if (nbReadBytes < 0)
		{
			std::cout << "[ERROR] Reading the Serial port was unsuccesfull!" << std::endl;
		}
		else if (nbReadBytes == 0)
		{
			std::cout << "[ERROR] Reading the Serial port was unsuccesfull!" << std::endl;
			std::cout << "[ERROR] Device disconnected!" << std::endl;
		}
		else if (nbReadBytes > 0)
		{
			readBuffer += std::string(&m_vReadBuffer[0], nbReadBytes);
		}
	}
	else
	{
		std::cout << "[ERROR] Try reading on a closed Serial port!" << std::endl;
		return -2;
	}
	
	return nbReadBytes;

}


bool Serial::writeData(const std::string& writeBuffer)
{
	if (m_bIsConnected)
	{
		if (write(m_i32SerialPortIndex, writeBuffer.c_str(), writeBuffer.size()))
		{
			//std::cout << "[INFO] Writing " << writeBuffer.size() << " bytes on the Serial port : " << writeBuffer << std::endl;
			return true;
		}
		else
		{
			std::cout << "[ERROR] Could not write on the Serial port!" << std::endl;
			return false;
		}
	}
	else
	{
		std::cout << "[ERROR] Try writing on a closed Serial port!" << std::endl;
		return false;
	}
}

bool Serial::isConnected()
{
	//Simply return the connection status
	return m_bIsConnected;
}

