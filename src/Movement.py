import serial
import time

port= 'COM4'
baudrate=9600
# Initialize the serial connection (adjust the port and baudrate as needed)
mbot_serial = serial.Serial(port= 'COM4' , baudrate=9600, timeout=1)

def openSerialPort(mbot_serial, port, baudRate):
	print("--> COM Port = ", port)
	print("--> Baud rate = ", baudrate)
	mbot_serial.baudrate = baudrate
	mbot_serial.port = port
	try:
		mbot_serial.open()
	except serial.SerialException as e:
		print("[ERROR] ", e)
		return	
	
	time.sleep(2) # give the connection 2s to settle (Arduino board resets)
	
openSerialPort(mbot_serial,port,baudrate)

def send_command(command):
    """
    Sends a command to the mBot over the serial connection.
    :param command: Single-character string (e.g., 'F', 'L', 'R', 'S')
    """
    mbot_serial.write(command.encode())  # Send the command as bytes
    time.sleep(0.1)  # Give the mBot time to process the command

# Movement functions
def move_forward(duration):
    send_command('F')  # Forward movement command
    time.sleep(duration)
    send_command('S')  # Stop after moving

def turn_left(duration):
    send_command('L')  # Left turn command
    time.sleep(duration)
    send_command('S')  # Stops after turning

def turn_right(duration):
    send_command('R')  # Right turn command
    time.sleep(duration)
    send_command('S')  # Stops after turning
    
def move_backwards(duration):
    send_command('B') # backward movement command
    time.sleep(duration)
    send_command('S') # stops

# Main sequence
try:
    print("Moving forward...")
    move_forward(1)  # Move forward for 2 seconds

    print("Turning left...")
    turn_left(1)  # Turn left for 1 second

    print("Turning right...")
    turn_right(1)  # Turn right for 1 second

    print("Sequence complete!")
except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    send_command('S')  # Ensure the mBot stops
    mbot_serial.close()  # Close the serial connection

#Ports are not gonna be used for wireless communication


