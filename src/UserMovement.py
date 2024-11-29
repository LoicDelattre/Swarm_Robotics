import serial
import time
import keyboard

port= 'COM5'
baudrate=9600
# Initialize the serial connection (adjust the port and baudrate as needed)
mbot_serial = serial.Serial(port= 'COM5' , baudrate=9600, timeout=1)

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
def move_forward(): 
    send_command('F')  # Forward movement command NEEDS TROUBLESHOOTING

def turn_left():
    send_command('L')  # Left turn command

def turn_right():
    send_command('R')  # Right turn command

def move_backwards():
    send_command('B') # backward movement command
    
def stop_movement():
    send_command('S')


# Main function to handle key presses
def control_mbot_with_keys():
    print("Use W (Forward), A (Left), D (Right), S (Backward) to control the bot.")
    print("Press 'Q' to quit.")
    
    try:
        while True:
            if keyboard.is_pressed('z'):  # Move forward
                move_forward()
            elif keyboard.is_pressed('q'):  # Turn left
                turn_left()
            elif keyboard.is_pressed('d'):  # Turn right
                turn_right()
            elif keyboard.is_pressed('s'):  # Move backward
                move_backwards()
            else:
                stop_movement()  # Stop when no key is pressed
            
            if keyboard.is_pressed('a'):  # Exit the program
                print("Exiting control...")
                break
            
            time.sleep(0.1)  # Reduce CPU usage
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        stop_movement()  # Ensure the mBot stops
        mbot_serial.close()  # Close the serial connection




#Ports are not gonna be used for wireless communication
# Run the key control function
control_mbot_with_keys()
print("Keys are active")
