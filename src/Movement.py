import serial
import time

# Initialize the serial connection (adjust port and baudrate as needed)
mbot_serial = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)

# Helper function to send commands
def send_command(command):
    """
    Sends a command to the mBot over the serial connection.
    :param command: List of bytes to send
    """
    mbot_serial.write(bytearray(command))
    time.sleep(0.1)  # Give the mBot time to process the command

# Function to move forward
def move_forward(speed, duration):
    """
    Moves the mBot forward.
    :param speed: Speed of the motors (0-255)
    :param duration: Duration to move in seconds
    """
    send_command([0xff, 0x55, 0x06, 0x02, 0x02, 0x05, speed, speed])  # Move forward
    time.sleep(duration)
    stop()  # Stop after moving

# Function to turn the mBot
def turn(direction, speed, duration):
    """
    Turns the mBot in place.
    :param direction: "left" or "right"
    :param speed: Speed of the motors (0-255)
    :param duration: Duration to turn in seconds
    """
    if direction == "left":
        send_command([0xff, 0x55, 0x06, 0x02, 0x02, 0x05, -speed & 0xff, speed])  # Turn left
    elif direction == "right":
        send_command([0xff, 0x55, 0x06, 0x02, 0x02, 0x05, speed, -speed & 0xff])  # Turn right
    time.sleep(duration)
    stop()  # Stop after turning

# Function to stop the mBot
def stop():
    """
    Stops the mBot by setting both motor speeds to 0.
    """
    send_command([0xff, 0x55, 0x06, 0x02, 0x02, 0x05, 0, 0])

# Main sequence
try:
    print("Moving forward...")
    move_forward(speed=100, duration=2)  # Move forward for 2 seconds

    print("Turning left...")
    turn(direction="left", speed=100, duration=1)  # Turn left for 1 second

    print("Moving forward again...")
    move_forward(speed=100, duration=2)  # Move forward for another 2 seconds

    print("Turning right...")
    turn(direction="right", speed=100, duration=1)  # Turn right for 1 second

    print("Sequence complete!")
except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    stop()  # Ensure mBot stops
    mbot_serial.close()  # Close the serial connection