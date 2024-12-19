import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Define key mappings
key_mapping = {
    'z': (1, 0),  # Move forward
    's': (-1, 0), # Move backward
    'q': (0, 1),  # Turn left
    'd': (0, -1), # Turn right
    'x': (0, 0)   # Stop
}

def get_key():
    """Get a single key press from the keyboard."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key

def main():
    rospy.init_node('keyboard_to_cmd_vel')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    global settings
    settings = termios.tcgetattr(sys.stdin)

    print("Use 'WASD' keys to control the robot:")
    print("  Z: Forward\n  S: Backward\n  Q: Turn Left\n  D: Turn Right\n  X: Stop")
    

    rate = rospy.Rate(10) # 10 Hz
    twist = Twist()

    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key in key_mapping:
                linear, angular = key_mapping[key]
                twist.linear.x = linear
                twist.angular.z = angular
            else:
                print("Unknown key: {}, stopping robot.".format(key))
                twist.linear.x = 0
                twist.angular.z = 0

            pub.publish(twist)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
