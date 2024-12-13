import rospy
from geometry_msgs.msg import Twist
import hid

#!/usr/bin/env python

if __name__ == '__main__':
    rospy.init_node('GyattBot')

    rospy.loginfo("GyattBot node started ©")
    rospy.loginfo("Kaaris 2024")

    def cmd_vel_callback(msg):
        
        # Send data via HID
        try:
            ######################Loic met le code ici######################
            h = hid.device()
            h.open(0x1234, 0x5678)  # VendorID/ProductID
            h.write(hex_data)
            h.close()
        except Exception as e:
            rospy.logerr(f"Failed to send data via HID: {e}")

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()


