import rospy
from geometry_msgs.msg import Twist
import hid
import time

#!/usr/bin/env python

if __name__ == '__main__':
    rospy.init_node('GyattBot')

    rospy.loginfo("GyattBot node started ©")
    rospy.loginfo("Kaaris 2024")

    class HIDComm():
        def __init__(self) -> None:

            self.vendor_id = 0x416 #1046
            self.product_id = 0xFFFF #65535
        
            device_list = hid.enumerate(self.vendor_id, self.product_id)
            self.botsNumber = len(device_list)#number of bots controlled
            print(device_list)

            self.devices = []
            for i in range(0, self.botsNumber):
                path = device_list[i]["path"]
                self.devices.append(self.openHidDevice(path))

            self.baseSleepTime = 2

            #initializa all variablesszszse
            self.x_cur_pos = 0x00
            self.y_cur_pos = 0x00
            self.x_cur_sign = 0x00
            self.y_cur_sign = 0x00
            self.x_tar_sign = 0x00
            self.x_targ_pos = 0x00
            self.y_tar_sign = 0x00
            self.y_targ_pos = 0x00
            self.cur_ang_sign = 0x00
            self.cur_ang = 0x00

            self.y_sign = 0x00 #makes stop
            self.y_tar_pos = 0x00

            self.runFlag = True
            pass

        def openHidDevice(self, path):
            print("'Opening device' - Kaaris 2024")
            h = hid.device()
            h.open_path(path)
            #h.open(self.vendor_id, self.product_id)
            h.set_nonblocking(1)
            print("Device opened")
            return h

        def writeData(self, xTargetSign, xTargetPos, yTargetSign, yTargetPos):
            byte_str = bytes([29, 0xff, 0x55, 0x0b, 0x00, self.x_cur_sign, self.x_cur_pos, self.y_cur_sign, self.y_cur_pos, 
                     xTargetSign, xTargetPos, yTargetSign, yTargetPos, self.cur_ang_sign, self.cur_ang] + [0x00]*14)
            print("Writing data")
            print(byte_str)

            for device in self.devices:
                device.write(byte_str)
	
            self.x_cur_sign = xTargetSign
            self.x_cur_pos = xTargetPos

            #self.y_cur_sign = yTargetSign
            #self.y_cur_pos = yTargetPos

            time.sleep(0.05)
            pass

        def readData(self):
            # read back some data 
            print("'Reading data'")   
            while True:
                d = self.h.read(64) #depreciated
                if d:
                    print(d)
                else:
                    break
			
            time.sleep(self.baseSleepTime)
            pass
	
        def closeDevice(self):
            print("Closing the device")
            for device in self.devices:
                device.close()
            pass

        def moveForward(self):
            x_sign = 0x00
            x_tar_pos = 0x00
            self.y_sign = 0xff
            self.y_tar_pos = 0xff
            self.writeData(x_sign, x_tar_pos, self.y_sign, self.y_tar_pos)
            print("skibidding forward")
            pass

        def moveBackward(self):
            x_sign = 0x00
            x_tar_pos = 0x00
            self.y_sign = 0x01
            self.y_tar_pos = 0xff
            self.writeData(x_sign, x_tar_pos, self.y_sign, self.y_tar_pos)
            print("skibidding backward")
            pass

        def stopMoving(self):
            x_sign = 0x00
            x_tar_pos = 0x00
            self.y_sign = 0x00 #makes stop
            self.y_tar_pos = 0x00
            self.writeData(x_sign, x_tar_pos, self.y_sign, self.y_tar_pos)
            print("stop skibidding")
            pass

        def turnLeft(self):
            x_sign = 0x01
            x_tar_pos = 0x10
            self.writeData(x_sign, x_tar_pos, self.y_sign, self.y_tar_pos)
            print("skibidding left")
            pass

        def turnRight(self):
            x_sign = 0x10
            x_tar_pos = 0x10
            self.writeData(x_sign, x_tar_pos, self.y_sign, self.y_tar_pos)
            print("skibidding left")
            pass

        def mainTask(self):
            while self.runFlag:
                self.checkKeys()
                #self.readData()
                time.sleep(0.01)
                pass
        
            print("Done")
            pass

    commREF = HIDComm()

    def cmd_vel_callback(msg): 
        try:
            if msg.linear.x == 1:
                commREF.moveForward()
            if msg.angular.z == -1:
                commREF.turnLeft()
            if msg.angular.z == 1:
                commREF.turnRight()
            if msg.linear.x == 0 and msg.angular.z == 0:
                commREF.stopMoving()
            if msg.linear.x == -1:
                commREF.moveBackward()
            if msg.linear.y == 1:
                commREF.stopMoving()
                commREF.closeDevice()
                runFlag = False
            pass

        except Exception as e:
            rospy.logerr(f"Failed to send data via HID: {e}")

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()


