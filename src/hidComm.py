#!/usr/bin/env python3

#sudo pip install hidapi 1.0.4
#sudo pip install hidapi-tools 

import hid
import time
import keyboard as keys

class HIDComm():
    def __init__(self) -> None:
        self.vendor_id = 0x416 #1046
        self.product_id = 0xFFFF #65535

        self.h = self.openHidDevice()

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

        self.runFlag = True
        pass

    def openHidDevice(self):
        print("Opening device")
        h = hid.device()
        h.open(self.vendor_id, self.product_id)
        h.set_nonblocking(1)
        return h

    def writeData(self, xTargetSign, xTargetPos, yTargetSign, yTargetPos):
        byte_str = bytes([29, 0xff, 0x55, 0x0b, 0x00, self.x_cur_sign, self.x_cur_pos, self.y_cur_sign, self.y_cur_pos, 
                     xTargetSign, xTargetPos, yTargetSign, yTargetPos, self.cur_ang_sign, self.cur_ang] + [0x00]*14)
        print("Writing data")
        print(byte_str)
        self.h.write(byte_str)
	
        self.x_cur_sign = xTargetSign
        self.x_cur_pos = xTargetPos

        #self.y_cur_sign = yTargetSign
        #self.y_cur_pos = yTargetPos

        time.sleep(0.05)
        pass

    def readData(self):
        # read back some data 
        print("Reading data")   
        while True:
            d = self.h.read(64)
            if d:
                print(d)
            else:
                break
			
        time.sleep(self.baseSleepTime)
        pass
	
    def closeDevice(self):
        print("Closing the device")
        self.h.close()
        pass

    def moveForward(self):
        x_sign = 0x00
        x_tar_pos = 0x00
        y_sign = 0xff
        y_tar_pos = 0xff
        self.writeData(x_sign, x_tar_pos, y_sign, y_tar_pos)
        print("skibidding forward")
        pass

    def moveBackward(self):
        x_sign = 0x00
        x_tar_pos = 0x00
        y_sign = 0x01
        y_tar_pos = 0xff
        self.writeData(x_sign, x_tar_pos, y_sign, y_tar_pos)
        print("skibidding backward")
        pass

    def stopMoving(self):
        x_sign = 0x00
        x_tar_pos = 0x00
        y_sign = 0x00 #makes stop
        y_tar_pos = 0x00
        self.writeData(x_sign, x_tar_pos, y_sign, y_tar_pos)
        print("stop skibidding")
        pass

    def checkKeys(self):
        if keys.is_pressed('z'):
            self.moveForward()
        if keys.is_pressed("e"):
            self.stopMoving()
        if keys.is_pressed("s"):
            self.moveBackward()
        if keys.is_pressed('p'):
            self.stopMoving()
            self.closeDevice()
            self.runFlag = False
        pass

    def mainTask(self):
        while self.runFlag:
            self.checkKeys()
            #self.readData()
            time.sleep(0.01)
            pass
        
        print("Done")
        pass

HIDComm().mainTask()