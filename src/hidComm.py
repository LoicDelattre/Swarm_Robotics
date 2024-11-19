#!/usr/bin/env python3

import hid
import time

vendor_id = 1046
product_id = 65535

print("Opening device")
	
h = hid.Device(vendor_id, product_id)
 
try:
	
    print("Manufacturer: %s" %h.manufacturer)
    print("Product: %s" %h.product)
    print("Serial No: %s" %h.serial)
	
	# enable non blocking mode
    h.nonblocking = 1
	
	# write some data
    print("Writing data")
    byte_str = bytes([29, 0xff, 0x55, 0x0b, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x10] + [0x00]*14)
    print(byte_str)
    h.write(byte_str)
		
    time.sleep(0.05)
	
	# read back some data
    print("Reading data")   
    while True:
        d = h.read(64)
        if d:
            print(d)
        else:
            break
			
    time.sleep(5)
    
	# write some data
    print("Writing data")
    byte_str = bytes([29, 0xff, 0x55, 0x0b, 0x12, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x02, 0x00, 0x10] + [0x00]*14)
    print(byte_str)
    h.write(byte_str)
	
    time.sleep(0.05)
	
	# read back some data
    print("Reading data")
    while True:
        d = h.read(64)  
        if d:
            print(d)
        else:
            break
	
    print("Closing the device")
    h.close()
	
except IOError as ex:
    print(ex)
    print("Error")
	
print("Done")