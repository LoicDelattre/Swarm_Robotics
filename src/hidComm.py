#!/usr/bin/env python3

#pip install hidapi
#pip install hidapi-tools

import hid
import time

vendor_id = 0x80EE #33006
product_id = 0x21 #33

print("Opening device")
	
h = hid.device()
h.open(vendor_id, product_id)
 
def writeData(byte_str):
    print("Writing data")
    print(byte_str)
    h.write(byte_str)
	
    time.sleep(0.05)
    pass

try:
	
    print("Manufacturer: %s" %h.get_manufacturer_string())
    print("Product: %s" %h.get_product_string())
    print("Serial No: %s" %h.get_serial_number_string())
	
	# enable non blocking mode
    h.set_nonblocking(1)
	
	# write some data
    writeData(bytes([29, 0xff, 0x55, 0x0b, 0x12, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x02, 0x00, 0x00] + [0x00]*14))
	
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
    writeData(bytes([29, 0xff, 0x55, 0x0b, 0x12, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x02, 0x00, 0x00] + [0x00]*14))
	
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