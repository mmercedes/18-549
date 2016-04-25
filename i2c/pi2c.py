import smbus
import time
STOP = 0
PEN_UP = 1
PEN_DOWN = 2
CALIBRATE = 3 
DRAW = 4
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(a1,a2,b1,b2,c1,c2,d1,d2,s):
        bus.write_i2c_block_data(address,a1, [a2,b1,b2,c1,c2,d1,d2,s])
        # bus.write_byte_data(address, 0, value)
        return -1

def readNumber():
        number = bus.read_byte(address)
        # number = bus.read_byte_data(address, 1)
        return number

while True:

        writeNumber(1,1,2,5,3,2,4,1,0)
        print "RPI: Hi Arduino, I sent you "
        # sleep one second
        time.sleep(1)

        number = readNumber()
        print "Arduino: Hey RPI, I received a digit ", number
        print