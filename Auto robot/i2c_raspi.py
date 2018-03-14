import RPi.GPIO as gpio
import smbus
#import smbus2 as smbus for cv environment
import time
import sys
bus = smbus.SMBus(1)
address = 0x04
gpio.setwarnings(False)
def main():
    gpio.setmode(gpio.BCM)
    gpio.setup(17, gpio.OUT)
    while 1:
	status = True
        gpio.output(17, status)
	var = input('Enter number less than 10:')
        status = False
	gpio.output(17, status)
        bus.write_byte(address, int(var))
        print ('RPI to Arduino:', int(var))
	time.sleep(1)
	bus.write_byte(address, 11)
	bus.read_byte(address)
	print('Arduino to RPI:',int(var))
	
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        gpio.cleanup()
        sys.exit(0)