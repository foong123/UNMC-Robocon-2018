import evdev
from evdev import InputDevice, categorize, ecodes
import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
#creates object 'gamepad' to store the data
#you can call it whatever you like
gamepad = InputDevice('/dev/input/event0')

#Usage Buttons
aBtn = 304          #DOWN
yBtn = 308          #UP
xBtn = 307          #LEFT
bBtn = 308          #RIGHT

L1 = 310            #TURN LEFT
L2 = 312
L3 = 317

R1 = 311            #TURN RIGHT
R2 = 313
R3 = 318

#AXIS Buttons
#Analog
LAvert = 1
LAhoriz = 0

RAvert = 5
RAhoriz = 2

#DPad
dvert = 17
dhoriz = 16

#Triggers
L2T = 10
R2T = 9

#KEY Buttons
start = 315
select = 314

#prints out device info at start
print(gamepad)

#loop and filter by event code and print the mapped label
for event in gamepad.read_loop():
    
    if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:

        if event.value != 0:
            
            #Action Buttons
            if event.code == aBtn:
                ser.write(b"A")
                print("Down")
            elif event.code == bBtn:
                ser.write(b"B")
                print("Right")
            elif event.code == xBtn:
                ser.write(b"X")
                print("Left")
            elif event.code == yBtn:
                ser.write(b"Y")
                print("Up")
            
            #Left Bumpers/Triggers
            elif event.code == L1:
                print("L1, val= %d ", event.value)
                ser.write(b"L1")
            #Right Bumpers/Triggers
            elif event.code == R1:
                print("R1, val= %d ", event.value)
                ser.write(b"R1")

            '''elif event.code == L2:
                print("L2, event.value= %d ", event.value)
            elif event.code == L2T:
                print("L2Trig, val= %d ", event.value)
            elif event.code == L3:
                print("L3, val= %d ", event.value)
            elif event.code == R2:
                print("R2, event.value= %d ", event.value)
            elif event.code == R2T:
                print("R2Trig, val= %d ", event.value)
            elif event.code == R3:
                print("R3, val= %d ", event.value)
                '''
        if event.value != 128:
            while(True):
                #Left Analog
                if event.code == LAvert and event.value < 128:
                    print(" LUp, val= %d ", event.value)
                elif event.code == LAvert and event.value > 128:
                    print(" LDown, val= %d ", event.value)
                elif event.code == LAhoriz and event.value < 128:
                    print(" LLeft, val= %d ", event.value)
                elif event.code == LAhoriz and event.value > 128:
                    print(" LRight, val= %d ", event.value)

                # Right Analog(Turning)
                elif event.code == RAvert and event.value < 128:
                    print(" RUp, val= %d ", event.value)
                elif event.code == RAvert and event.value > 128:
                    print(" RDown, val= %d ", event.value)
                elif event.code == RAhoriz and event.value < 128:
                    print(" RLeft, val= %d ", event.value)
                elif event.code == RAhoriz and event.value > 128:
                    print(" RRight, val= %d ", event.value)

            ''' #DPad    
            elif event.code == dvert and event.value == -1:
               print("Dup")
            elif event.code == dvert and event.value == 1:
               print("Ddown")
            elif event.code == dhoriz and event.value == -1:
               print("Dleft")
            elif event.code == dhoriz and event.value == 1:
               print("Dright")
            
            #Start/Select
            elif event.code == start:
               print("start, val= %d ", event.value)
            elif event.code == select:
               print("select, val= %d ", event.value)
               '''