import sys, getopt
import pigpio
sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"
pi = pigpio.pi()

if not pi.connected():
	exit()
	
servo_pin = 18;
pwmA = 23
pwmB = 24

pi.setP_mode(servo_pin,pigpio.OUTPUT)
pi.setP_mode(pwmA,pigpio.OUTPUT)
pi.setP_mode(pwmB,pigpio.OUTPUT)

def motorstart()
	set_PWM_dutycycle(pwmA,100)
	set_PWM_dutycycle(pwmB,0)

def servo_start()
	pi.set_servo_pulsewidth(servo_pin, 1000) # safe anti-clockwise
	print("clamped")


def release()
		pi.set_servo_pulsewidth(servo_pin, 1000) # safe anti-clockwise
		print("Released")


	
	
print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)
t1 = time.perf_counter()
##print(t1)
release_angle = 15
error = 1
servo_start()
signal =  input("Ready?")
if signal == "OK" :
	motorstart()

while True:
  if imu.IMURead():
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
	roll = int(math.degrees(fusionPose[0])
	pitch = int(math.degrees(fusionPose[1])
	yaw = int(math.degrees(fusionPose[2])
    print("r: %d p: %d y: %d" % (roll,pitch,yaw))
	if pitch <= release_angle + error and pitch >= release_angle - error:
		release()
		print(pitch)
		while(1):
			pass
    #time.sleep(poll_interval*1.0/1000.0)
   
        
