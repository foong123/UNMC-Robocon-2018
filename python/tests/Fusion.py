import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math

SETTINGS_FILE = "RTIMULib"

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
n= 1
while True:
  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    print("r: %d p: %d y: %d" % (int(math.degrees(fusionPose[0])), 
        int(math.degrees(fusionPose[1])), int(math.degrees(fusionPose[2]))))
    #time.sleep(poll_interval*1.0/1000.0)
    n+= 1
   # if(time.perf_counter()-t1 >= 1):
    #    print(n)
     #   print(time.perf_counter() - t1)
      #  while(1):
       #     pass
        
