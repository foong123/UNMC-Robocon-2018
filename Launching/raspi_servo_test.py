import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
pwm=GPIO.PWM(13, 50)
pwm.start(0)

for i in range(360)
	SetAngle(90,i) 
	print(i)
	time.sleep(1)
	
pwm.stop()



def SetAngle(angle,i):
	duty = angle / (i)
	GPIO.output(13, True)
	pwm.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(13, False)
	pwm.ChangeDutyCycle(0)