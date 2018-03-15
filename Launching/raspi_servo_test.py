import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
pwm_pin = 18
GPIO.setup(pwm_pin, GPIO.OUT)
pwm=GPIO.PWM(pwm_pin, 50)
pwm.start(0)

for i in range(360)
	SetAngle(45,i) 
	print(i)
	
	
pwm.stop()



def SetAngle(angle,i):
	duty = angle / (i)
	GPIO.output(pwm_pin, True)
	pwm.ChangeDutyCycle(duty)
	time.sleep(1)
	GPIO.output(pwm_pin, False)
	pwm.ChangeDutyCycle(0)