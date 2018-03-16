import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
pwm_pin = 18
GPIO.setup(pwm_pin, GPIO.OUT)

for f in range(50,330,25)
	pwm=GPIO.PWM(pwm_pin, f)
	pwm.start(0)
	for i in range(100)
		SetAngle(i) 
		print(i)
	
	
pwm.stop()



def SetAngle(duty):
	#duty = angle / (i)
	GPIO.output(pwm_pin, True)
	pwm.ChangeDutyCycle(duty)# where 0.0 <= dc <= 100.0
	time.sleep(0.5)
	GPIO.output(pwm_pin, False)
	pwm.ChangeDutyCycle(0)