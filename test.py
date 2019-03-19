import RPi.GPIO as GPIO
from time import sleep
import sys
import time


#GPIO.cleanup()

ESC_PORT1 = 19 #ポート番号
TONE = 500 #周波数の指定

# GPIOの設定 各ポートを操作可能にしてる
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PORT1, GPIO.OUT)

#PWMを使う準備
pwm1=GPIO.PWM(ESC_PORT1,TONE)




pwm1.start(0)
#sleep(2)
while True:
	try:
		c=input("0から100までの数：")
		pwm1.ChangeDutyCycle(float(c))
		#pwm1.start(75)
		print(time.time())
		"""
		current_time = time.time()
		delta_time = current_time - last_time
		if delta_time<1/64:
			continue
		
		sleep(1/256)
		pwm1.stop()

		print(delta_time)
	
		last_time=current_time
		"""
	except KeyboardInterrupt:
		break


pwm1.stop()
GPIO.cleanup()
sys.exit()
