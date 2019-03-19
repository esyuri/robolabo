#1.1ms~1.9ms


import RPi.GPIO as GPIO
from time import sleep
import sys
import time

ESC_PORT1 = 16 #ポート番号
ESC_PORT2 = 19
ESC_PORT3 = 20
ESC_PORT4 = 26


# GPIOの設定 各ポートを操作可能にしてる
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PORT1, GPIO.OUT)
GPIO.setup(ESC_PORT2, GPIO.OUT)
GPIO.setup(ESC_PORT3, GPIO.OUT)
GPIO.setup(ESC_PORT4, GPIO.OUT)


current_time = time.time()
last_time = current_time

while True:
	try:
		#各ピンをONに
		GPIO.output(ESC_PORT1,GPIO.HIGH)
		GPIO.output(ESC_PORT2,GPIO.HIGH)
		GPIO.output(ESC_PORT3,GPIO.HIGH)
		GPIO.output(ESC_PORT4,GPIO.HIGH)
		c1=1
		c2=1
		c3=1
		c4=1
		cend=1
		
		
		ESClooptimer = time.time()
		ESC1_timer=0.001+ESClooptimer;
		"""
		ESC2_timer=0.001+ESClooptimer;
		ESC3_timer=0.0015+ESClooptimer;
		ESC4_timer=0.001+ESClooptimer;
		"""
		end_timer=0.02+ESClooptimer;
		
		#すべてのピンがOFFになるまで回す
		while c1+c2+c3+c4+cend>0:
			ESClooptimer = time.time()
			if ESC1_timer <= ESClooptimer:
				pwm1=GPIO.output(ESC_PORT1,GPIO.LOW)
				c1=0
			
			"""
			if ESC2_timer <= ESClooptimer:
				pwm2=GPIO.output(ESC_PORT2,GPIO.LOW)
				c2=0
			if ESC3_timer <= ESClooptimer:
				pwm3=GPIO.output(ESC_PORT3,GPIO.LOW)
				c3=0
			if ESC4_timer <= ESClooptimer:
				pwm4=GPIO.output(ESC_PORT4,GPIO.LOW)
				c4=0
			"""
			if end_timer <= ESClooptimer:
				cend=0
			
		
		"""
		c=input("0から100までの数：")
		pwm1.ChangeDutyCycle(float(c))
		#pwm1.start(75)
		print(time.time())
		sleep(1)
		"""
		
	except KeyboardInterrupt:
		break


GPIO.cleanup()
sys.exit()
