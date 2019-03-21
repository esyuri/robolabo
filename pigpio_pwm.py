#50~95にした時５４から５９の範囲で変わる


#!/usr/bin/python
# -*- coding:utf-8 -*-

import pigpio
import time

pwm_pin = 27

pi = pigpio.pi()
def setup_PWM(pin, pwm_frequency, max_microsec):
	pi.set_mode(pin, pigpio.OUTPUT)
	pi.set_PWM_frequency(pin, pwm_frequency)
	pi.set_PWM_range(pin, max_microsec)

setup_PWM(pwm_pin, 500, 100 )

while True:
	try:
		c=input("0から100までの数：")
		pi.set_PWM_dutycycle(pwm_pin, float(c))
		print(pi.get_PWM_frequency(pwm_pin))
		print(pi.get_PWM_dutycycle(pwm_pin))
		
		
	except KeyboardInterrupt:
		break


pi.set_PWM_dutycycle(pwm_pin, 0)
