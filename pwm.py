#50~95にした時５４から５９の範囲で変わる



#!/usr/bin/python
# -*- coding:utf-8 -*-

import pigpio
import time

pwm_pin1 = 19
#pwm_pin2 = 20
pwm_pin3 = 27
pwm_pin4 = 24

pi = pigpio.pi()
def setup_PWM(pin, pwm_frequency, max_microsec):
	pi.set_mode(pin, pigpio.OUTPUT)
	pi.set_PWM_frequency(pin, pwm_frequency)
	pi.set_PWM_range(pin, max_microsec)

setup_PWM(pwm_pin1, 500, 100 )
#setup_PWM(pwm_pin2, 500, 100 )
setup_PWM(pwm_pin3, 500, 100 )
setup_PWM(pwm_pin4, 500, 100 )

pi.set_PWM_dutycycle(pwm_pin1, 56.5)
#pi.set_PWM_dutycycle(pwm_pin2, 56.5)
pi.set_PWM_dutycycle(pwm_pin3, 56.5)
pi.set_PWM_dutycycle(pwm_pin4, 56.5)

while True:
	try:
		c=input("0から100までの数：")
		pi.set_PWM_dutycycle(pwm_pin1, float(c))
		#pi.set_PWM_dutycycle(pwm_pin2, float(c))
		pi.set_PWM_dutycycle(pwm_pin3, float(c))
		pi.set_PWM_dutycycle(pwm_pin4, float(c))
		print(pi.get_PWM_frequency(pwm_pin1))
		print(pi.get_PWM_dutycycle(pwm_pin1))
		
		
	except KeyboardInterrupt:
		break


pi.set_PWM_dutycycle(pwm_pin1, 0)
#pi.set_PWM_dutycycle(pwm_pin2, 0)
pi.set_PWM_dutycycle(pwm_pin3, 0)
pi.set_PWM_dutycycle(pwm_pin4, 0)
