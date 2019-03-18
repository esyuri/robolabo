# -*- coding: utf-8 -*-
#!/usr/bin/python

import waiacc
import Madgwick2
import ctrl
from time import sleep
import time

madgwickAHRS = Madgwick2.MadgwickAHRS(1/64, Madgwick2.Quaternion(1, 0, 0, 0), 1)
PIDctrl = ctrl.ctrl()


sample_time = 1/64
current_time = time.time()
last_time = current_time

while 1:
	current_time = time.time()
	delta_time = current_time - last_time
	if delta_time<1/64:
		continue
		
	
	accel = waiacc.get_accel_data()
	gyro = waiacc.get_gyro_data()
	#print(accel + gyro)
	#madgwickAHRS.update_imu(gyro, accel)
	rx,ry,rz=madgwickAHRS.update_imu(gyro, accel)
	PIDctrl.motor(rx,ry,rz)
	motor_power=PIDctrl.motorP
	print(motor_power)
	

	print(delta_time)
	
	last_time=current_time
	


