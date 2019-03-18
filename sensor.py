# -*- coding: utf-8 -*-
#!/usr/bin/python

import waiacc
import Madgwick2
from time import sleep




madgwickAHRS = Madgwick2.MadgwickAHRS(1/64, Madgwick2.Quaternion(1, 0, 0, 0), 1)




while 1:
	accel = waiacc.get_accel_data()
	gyro = waiacc.get_gyro_data()
	#print(accel + gyro)
	#madgwickAHRS.update_imu(gyro, accel)
	rx,ry,rz=madgwickAHRS.update_imu(gyro, accel)
	print('\n',rx,'\n',ry,'\n',rz,'\n') 
	sleep(1/64)

    

