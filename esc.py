#センサーから計算までの処理速度およそ0.013


import RPI.GPIO as GPIO
from time import sleep
import sys


### 姿勢制御の計算準備部分
import waiacc
import Madgwick2
import ctrl
import time

madgwickAHRS = Madgwick2.MadgwickAHRS(1/256, Madgwick2.Quaternion(1, 0, 0, 0), 1)
PIDctrl = ctrl.ctrl()
s=time.time()
###

ESC_PORT1 = 16 #ポート番号
ESC_PORT2 = 19
ESC_PORT3 = 20
ESC_PORT4 = 26
TONE = 500 #周波数の指定

# GPIOの設定 各ポートを操作可能にしてる
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PORT1, GPIO.OUT)
GPIO.setup(ESC_PORT2, GPIO.OUT)
GPIO.setup(ESC_PORT3, GPIO.OUT)
GPIO.setup(ESC_PORT4, GPIO.OUT)
#PWMを使う準備
pwm1=GPIO.PWM(ESC_PORT1,TONE)
pwm2=GPIO.PWM(ESC_PORT2,TONE)
pwm3=GPIO.PWM(ESC_PORT3,TONE)
pwm4=GPIO.PWM(ESC_PORT4,TONE)



#ここから開始

#デューティー比75%で再生
pwm1.start(75)
pwm2.start(75)
pwm3.start(75)
pwm4.start(75)

while True:
	try:
		#制御量を計算
		accel = waiacc.get_accel_data()
		gyro = waiacc.get_gyro_data()
		#print(accel + gyro)
		#madgwickAHRS.update_imu(gyro, accel)
		rx,ry,rz=madgwickAHRS.update_imu(gyro, accel)
		PIDctrl.motor(rx,ry,rz)
		motor_power=PIDctrl.motorP
		print(motor_power)
		
		#計算値に合わせてパルス幅の割合を変化
		pwm1.ChangeDutyCycle(motor_power[0])
		pwm2.ChangeDutyCycle(motor_power[1])
		pwm3.ChangeDutyCycle(motor_power[2])
		pwm4.ChangeDutyCycle(motor_power[3])

		
		
	except KeyboardInterrupt:
		break

#停止
pwm1.stop()
pwm2.stop()
pwm3.stop()
pwm4.stop()
GPIO.cleanup()
sys.exit()
