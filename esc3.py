
#範囲は50~95
#54~66で稼働


#センサーから計算までの処理速度およそ0.013
""""
ESCの配線
茶：グラウンド
赤：VCC
黃：信号　
"""


import pigpio
import time
import sys

pi = pigpio.pi()

### 姿勢制御の計算準備部分
import waiacc
import Madgwick2
import ctrl3
#import time

madgwickAHRS = Madgwick2.MadgwickAHRS(1/256, Madgwick2.Quaternion(1, 0, 0, 0), 1)
PIDctrl = ctrl3.ctrl()
sample_time = 1/64
current_time = time.time()
last_time = current_time
###

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

pi.set_PWM_dutycycle(pwm_pin1, 60)
#pi.set_PWM_dutycycle(pwm_pin2, 60)
pi.set_PWM_dutycycle(pwm_pin3, 60)
pi.set_PWM_dutycycle(pwm_pin4, 60)


while True:
	try:
		current_time = time.time()
		delta_time = current_time - last_time
		if delta_time<1/64:
			continue
		

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
		
		pi.set_PWM_dutycycle(pwm_pin1, float(motor_power[0]))
		#pi.set_PWM_dutycycle(pwm_pin2, float(motor_power[1]))
		pi.set_PWM_dutycycle(pwm_pin3, float(motor_power[1]))
		pi.set_PWM_dutycycle(pwm_pin4, float(motor_power[2]))
		

		print(delta_time)
	
		last_time=current_time
		
	except KeyboardInterrupt:
		break

#停止
pi.set_PWM_dutycycle(pwm_pin1, 0)
#pi.set_PWM_dutycycle(pwm_pin2, 0)
pi.set_PWM_dutycycle(pwm_pin3, 0)
pi.set_PWM_dutycycle(pwm_pin4, 0)
sys.exit()

