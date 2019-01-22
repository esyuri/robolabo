# LSM330DLC 加速度ジャイロセンサ

# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
from time import sleep

# i2c device address
dev_addr_a = 0x18
dev_addr_g = 0x6a

# register address
# acceleromete
ctrl_reg1_a = 0x20
ctrl_reg2_a = 0x21
ctrl_reg3_a = 0x22
ctrl_reg4_a = 0x23
ctrl_reg5_a = 0x24

accel_xout = 0x28  # 0x28(L)と0x29(H)で表現
accel_yout = 0x2a
accel_zout = 0x2c

# gyro
ctrl_reg1_g = 0x20
ctrl_reg2_g = 0x21
ctrl_reg3_g = 0x22
ctrl_reg4_g = 0x23
ctrl_reg5_g = 0x24

gyro_xout = 0x28  # 0x28(L)と0x29(H)で表現
gyro_yout = 0x2a
gyro_zout = 0x2c


bus = smbus.SMBus(1)  # 1or2,i2cdetectが成功する方

# setup
# acceleromete
bus.write_byte_data(dev_addr_a, ctrl_reg1_a, 0b10010111)
bus.write_byte_data(dev_addr_a, ctrl_reg2_a, 0b00000000)
bus.write_byte_data(dev_addr_a, ctrl_reg3_a, 0b00001000)
bus.write_byte_data(dev_addr_a, ctrl_reg4_a, 0b00001000) #1mg/digit
bus.write_byte_data(dev_addr_a, ctrl_reg5_a, 0b01000000)
# gyro
bus.write_byte_data(dev_addr_g, ctrl_reg1_g, 0b00001111)
bus.write_byte_data(dev_addr_g, ctrl_reg2_g, 0b00000000)
bus.write_byte_data(dev_addr_g, ctrl_reg3_g, 0b00000000)
bus.write_byte_data(dev_addr_g, ctrl_reg4_g, 0b00000000) #8.75mdps/digit
bus.write_byte_data(dev_addr_g, ctrl_reg5_g, 0b00000000)


def read_raw_data(dev_addr, reg_addr):
    low = bus.read_byte_data(dev_addr, reg_addr)   # 1byte = 8bit
    high = bus.read_byte_data(dev_addr, reg_addr+1)
    raw_data = (high << 8) | low  #16bit
    # (2^16=) 65536 /2 = 32768
    if raw_data > 32767:  # ０から数える
        # minus
        raw_data -= 65536
    return raw_data

def get_accel_data():
    raw_a_x = read_raw_data(dev_addr_a, accel_xout) >> 4  # why?
    raw_a_y = read_raw_data(dev_addr_a, accel_yout) >> 4
    raw_a_z = read_raw_data(dev_addr_a, accel_zout) >> 4
    a_x = raw_a_x * 0.001  # 1mg/digit
    a_y = raw_a_y * 0.001
    a_z = raw_a_z * 0.001
    return [a_x, a_y, a_z]

def get_gyro_data():
    raw_g_x = read_raw_data(dev_addr_g, gyro_xout)
    raw_g_y = read_raw_data(dev_addr_g, gyro_yout)
    raw_g_z = read_raw_data(dev_addr_g, gyro_zout)
    g_x = raw_g_x * 0.00875  # 8.75mdps/digit 一秒あたりのミリ度
    g_y = raw_g_y * 0.00875
    g_z = raw_g_z * 0.00875
    
    g_x = g_x * 3.1415/180000  # ラジアンに変換
    g_y = g_y * 3.1415/180000
    g_z = g_z * 3.1415/180000
    
    return [g_x, g_y, g_z]

while 1:
    print(get_accel_data() + get_gyro_data())
    sleep(1)
