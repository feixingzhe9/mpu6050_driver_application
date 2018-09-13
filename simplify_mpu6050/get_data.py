#!/usr/bin/env python
# coding=utf-8

import sys
from fcntl import ioctl
import array
import ctypes
import time


#define SMPLRT_DIV      0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG          0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG     0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG        0x1C    //加速计自检、测量范围及高通滤波，典型值：0x18(不自检，2G，5Hz)
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47    //陀螺仪z轴角速度数据寄存器（高位）
#define GYRO_ZOUT_L     0x48    //陀螺仪z轴角速度数据寄存器（低位）
#define PWR_MGMT_1      0x6B    //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I        0x75    //IIC地址寄存器(默认数值0x68，只读)
#define SlaveAddress        0x68    //MPU6050-I2C地址寄存器
#define W_FLG           0
#define R_FLG           1

#    mpu6050_write_byte(client, PWR_MGMT_1, 0x00);
#    mpu6050_write_byte(client, SMPLRT_DIV, 0x07);
#    mpu6050_write_byte(client, CONFIG, 0x06);
#    mpu6050_write_byte(client, GYRO_CONFIG, 0x18);
#    mpu6050_write_byte(client, ACCEL_CONFIG, 0x0);

GRA_ACC = 9.7964    # gravitational acceleration in Shanghai

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

IOCTL_WRITE = 0x80000000
IOCTL_READ = 0x00


ioctl_buf = array.array('B', [0])
fd = 0

def open_mpu6050():
    fd = open("/dev/mpu60500")
    return fd

def init_mpu6050(fd):
    global ioctl_buf
    ioctl_buf[0] = 0x00
    ioctl(fd, PWR_MGMT_1 | IOCTL_WRITE, ioctl_buf, 1)

    ioctl_buf[0] = 0x07
    ioctl(fd, SMPLRT_DIV | IOCTL_WRITE, ioctl_buf, 1)

    ioctl_buf[0] = 0x06
    ioctl(fd, CONFIG | IOCTL_WRITE, ioctl_buf, 1)

    ioctl_buf[0] = 0x18
    ioctl(fd, GYRO_CONFIG | IOCTL_WRITE, ioctl_buf, 1)

    ioctl_buf[0] = 0x00
    ioctl(fd, ACCEL_CONFIG | IOCTL_WRITE, ioctl_buf, 1)

class MPU6050_data(object):
    def __init__(self, _fd):
        self.fd = _fd
        ioctl_buf = array.array('B', [0])

    def build_value_by_reg_addr(self, addr_h, addr_l):
        ioctl_buf[0] = 0

        ioctl(fd, addr_h, ioctl_buf, 1)
        h = ioctl_buf[0]
        ioctl_buf[0] = 0

        ioctl(fd, addr_l, ioctl_buf, 1)
        l = ioctl_buf[0]

        return ctypes.c_int16(l).value + ctypes.c_int16(h<<8).value

    def get_temperature(self):
        temp = self.build_value_by_reg_addr(65, 66)
        temp = ctypes.c_float(temp).value/340 + ctypes.c_float(36.53).value
        return temp

    def get_acc_x_origin(self):
        return self.build_value_by_reg_addr(59, 60)

    def get_acc_y_origin(self):
        return self.build_value_by_reg_addr(61, 62)

    def get_acc_z_origin(self):
        return self.build_value_by_reg_addr(63, 64)

    def get_gyro_x_origin(self):
        return self.build_value_by_reg_addr(67, 68)

    def get_gyro_y_origin(self):
        return self.build_value_by_reg_addr(69, 70)

    def get_gyro_z_origin(self):
        return self.build_value_by_reg_addr(71, 72)

    def get_gyro_x(self):
        return ctypes.c_float(self.get_gyro_x_origin()).value/131

    def get_gyro_y(self):
        return ctypes.c_float(self.get_gyro_y_origin()).value/131

    def get_gyro_z(self):
        return ctypes.c_float(self.get_gyro_z_origin()).value/131


    def get_acc_x(self):
        global GRA_ACC
        return (ctypes.c_float(self.get_acc_x_origin()).value/16384) * GRA_ACC

    def get_acc_y(self):
        global GRA_ACC
        return (ctypes.c_float(self.get_acc_y_origin()).value/16384) * GRA_ACC

    def get_acc_z(self):
        global GRA_ACC
        return (ctypes.c_float(self.get_acc_z_origin()).value/16384) * GRA_ACC


def kalman_filter(data_in): #test
    data_out = data_in
    return data_out

def main():
    global fd
    fd = open_mpu6050()
    if fd < 0:
        print "can not open /dev/mpu6050 !"
    else:
        print "open mpu6050 sucessfully"

    init_mpu6050(fd)
    time.sleep(0.1)
    acc = MPU6050_data(fd)

    while 1:
        temperature = kalman_filter(acc.get_temperature())
        acc_x = kalman_filter(acc.get_acc_x())
        acc_y = kalman_filter(acc.get_acc_y())
        acc_z = kalman_filter(acc.get_acc_z())
        gyro_x = kalman_filter(acc.get_gyro_x())
        gyro_y = kalman_filter(acc.get_gyro_y())
        gyro_z = kalman_filter(acc.get_gyro_z())
        print "get temperature  : ", temperature
        print "get acc_X        : ",acc_x , " m/s2"
        print "get acc_Y        : ",acc_y , " m/s2"
        print "get acc_Z        : ",acc_z , " m/s2"
        print "get gyro_X       : ",gyro_x, "°/s"
        print "get gyro_Y       : ",gyro_y, "°/s"
        print "get gyro_Z       : ",gyro_z, "°/s"

        #print "gra_acc :",pow(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2), float(1)/2)
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except Exception:
        print sys.exc_info()
        exit(1)

