# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 18:32:36 2017

@author: fengqiang
Copyright (c) Benewake LLC.  All rights reserved.
"""

import serial
import time
import motor
import car_dir as steer
import RPi.GPIO as GPIO

busnum = 1          # Edit busnum to 0 for Raspberry Pi 1. For RPi 2 and above, use 1
forward0 = 'True'
forward1 = 'False'

class serialPort:
    def __init__(self):
        self.baudrate = 115200
        self.portName = "/dev/ttyUSB1"
        self.port = None

    def setBardRate(self, baudRate):
        self.baudrate = baudRate

    def openPort(self, portName):
        if self.portName != portName and self.port != None:
            self.port.close()
            self.port = None
        self.portName = portName
        if self.port == None:
            print self.portName, self.baudrate
            self.port = serial.Serial(port = self.portName, baudrate = self.baudrate, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1)

    def closePort(self):
        self.port.close()
        self.port = None

    def readPort(self, length):
        return self.port.read(length)

    def writePort(self, data):
        self.port.write(data)

    def clearPort(self):
        try:
            self.port.flush()
            self.port.read(100)
        except:
            return

    def lidarStart(self, width, depth, fps, protocol = 0):
        data = [0xaa, 0x55, 0x01 | (protocol << 5), width & 0xff, (width >> 8) & 0xff, depth & 0xff, (depth >> 8) & 0xff, fps]
        for i in range(20):
            self.writePort(data)
            ret = self.readPort(1)
            if ord(ret) == 0x90:
                ret = 0
                break
            elif ord(ret) == 0x91:
                ret = -1
            else:
                ret = -2
        return ret

    def lidarStop(self):
        data = [0xaa, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        while True:
            self.clearPort()
            self.writePort(data)
            ret = self.readPort(1)
            if ord(ret) == 0x90:
                break
            time.sleep(0.1)

    def lidarRead(self):
        while True:
            data = self.readPort(2)
            # 检查帧头
            if ord(data[0]) != 0xaa or ord(data[1]) != 0x55:
                print data
                continue
            else:
                data = self.readPort(2)
                # 检查长度
                if (ord(data[0]) != 0x01 and ord(data[0]) != 0x03) :
                    continue
                else:
                    nPoints = ord(data[0])
                    break
        # 读取一个点的距离和角度数据
        data = self.readPort(3 * nPoints)
        Z = ord(data[0]) | (ord(data[1]) << 8)
        theta = ord(data[2])
        if theta & 0x80:
            theta = -1 * (((~theta) + 1) & 0xff)
        print "min =",Z,theta
        if nPoints == 3:
            Z1 = ord(data[3]) | (ord(data[4]) << 8)
            theta1 = ord(data[5])
            if theta1 & 0x80:
                theta1 = -1 * (((~theta1) + 1) & 0xff)
            Z2= ord(data[6]) | (ord(data[7]) << 8)
            theta2 = ord(data[8])
            if theta2 & 0x80:
                theta2 = -1 * (((~theta2) + 1) & 0xff)
            print "left =",Z1,theta1
            print "right =",Z2,theta2
        print "\n"
        return (Z, theta)

    def lidarReadLine(self):
        Z = []
        theta = []
        while True:
            data = self.readPort(2)
            # 检查帧头
            if ord(data[0]) != 0xaa or ord(data[1]) != 0x55:
                continue
            else:
                data = self.readPort(2)
                # 检查长度
                if ord(data[0]) != 0x40 or ord(data[1]) != 0x01:
                    continue
                else:
                    break
        # 读取320个点的距离和角度数据
        data = self.readPort(3 * 320)
        for i in range(320):
            Z.append(ord(data[3 * i + 0]) | (ord(data[3 * i + 1]) << 8))
            theta_tmp = ord(data[2])
            if theta_tmp & 0x80:
                theta_tmp = -1 * (((~theta_tmp) + 1) & 0xff)
            theta.append(theta_tmp)
        return (Z, theta)


if __name__ == "__main__":

    port = serialPort()
    motor.setup(busnum=busnum)
    steer.setup(busnum=busnum)
    print 'motor moving forward'
    motor.setSpeed(50)
    motor.motor0(forward0)
    motor.motor1(forward1)
    steer.turn(15)


    port.openPort("/dev/ttyUSB1")
    port.lidarStart(320, 320, 2)
    while True:
        print "read data"
        print port.lidarRead()
#    port = serial.Serial(port = 'COM10',
#                         baudrate = 115200,
#                         bytesize = serial.EIGHTBITS,
#                         parity = serial.PARITY_NONE,
#                         stopbits = serial.STOPBITS_ONE)
#    port.open()
#    print port.isOpen()
#
#    time.sleep(1)
#    port.write([0xaa, 0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02])
#    port.flush()
#    print ord(port.read(1))
#    port.close()
#    while True:
#        print "wait"
