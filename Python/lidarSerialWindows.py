# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 18:32:36 2017

@author: fengqiang
Copyright (c) Benewake LLC.  All rights reserved.

Modified by Chris Anderson, DIY Robocars
"""

# Support Python 3.x
# It requires that the "pyserial" library has been installed already. To do that enter "pip3 install pyserial" on command line.
# Confusingly, do NOT install the "serial" library. If need be enter "pip3 uninstall serial" before installing pyserial


import serial
import time
class serialPort:
    def __init__(self):
        self.baudrate = 115200
        self.portName = "COM26"
        self.port = None
                   
    def setBardRate(self, baudRate):
        self.baudrate = baudRate

    def openPort(self, portName):   
        if self.portName != portName and self.port != None:
            self.port.close()
            self.port = None
        self.portName = portName    
        if self.port == None:
            print (self.portName, self.baudrate)
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
            if ret == 0x90:
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
            if ret == 0x90:
                break
            time.sleep(0.1)
    
    def lidarRead(self):
        while True:
            data = self.readPort(2)
            # Check frame header
            if data[0] != 0xaa or data[1] != 0x55:
                print (data)
                continue
            else:
                data = self.readPort(2)
                # Inspection length
                if data[0] != 0x01 and data[0] != 0x03:
                    continue
                else:
                    nPoints = data[0]
                    break
        # Reading a point's distance and angle data         
        data = self.readPort(3 * nPoints)
        Z = data[0] | (data[1] << 8)
        theta = data[2]
        if theta & 0x80:
            theta = -1 * (((~theta) + 1) & 0xff)
 #       print ("min =",Z,theta)
        if nPoints == 3:
            Z1 = data[3] | (data[4] << 8)
            theta1 = data[5]
            if theta1 & 0x80:
                theta1 = -1 * (((~theta1) + 1) & 0xff)
            Z2= data[6] | (data[7] << 8)
            theta2 = data[8]
            if theta2 & 0x80:
                theta2 = -1 * (((~theta2) + 1) & 0xff)        
            print ("left =",Z1,theta1)
            print ("right =",Z2,theta2)
        print ("\n")
        return (Z, theta)
        
    def lidarReadLine(self):
        Z = []
        theta = []
        while True:
            data = self.readPort(2)
            # Check frame header
            if data[0] != 0xaa or data[1] != 0x55:
                continue
            else:
                data = self.readPort(2)
                # Inspection length
                if data[0] != 0x40 or data[1] != 0x01:
                    continue
                else:
                    break
        # Reading 320 points of distance and angle data      
        data = self.readPort(3 * 320)
        for i in range(320):            
            Z.append(data[3 * i + 0] | (data[3 * i + 1] << 8))
            theta_tmp = data[2]
            if theta_tmp & 0x80:
                theta_tmp = -1 * (((~theta_tmp) + 1) & 0xff)
            theta.append(theta_tmp)
        return (Z, theta)        
        
         
if __name__ == "__main__":

    port = serialPort()
    port.openPort("COM26")
    port.lidarStart(30, 120, 10)
    while True:
        data = port.lidarRead()
        print (data[1])

