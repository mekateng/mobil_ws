#!/usr/bin/env python
# -*- coding: utf-8 -*-

#First entered port is always Serial One. Switch ports in case of need.

import rospy
import serial
import time
from std_msgs.msg import String
import rosparam


serialString1='/dev/mobil_serial'
serialMsg=' '

#get data from pc
def serialCallback(data):
    global serialMsg
    serialMsg = data.data


def letsSerial():
    rospy.init_node("serial_rover")
    global serialMsg
    global serialString1
    rate = rospy.Rate(30)

    rospy.Subscriber("/mobil_serial_topic", String, serialCallback)
    
    sensor_pub = rospy.Publisher("/mobil_serial_encoder", String, queue_size=10)
    
    printOnce = True

    while not rospy.is_shutdown():

        print("")
        print("")
        print("Serial port 1: " + serialString1)

            #1 port open.
        if serialString1 != '/dev/ttyUSB':
			ser = serial.Serial(port=serialString1, baudrate=int(9600), parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)  # open serial
			ser.timeout = 1
			while ser.isOpen() and not rospy.is_shutdown():
				if printOnce == True :
					print( "serial1 is open")
					printOnce = False

				receive = ser.readline()
				#rate.sleep()
				ser.writelines( serialMsg+"\n")

            #There is no data that goes to the rover from pc. So there is no other writelines.

				sensor_pub.publish(receive)

				ser.flushInput()
				ser.flushOutput()
				print( "Reading from 1: " + str(receive) + "Writing this to 1: " +  str(serialMsg))
                                rate.sleep()
			rospy.spin()




if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass
