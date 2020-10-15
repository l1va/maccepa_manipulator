#!/usr/bin/env python

import rospy
from maccepa_msgs.msg import EncRawData
import serial

def talker():
    pub = rospy.Publisher('raw_encoders_data', EncRawData, queue_size=10)

    rospy.init_node('encoders_data_from_arduino', anonymous=True)
    port = rospy.get_param("raw_encoders_data/port_adress")
    ser = serial.Serial(port, 1000000)
    ser.readline()

    msg = EncRawData()

    while not rospy.is_shutdown():
        raw_data = [int(i) for i in ser.readline().strip().split(" ")]
        msg.pose_1 = raw_data[0]
        msg.error_1 = raw_data[1]
        msg.time_1 = raw_data[2]
        msg.pose_2 = raw_data[3]
        msg.error_2 = raw_data[4]
        msg.time_2 = raw_data[5]
        
        pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

