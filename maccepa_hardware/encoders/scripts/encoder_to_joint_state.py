#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32
from maccepa_msgs.msg import EncRawData
import math
import serial
import numpy as np
from collections import deque
from scipy import zeros, signal, random
from threading import Lock

NODE_NAME = "encoder_to_joint_state"
 
class EncoderRawData:
    def __init__(self):
        self.pos = 0
        self.error_code = 0
        self.time = 0
        self.mutex = Lock()
        self.read_status = 0
    
    def setData(self, pos, error, time):
        self.mutex.acquire()

        self.pos = pos
        self.error_code = error
        self.time = time
        self.read_status = 1

        self.mutex.release()
    
    def getData(self):
        if(self.read_status == 0):
            print("read data error")

        self.mutex.acquire()
        pos = self.pos
        error = self.error_code
        time = self.time

        self.mutex.release()

        self.read_status = 0

        return pos, error, time

    def isDataReady(self):
        if self.read_status == 1:
            return 1
        return 0

class Encoder:
    def __init__(self, offset):
        self.max_count = 522239
        self.offset = offset

        self.prev_pos = 0.
        self.prev_velocity = 0.
        self.prev_time = 0.
        self.us_in_sec = 1000000.

        self.b, self.a = signal.butter(4, 0.3, 'low')
        self.z = signal.lfilter_zi(self.b, self.a)
        
    def getPosition(self, raw_position, error_code, time):
        if(error_code == 1):
            return self.prev_pos + self.prev_velocity * (time - self.prev_time) / self.us_in_sec

        enc = raw_position-self.offset
        if(enc > self.max_count / 2):
            enc = - self.max_count + enc

        return - float(enc) / self.max_count * 2 * math.pi

    def getVelocity(self, actual_pos, time):
        delta_pos = actual_pos - self.prev_pos

        if(abs(delta_pos) > 5):
            delta_pos = delta_pos - np.sign(delta_pos) * 2 * math.pi
        
        velocity = delta_pos / (time - self.prev_time) * self.us_in_sec; 

        if(abs(velocity - self.prev_velocity) > 10):
            velocity = self.prev_velocity

        
        filt_velocity, self.z = signal.lfilter(self.b, self.a, [velocity], zi=self.z)
        return filt_velocity[0]

    def update(self, actual_pos, actual_velocity, actual_time):
        self.prev_pos = actual_pos
        self.prev_velocity = actual_velocity
        self.prev_time = actual_time


def callback(data):
    global raw_data_list
    raw_data_list[0].setData(data.pose_1, data.error_1, data.time_1)
    raw_data_list[1].setData(data.pose_2, data.error_2, data.time_2)
    
def initMsgJointState():
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['shoulder', 'elbow']
    msg.position = [0,0]
    msg.velocity = [0,0]
    msg.effort = [0,0]

    return msg


def listener():
    rospy.init_node(NODE_NAME, anonymous=True)

    rospy.Subscriber("raw_encoders_data", EncRawData, callback)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(100) # 10hz

    b, a = signal.butter(4, 0.3, 'low')
    z = signal.lfilter_zi(b, a)

    msg = initMsgJointState()

    first_offset = rospy.get_param( NODE_NAME + "/first_encoder_offset")
    second_offset = rospy.get_param(NODE_NAME + "/second_encoder_offset")

    encoder_1 = Encoder(first_offset) #encoder value of zero position
    encoder_2 = Encoder(second_offset) #encoder value of zero position
    while not rospy.is_shutdown():
        if(raw_data_list[0].isDataReady() == 1):
            pos1, error1, time1 = raw_data_list[0].getData()
            msg.position[0] = encoder_1.getPosition(pos1, error1, time1)
            msg.velocity[0] = encoder_1.getVelocity(msg.position[0], time1)
            encoder_1.update(msg.position[0],msg.velocity[0],time1)

        if(raw_data_list[1].isDataReady() == 1):
            pos2, error2, time2 = raw_data_list[1].getData()
            msg.position[1] = encoder_2.getPosition(pos2, error2,time2)
            msg.velocity[1] = encoder_2.getVelocity(msg.position[1], time2)
            encoder_2.update(msg.position[1],msg.velocity[1],time2)

        pub.publish(msg)
        rate.sleep()

raw_data_list = [EncoderRawData(), EncoderRawData()]

if __name__ == '__main__':
    listener()