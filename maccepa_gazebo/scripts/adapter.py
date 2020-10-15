#!/usr/bin/env python
import rospy
from maccepa_msgs.msg import MotorsPose
from maccepa_msgs.msg import MotorsVel
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# Adapter:
#   - takes MotorsPose message from specified topics
#   - breaks it into separate blocks
#   - publish them to each motor topic independently

global CMD_PUBS
global motors_state
MODE_POSITION = 1
MODE_VELOCITY = 2

# represents single Motor instance
class Motor:
    def __init__(self):
        self.pos = 0.0

    def getPosition(self):
        return self.pos

    def setPosition(self, pos):
        self.pos = pos


# represents single maccepa actuator instance
# consists of 2 Motors
class MaccepaActuator:
    def __init__(self):
        self.cardioid = Motor()
        self.pretension = Motor()

    def getPosition(self):
        return [self.cardioid.getPosition(),
                self.pretension.getPosition()]

    def setPosition(self, cardioid_position, pretension_position):
        self.cardioid.setPosition(cardioid_position)
        self.pretension.setPosition(pretension_position)


def command_callback(data, mode):
    global CMD_PUBS
    if mode == MODE_POSITION:
        CMD_PUBS[0].publish(data.motor_1_pose)
        CMD_PUBS[1].publish(data.motor_2_pose)
        CMD_PUBS[2].publish(data.motor_3_pose)
        CMD_PUBS[3].publish(data.motor_4_pose)
    elif mode == MODE_VELOCITY:
        CMD_PUBS[0].publish(data.motor_1_vel)
        CMD_PUBS[1].publish(data.motor_2_vel)
        CMD_PUBS[2].publish(data.motor_3_vel)
        CMD_PUBS[3].publish(data.motor_4_vel)


def state_callback(data, index):
    maccepaActuators[index].setPosition(
        data.position[data.name.index('cardioid')],
        data.position[data.name.index('pretension')])


def main():
    rospy.init_node('maccepa_gazebo_adapter', anonymous=False)

    # command topics adapter logic
    # parse params
    command_position_topic_name = rospy.get_param('~command_position_topic')
    command_velocity_topic_name = rospy.get_param('~command_velocity_topic')
    motor_1_topic_name = rospy.get_param('~motor_1_topic')
    motor_2_topic_name = rospy.get_param('~motor_2_topic')
    motor_3_topic_name = rospy.get_param('~motor_3_topic')
    motor_4_topic_name = rospy.get_param('~motor_4_topic')

    # create subscriber and publishers for command topics
    # bind position and velocity topics to the same callback
    rospy.Subscriber(command_position_topic_name, MotorsPose, command_callback, MODE_POSITION)
    rospy.Subscriber(command_velocity_topic_name, MotorsVel, command_callback, MODE_VELOCITY)
    global CMD_PUBS
    CMD_PUBS = [
        rospy.Publisher(motor_1_topic_name, Float64, queue_size=10),
        rospy.Publisher(motor_2_topic_name, Float64, queue_size=10),
        rospy.Publisher(motor_3_topic_name, Float64, queue_size=10),
        rospy.Publisher(motor_4_topic_name, Float64, queue_size=10),
    ]

    # state topics adapter logic
    state_topic_rate = rospy.Rate(1000)
    # parse params
    motors_state_topic_name = rospy.get_param('~motors_state_topic')
    maccepa_plugin_1_state_topic_name = rospy.get_param('~maccepa_plugin_1_state_topic')
    maccepa_plugin_2_state_topic_name = rospy.get_param('~maccepa_plugin_2_state_topic')

    # create subscribers and publisher for state topics
    rospy.Subscriber(maccepa_plugin_1_state_topic_name, JointState, state_callback, 0)
    rospy.Subscriber(maccepa_plugin_2_state_topic_name, JointState, state_callback, 1)
    maccepa_state_pub = rospy.Publisher(motors_state_topic_name, MotorsPose, queue_size=10)

    rospy.loginfo("[MACCEPA_GAZEBO_ADAPTER] node loaded.")
    # in loop publish message to the state topic
    msg = MotorsPose()
    while not rospy.is_shutdown():
        msg.motor_1_pose = maccepaActuators[0].getPosition()[0]
        msg.motor_2_pose = maccepaActuators[0].getPosition()[1]
        msg.motor_3_pose = maccepaActuators[1].getPosition()[0]
        msg.motor_4_pose = maccepaActuators[1].getPosition()[1]
        maccepa_state_pub.publish(msg)
        try:
            state_topic_rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            pass


# create empty maccepa actuators
maccepaActuators = [MaccepaActuator(), MaccepaActuator()]

if __name__ == '__main__':
    main()
