#!/usr/bin/env python
# license removed for brevity

#ROS
import rospy
#from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import SetLinkProperties, SetLinkPropertiesRequest


#Python
import numpy as np
import random
import datetime
import os
import time
from math import pi, sin
import pickle

Verbose = False
WriteLog = True

if WriteLog:
    dir = os.path.dirname(__file__)
    filepath = os.path.join(dir, 'log_' + str(datetime.datetime.now()) + '.log')
    f = open(filepath, "w+")


#TODO:
#rename all MACCEPA joints, motors and topics and variables related to the two maccepa motors, 
#such that there would only be 2 names, consistent across all files
#cardioid_motor and pretension_motor?

#try to make a model for static errors of the inner MACCEPA loop?

dof = 2

#topic parameters
publisher_ids = {}
publisher_pizza_id = {}

topicname_command = {1: '/maccepa_plugin/shoulder/cardioid_motor/command',
			         2: '/maccepa_plugin/shoulder/pretension_motor/command',
                     3: '/maccepa_plugin/elbow/cardioid_motor/command',
                     4: '/maccepa_plugin/elbow/pretension_motor/command'}


topicname_maccepa_shoulder_feedback = '/maccepa_plugin/shoulder/motors/state'
topicname_maccepa_elbow_feedback = '/maccepa_plugin/elbow/motors/state'

topicname_joint_states = '/maccepa/gazebo_joint_states'

topicname_set_model_state = '/gazebo/set_model_state'

#topicname_SetLinkProperties = '/gazebo/set_link_properties'

#MACCEPA
with open ('MACCEPA_res', 'rb') as fp:
    MACCEPA_res = pickle.load(fp)

cardioid_hard_limit = pi/4 

long_linearization_constant = 20
condition_threshold = 100

#trajectories
period = 31 #duration of a control cycle
	
#controller constants
Kp = np.eye(dof) * 20
Kd = np.eye(dof) * 7

#q_desired = np.array([np.nan, np.nan])
#v_desired = np.zeros((dof, ))
#k_desired_0 = np.array([np.nan, np.nan])


current_maccepa_shoulder_position = np.zeros((2, ))
current_maccepa_elbow_position    = np.zeros((2, ))

current_maccepa_shoulder_effort   = np.zeros((1, ))
current_maccepa_elbow_effort      = np.zeros((1, ))

current_joint_position            = np.zeros((dof, ))
current_joint_velocity            = np.zeros((dof, ))

################################################
################################################
# helper functions

def fix_q(q0):
    q = ( q0 + np.pi) % (2 * np.pi ) - np.pi
    return q

################################################
################################################
# MACCEPA MODEL

def get_index(phi1, phi2):
	
    if phi1 < -0.999*cardioid_hard_limit:
        phi1 = -0.999*cardioid_hard_limit
        
    if phi1 > 0.999*cardioid_hard_limit:
        phi1 = 0.999*cardioid_hard_limit
    
    index1 = (phi1 - MACCEPA_res['min_deflection_angle']) / MACCEPA_res['step_deflection_angle']
    index2 = (phi2 - MACCEPA_res['min_motor2_angle'])     / MACCEPA_res['step_motor2_angle']
	
    index1 = int(index1)
    index2 = int(index2)
	
    if index1 > MACCEPA_res['Count']:
        raise NameError('requested unsupported value of angle phi1')
    if index2 > MACCEPA_res['Count']:
        raise NameError('requested unsupported value of angle phi2')
	
    return index1, index2


def get_linear_model(phi1, phi2):
	
    index1, index2 = get_index(phi1, phi2)
    
    phi1_table  = MACCEPA_res['deflection_angle'][index1,   index2]
    phi2_table  = MACCEPA_res['motor2_angle']    [index1,   index2]
    
    #the table value would be a little off the actual value we want
    #so we find how far off we are and correct the value using linear approximation
    delta_phi1 = phi1 - phi1_table
    delta_phi2 = phi2 - phi2_table
    
    if index1 < (1.0 * MACCEPA_res['Count'] / 2):
        delta1 = 1
    else:
        delta1 = -1
    
    if index2 < (1.0 * MACCEPA_res['Count'] / 2):
        delta2 = 1
    else:
        delta2 = -1
    
    tau0  = MACCEPA_res['torque'][index1,        index2]
    tau11 = MACCEPA_res['torque'][index1+delta1, index2]
    tau12 = MACCEPA_res['torque'][index1,        index2+delta2]
    
    tau11_long = MACCEPA_res['torque'][index1+delta1*long_linearization_constant, index2]
    tau12_long = MACCEPA_res['torque'][index1,                                    index2+delta2*long_linearization_constant]
    
    T      = np.array([(tau11      - tau0) / (delta1*MACCEPA_res['step_deflection_angle']), 
                       (tau12      - tau0) / (delta2*MACCEPA_res['step_motor2_angle'])  ]) 
    
    T_long = np.array([(tau11_long - tau0) / (delta1*long_linearization_constant*MACCEPA_res['step_deflection_angle']), 
                       (tau12_long - tau0) / (delta2*long_linearization_constant*MACCEPA_res['step_motor2_angle'])])
    
    delta_tau = T.dot(np.array([delta_phi1, delta_phi2]))
    tau0 = tau0 + delta_tau
	
    k0  = MACCEPA_res['stiffness'][index1,        index2]
    k11 = MACCEPA_res['stiffness'][index1+delta1, index2]
    k12 = MACCEPA_res['stiffness'][index1,        index2+delta2]
    
    k11_long = MACCEPA_res['stiffness'][index1+delta1*long_linearization_constant, index2]
    k12_long = MACCEPA_res['stiffness'][index1,                                    index2+delta2*long_linearization_constant]
	
    K      = np.array([(k11      - k0) / (delta1*MACCEPA_res['step_deflection_angle']), 
                       (k12      - k0) / (delta2*MACCEPA_res['step_motor2_angle'])      ])
    
    K_long = np.array([(k11_long - k0) / (delta1*long_linearization_constant*MACCEPA_res['step_deflection_angle']), 
                       (k12_long - k0) / (delta2*long_linearization_constant*MACCEPA_res['step_motor2_angle'])     ])
	
    delta_k = K.dot(np.array([delta_phi1, delta_phi2]))
    k0 = k0 + delta_k
    
    return tau0, k0, T, T_long, K, K_long



################################################
################################################
# Control

def gravity_compensation(q1, q2):
    g = 9.81
    L1 = 0.25
    L2 = 0.25
    m1 = 1.0
    m2 = 1.0
    m3 = 0.5
    
    #generated in matlab
    #G = -np.array([(g*m2*((L2*sin(q1 + q2))/2 + L1*sin(q1)) + (L1*g*m1*sin(q1))/2), ((L2*g*m2*sin(q1 + q2))/2)])
    
    G = -np.array([(L2*sin(q1 + q2)*(m3 + m2/2)*g + L1*sin(q1)*(m3 + m2 + m1/2)*g),
                   (L2*sin(q1 + q2)*(m3 + m2/2)*g)])
    
    return G
 

def control_input(time_abs):
    
    #control input constants
    time_switch_1 = (0, 3)    #wait at the point A for the springs to wind up
    time_switch_2 = (3, 8)    #go from point A to point B
    time_switch_3 = (8, 11)   #wait at the point B for the springs to unwind
    time_switch_4 = (11, 14)  #catch pizza at the point B, wait for the springs to wind up
    time_switch_5 = (14, 19)  #go from point B to point C
    time_switch_6 = (19, 22)  #wait at the point C
    time_switch_7 = (22, 27)  #go from point C to point A
    time_switch_8 = (27, 30)  #wait at the point A for the springs to unwind
    
    pretension_init = pi
    pretension_max = 9*pi
    pretension_catch = 0.8*pi
    pretension_rate = (pretension_max - min(pretension_init, pretension_catch)) / (time_switch_1[1] - time_switch_1[0])
    
    PointA = np.array([0, 0])
    PointB = np.array([-pi/6, 2*pi/6])
    PointC = np.array([-1.5*pi/3, 2*pi/3])
    
    kp_max = 20
    kd_max = 7
    
    kp_min = 12
    kd_min = 4
    
    global Kp
    global Kd
        
    t = time_abs % period
    
    if (time_switch_1[0] < t) and (t < time_switch_1[1]):
        
        koef = (t - time_switch_1[0]) / (time_switch_1[1] - time_switch_1[0])
        q = PointA
        pretension = pretension_max * koef + pretension_init * (1 - koef)
        
        Kp = np.eye(dof) * kp_max
        Kd = np.eye(dof) * kd_max
        
    elif (time_switch_2[0] < t) and (t < time_switch_2[1]):
        
        koef = (t - time_switch_2[0]) / (time_switch_2[1] - time_switch_2[0])
        q = PointB*koef + PointA*(1 - koef)
        pretension = pretension_max
        
        Kp = np.eye(dof) * kp_max
        Kd = np.eye(dof) * kd_max
    
    elif (time_switch_3[0] < t) and (t < time_switch_3[1]):
        
        koef = (t - time_switch_3[0]) / (time_switch_3[1] - time_switch_3[0])
        q = PointB
        pretension = pretension_catch * koef + pretension_max * (1 - koef)
                
        Kp = np.eye(dof) * kp_min
        Kd = np.eye(dof) * kd_min
        
    elif (time_switch_4[0] < t) and (t < time_switch_4[1]):
        
        koef = (t - time_switch_4[0]) / (time_switch_4[1] - time_switch_4[0])
        q = PointB    
        pretension = pretension_max * koef + pretension_catch * (1 - koef)
        
        Kp = np.eye(dof) * kp_min
        Kd = np.eye(dof) * kd_min
        
    elif (time_switch_5[0] < t) and (t < time_switch_5[1]):
        
        koef = (t - time_switch_5[0]) / (time_switch_5[1] - time_switch_5[0])
        q = PointC*koef + PointB*(1 - koef)
        pretension = pretension_max
        
        Kp = np.eye(dof) * kp_max
        Kd = np.eye(dof) * kd_max
    
    elif (time_switch_6[0] < t) and (t < time_switch_6[1]):
        q = PointC
        pretension = pretension_max
        
        Kp = np.eye(dof) * kp_max
        Kd = np.eye(dof) * kd_max
        
    elif (time_switch_7[0] < t) and (t < time_switch_7[1]):
        
        koef = (t - time_switch_7[0]) / (time_switch_7[1] - time_switch_7[0])
        q = PointA*koef + PointC*(1 - koef)
        pretension = pretension_max
        
        Kp = np.eye(dof) * kp_max
        Kd = np.eye(dof) * kd_max
        
    elif (time_switch_8[0] < t) and (t < time_switch_8[1]):
        
        koef = (t - time_switch_8[0]) / (time_switch_8[1] - time_switch_8[0])
        q = PointA
        pretension = pretension_init * koef + pretension_max * (1 - koef)
        
        Kp = np.eye(dof) * kp_max
        Kd = np.eye(dof) * kd_max
        
    else:
        q = PointA 
        pretension = pretension_init
    
    v = np.zeros((dof, ))
    
    #print " "
    #print "**************************"
    #print "t: ", t
    #print "q, v, pretension: ", q, v, pretension
    #print "**************************"
    #print " "
    
    return q, v, pretension


def pizza_teleportation(time_abs):
    
    time_pizza_appears = (11, 11.2)
    time_pizza_disappears = (22, 22.2)
    
    t = time_abs % period
    
    ToTeleport = False
    if (time_pizza_appears[0] < t) and (t < time_pizza_appears[1]):
        pizza_position = np.array([-0.05, 0, 1.5])
        ToTeleport = True
    elif (time_pizza_disappears[0] < t) and (t < time_pizza_disappears[1]):
        pizza_position = np.array([-1, -1, 0])
        ToTeleport = True
    
    
    #print "t, time_abs, ToTeleport: ", t, time_abs, ToTeleport
    
    if ToTeleport:
        state = ModelState()
        state.pose = Pose()

        mu = 0.0
        sigma = 0.05
        max_bricks = 6

        n = random.randrange(1, max_bricks+1)  # random from 1 to 6 inclusive
        for i in range(1, n+1):
            state.model_name = 'brick' + str(i)
            state.pose.position.x = pizza_position[0] + random.gauss(mu, sigma)
            state.pose.position.y = pizza_position[1]
            state.pose.position.z = pizza_position[2] + 0.3 + 0.05*i
            publisher_pizza_id[0].publish(state)

        for i in range(n+1, max_bricks+1):
            state.model_name = 'brick' + str(i)
            state.pose.position.x = 0.0
            state.pose.position.y = 0.0
            state.pose.position.z = 0.0 + 0.05*i
            publisher_pizza_id[0].publish(state)
    
    
################################################
################################################  
# Communication  
    
def publish_phi(phi11, phi12, phi21, phi22):
    
#    if phi11 < -cardioid_hard_limit:
#        phi11 = -cardioid_hard_limit
#    if phi11 > cardioid_hard_limit:
#        phi11 = cardioid_hard_limit
#    
#    if phi21 < -cardioid_hard_limit:
#        phi21 = -cardioid_hard_limit
#    if phi21 > cardioid_hard_limit:
#        phi21 = cardioid_hard_limit
    
    out = np.array([phi11, phi12, phi21, phi22])
    
    for i in range(0, len(publisher_ids)):
        pub = publisher_ids[i]		#
        pub.publish(float(out[i]))	#send commands to joint_effort_controller from ROS Control

################################################
################################################      

def callback_JointState(JointState):
    
    
    #get current time
    now = rospy.get_rostime()
    t = 1.0*now.secs + 1.0*now.nsecs / 1000000000
    
    ##################
    #robot sensor feedback
    current_joint_position = np.array(JointState.position) #get current q
    current_joint_velocity = np.array(JointState.velocity) #get current v
    
    q = current_joint_position[0:2]
    v = current_joint_velocity[0:2]
    ##################

    ##################
    #maccepa sensor feedback, maccepa model
    phi11 = current_maccepa_shoulder_position[0] - q[0]
    phi12 = current_maccepa_shoulder_position[1]
    phi21 = current_maccepa_elbow_position[0] - q[1]
    phi22 = current_maccepa_elbow_position[1]

    tau01, k01, T1, T1_long, K1, K1_long = get_linear_model(phi11, phi12)
    tau02, k02, T2, T2_long, K2, K2_long = get_linear_model(phi21, phi22)
    ##################
    
    ##################
    #control input
    q_desired, v_desired, pretension = control_input(t) 
    
    ##################   
    #control error 
    error = q_desired - q
    d_error = v_desired - v

    ################## 
    #forward dynamics, controller
    tau_gravity = gravity_compensation(q[0], q[1])
    
    tau_desired = Kp.dot(error) + Kd.dot(d_error) 
    tau_desired = tau_desired + tau_gravity
    ##################

    delta_phi_cardioid1 = (tau_desired[0] - tau01) / T1[0] 
    delta_phi_cardioid2 = (tau_desired[1] - tau02) / T2[0] 
        
    phi_cardioid1 = q[0] + delta_phi_cardioid1  
    phi_cardioid2 = q[1] + delta_phi_cardioid2  
        
    publish_phi(phi_cardioid1, 
                pretension, 
                phi_cardioid2, 
                pretension)
    
    pizza_teleportation(t)

    
    
    if Verbose:
        print "error: ", error, "d_error: ", d_error
        print "current_maccepa_shoulder_position: ", current_maccepa_shoulder_position
        print "current_maccepa_elbow_position: ", current_maccepa_elbow_position
        print "q: ", q
        print "phi11, phi12: ", phi11, phi12
        print "phi21, phi22: ", phi21, phi22
        print "tau_desired: ", tau_desired 
        print "tau model: ", tau01, tau02        
        print "tau simulation", current_maccepa_shoulder_effort, current_maccepa_elbow_effort
        print "tau simulation2", np.array(JointState.effort)
        print "phi_cardioid1, phi_cardioid2: ", phi_cardioid1, phi_cardioid2
        print "k01, k02:", k01, k02#, ", k_desired: ",  k_desired
        print " "

    if WriteLog:
        f.write("%5.5f " % t)
        f.write("%5.5f " % phi11)
        f.write('\n')
        

def callback_maccepa_shoulder(JointState):	
    global current_maccepa_shoulder_position
    global current_maccepa_shoulder_effort
    
    current_maccepa_shoulder_position = np.array(JointState.position)
    current_maccepa_shoulder_effort[0] = JointState.effort[0]
	
def callback_maccepa_elbow(JointState):
    global current_maccepa_elbow_position
    global current_maccepa_elbow_effort
    
    current_maccepa_elbow_position = np.array(JointState.position)
    current_maccepa_elbow_effort[0] = JointState.effort[0]
    
    
def listener_JointState():
	#create node
    rospy.init_node('custom_controller', anonymous=False)
	
	#create publishers for all joint_effort_controller/command topics, one per joint
    for i in range(0, len(topicname_command)):
	
        publisher_ids[i] = rospy.Publisher(topicname_command[i+1], Float64, queue_size=10)
		
        print "created publisher #", i, ": ", topicname_command[i+1]
	
    
	#subscribe to the joint_states topic, published by ROS Control
    rospy.Subscriber(topicname_joint_states, JointState, callback_JointState, queue_size=1)
    print "created Subscriber: ", topicname_joint_states
	
	
    rospy.Subscriber(topicname_maccepa_shoulder_feedback, JointState, callback_maccepa_shoulder, queue_size=1)
    print "Created Subscriber: ", topicname_maccepa_shoulder_feedback
	
	
    rospy.Subscriber(topicname_maccepa_elbow_feedback, JointState, callback_maccepa_elbow, queue_size=1)	
    print "Created Subscriber: ", topicname_maccepa_elbow_feedback
	
    
    publisher_pizza_id[0] = rospy.Publisher(topicname_set_model_state, ModelState, queue_size=10)
    print "Created pizza publisher: ", topicname_set_model_state
    
    
    #publisher_pizza_id[1] = rospy.ServiceProxy(topicname_SetLinkProperties, SetLinkProperties)
    #print "Created pizza service:", topicname_SetLinkProperties
    #LinkProperties = SetLinkPropertiesRequest() #'Request' is added to get the message type. Horrid.
    #LinkProperties.link_name = 'brick'
    #LinkProperties.mass = 0.5
    #LinkProperties.ixx = 0.01
    #LinkProperties.iyy = 0.01
    #LinkProperties.izz = 0.01
    #
    #publisher_pizza_id[1](LinkProperties)
    
    
    print " "
    print "Something is working!!!!!"
	
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()    
    
################################################
################################################	


if __name__ == '__main__':
    try:
        listener_JointState()
        #main_loop()
        
    except rospy.ROSInterruptException:
        pass










