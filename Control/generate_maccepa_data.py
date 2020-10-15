import numpy as np
from math import pi, sin, cos, atan, sqrt

import pickle

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import os

################################
### MACCEPA PARAMETERS

min_deflection_angle = -pi/4
max_deflection_angle = pi/4

min_motor2_angle = 0
max_motor2_angle = 15*pi

R_pretension = 0.0045
C = 0.042
B = 0.022
R = 0.0145
k = [4800, 1200]

################################
### MACCEPA MODEL

def calculate_torque(deflection_angle, motor2_angle, index):
	abs_deflection_angle = abs(deflection_angle)
	sign_deflection_angle = np.sign(deflection_angle)
		
	#/* look for formulas in paper */
	pretension = motor2_angle * R_pretension

	I = 1.0 * C * cos(abs_deflection_angle) - B
	H = 1.0 * C * sin(abs_deflection_angle) - R
	E = 1.0 * sqrt(I*I - R*R + H*H)
	theta = 1.0 * atan(E/R)
	gamma = 1.0 * atan(H/I)
	lmbda = 1.0 * pi/2 - theta + gamma
	kappa = 1.0 * pi/2 - theta + gamma - abs_deflection_angle
	force = 1.0 * k[index] * (B - C + pretension + E + R*lmbda) * sign_deflection_angle

	force_arm = abs(C * sin(kappa))
		
	torque = force * force_arm
	
	return torque

################################
### MESH PARAMETERS

Count = 1000

step_deflection_angle = (max_deflection_angle - min_deflection_angle) * (1.0 / Count)
step_motor2_angle     = (max_motor2_angle     - min_motor2_angle)     * (1.0 / Count)

delta_q = 0.01

################################
### Generate data

for motor_index in range(0,2):
	Result = {'torque':             np.zeros((Count, Count)),
			  'stiffness':          np.zeros((Count, Count)),
			  'deflection_angle':   np.zeros((Count, Count)),
			  'motor2_angle':       np.zeros((Count, Count))}

	for i in range(0, Count):
		for j in range(0, Count):
			deflection_angle = min_deflection_angle + step_deflection_angle * i
			motor2_angle     = min_motor2_angle     + step_motor2_angle     * j

			torque = calculate_torque(deflection_angle, motor2_angle, motor_index)

			dtorque = calculate_torque((deflection_angle - delta_q), motor2_angle, motor_index) - torque
			stiffness = -dtorque / delta_q

			Result['torque'][i, j] = torque
			Result['stiffness'][i, j] = stiffness

			Result['deflection_angle'][i, j] = deflection_angle
			Result['motor2_angle'][i, j] = motor2_angle

			#print " torque: ", torque, " kappa: ", kappa, " lmbda: ", lmbda, " gamma: ", gamma, " theta: ", theta
			#print " torque: ", torque, " E: ", E, " H: ", H, " I: ", I, " pretension: ", pretension
			#print " torque: ", torque, " deflection_angle: ", deflection_angle, " motor2_angle: ", motor2_angle

	Result['min_deflection_angle'] = min_deflection_angle
	Result['max_deflection_angle'] = max_deflection_angle
	Result['min_motor2_angle'] = min_motor2_angle
	Result['max_motor2_angle'] = max_motor2_angle
	Result['Count'] = Count
	Result['step_deflection_angle'] = step_deflection_angle
	Result['step_motor2_angle']     = step_motor2_angle

	dir = os.path.dirname(__file__)
	filepath = os.path.join(dir, 'MACCEPA_res' + str(motor_index))

	with open(filepath, 'wb') as fp:
		pickle.dump(Result, fp)

	print "Result_torque: ", Result['torque']
	print "Result_stiffness: ", Result['stiffness']
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	ax.plot_surface(Result['deflection_angle'], Result['motor2_angle'], Result['torque'])
	#plt.show()


	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	ax.plot_surface(Result['deflection_angle'], Result['motor2_angle'], Result['stiffness'])
	plt.show()

