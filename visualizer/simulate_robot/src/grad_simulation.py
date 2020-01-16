#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import sys

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Imu, JointState
from xsens_msgs.msg import orientationEstimate, velocityEstimate

q = np.array([0.,0.])
qdot = np.array([0.,0.])

#enviroment parameters
g = 9.81

#robot parameters
I1 = 0.0058
I2 = 0.070
d1 = 0.232
d2 = 0.121
l1 = 0.323
m1 = 0.710
m2 = 0.351

#control parameters
kd = 200
kp = 2000
alpha = np.pi/6

def callback_joint_states(data):
  global q, qdot

  q = data.position
  qdot = data.velocity

  sys.stdout.write("\rq1 %7.2f rad|q2 %7.2f rad|q1dot %7.2f rad/s|q2dot %7.2f rad/s" % (q[0],q[1],qdot[0],qdot[1]))
  sys.stdout.flush()

def calculate_torque(angle, a_vel):

  q2d = alpha * np.arctan(angle[1])

  t2 = d2**2
  t3 = m2*t2
  t5 = np.cos(angle[1])
  t9 = d2*l1*m2*t5
  t4 = I2+t3+t9
  t6 = np.cos(angle[0])
  t7 = angle[0]+angle[1]
  t8 = np.cos(t7)
  t10 = d1**2
  t11 = m1*t10
  t12 = l1**2
  t13 = m2*t12
  t14 = d2*l1*m2*t5*2.0
  t15 = I1+I2+t3+t11+t13+t14
  t16 = 1.0/t15
  t17 = np.sin(angle[1])
  Tc = -(kd*a_vel[1]+kp*(angle[1]-q2d))*(I2+t3-t4**2*t16)-g*t4*t16*(d1*m1*t6+d2*m2*t8+l1*m2*t6)+d2*g*m2*t8+d2*l1*m2*t17*a_vel[0]**2+d2*l1*m2*t4*t16*t17*a_vel[1]*(a_vel[0]*2.0+a_vel[1])
  
  return Tc

def main():
  rospy.init_node('graduation_simulator_signal', anonymous=True)

  pub = rospy.Publisher('IB_robot/joint2_controller/command', Float64, queue_size=0)
  rate = rospy.Rate(150)

  while not rospy.is_shutdown():
    rospy.Subscriber("IB_robot/joint_states", JointState, callback_joint_states)

    pub.publish(calculate_torque(q,qdot))

    rate.sleep()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException: pass