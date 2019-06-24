#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import JointState

import numpy as np
import sys

angle1 = 0.0
angle2 = 0.0

def callback_joint_states(data):
  angle1 = data.position[0]
  angle2 = data.position[1]

  sys.stdout.write("\r1st joint %7.2f |2nd joint %7.2f" % (angle1,angle2))
  sys.stdout.flush()

def main():
  rospy.init_node("joint_states_subscriber", anonymous=False)

  rospy.Subscriber("IB_robot/joint_states",JointState,callback_joint_states)
  rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
