import roslib
import rospy
import select
import mtdevice
import math
import pdb

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from xsens_msgs.msg import sensorSample, baroSample, gnssSample
from xsens_msgs.msg import positionEstimate, velocityEstimate,orientationEstimate

import numpy

def callback_Euler(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)

def imu_parser():
    
    rospy.init_node('imu_parser',anonymous=True)

    rospy.Subscrier("imu/filter/orientation",Imu,callback_Euler)

    rospy.spin()

if __name__ == '__main__':
    imu_parser()
