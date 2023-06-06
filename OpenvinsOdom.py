# !/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 23-6-6 下午19:4
# @Author  : 刘丹迪
# @File    : OpenvinsOdom.py

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def thread_job():
    rospy.spin()
    
class Controller:
    
    def __init__(self):
        self.pos = PositionTarget()
        self.pos.type_mask = int('101111111000', 2)
        self.pos.coordinate_frame= 1
        
        rospy.Subscriber("/ov_msckf/poseimu", PoseWithCovarianceStamped,self.openvins_Cb)
        self.pos_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size = 1)# topic 
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()
    
        
    def openvins_Cb(self,msg):  
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler_q = quaternion2euler(quaternion)
        self.pos.header.stamp = rospy.Time.now()
        self.pos.position.x =  msg.pose.pose.position.x
        self.pos.position.y =  msg.pose.pose.position.y
        self.pos.position.z =  msg.pose.pose.position.z
        self.pos.yaw = euler_q[2]
        
# 
def main():
    print("start!")
    rospy.init_node('openvins_odom_node', anonymous=True)

    cnt = Controller()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        cnt.pos_pub.publish(cnt.pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
