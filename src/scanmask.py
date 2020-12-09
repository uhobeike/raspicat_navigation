#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8

flag  = 0  

pub = rospy.Publisher('scan2', LaserScan)
    
def callback(data):
    global flag
    if (flag == 1):
        print("will mask")
        print(flag)
        l = list(data.ranges)
        mask = [60]*362
        l[273:635] = mask
        data.ranges = tuple(l)
    pub.publish(data)        

def flag_cb(data):
    global flag
    print("flagreceived")
    print(data)
    flag = data.data 
    print(flag)

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Subscriber("slope_flag", Int8, flag_cb)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()

