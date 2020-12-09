#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int8

num_of_slope = 1
slope_areas = [[ 60, 70 , -15, 3]] # square 
"""
 [0, 3]---------[1, 3]      [x, y]
  |               |         [0] < [1]
  |               |         [2] < [3]
 [0, 2]---------[1, 2]


"""

pub = rospy.Publisher('slope_flag', Int8)
    
def callback(data):
    for i in range(num_of_slope):
        if( slope_areas[i][0] < data.pose.pose.position.x and \
            slope_areas[i][1] > data.pose.pose.position.x and \
            slope_areas[i][2] < data.pose.pose.position.y and \
            slope_areas[i][3] > data.pose.pose.position.y ):
            print("true")
            flag = 1
            break
        else:
            flag = 0
            #print(data.pose.pose.position.x)
            #print(slope_areas[i][0])
            #print(slope_areas[i][1])
            print("false")        
    print(flag)
    pub.publish(flag)        

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()

