#!/usr/bin/env python
# -*- coding: utf-8 -*-
    
import rospy
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
How to
---------------------------
Moving around:
        
        s         f  g  j  k 
        

j/k : waypoint add/waypoint add remove
g   : goal set
f   : finish and file write waypoint
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('waypoint_set_controller_for_rviz')
    pub_way = rospy.Publisher('waypoint_set_flag', String, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    pub_flag = 1
    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'f' :
                 str = "finish and file_write_waypoint"
                 pub_way.publish(str)
                 rospy.loginfo("finish and file write waypoint \(^_^)/")
            elif key == 'g' :
                 str = "goal_set"
                 pub_way.publish(str)
                 rospy.loginfo("goal_set..............")
            elif key == 'j' :
                 str = "way_point_add"
                 pub_way.publish(str)
                 rospy.loginfo("way_point_add.........")
            elif key == 'k' :
                 str = "way_point_remove"
                 pub_way.publish(str)
                 rospy.loginfo("way_point_remove.........")
            elif key == 'q' :
                 pub_flag = 0
            elif key == ' ' or key == 's' :
                
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

    except:
        print(e)

    finally:
      rospy.loginfo("see you!")

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
