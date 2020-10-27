#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from subprocess import * 
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
How to
---------------------------
Moving around:
      
    s  f  g  j  
      c
s   : waypoint_rviz node RUN
j   : waypoint add remove
g   : goal set
c   : corner set
f   : finish and file write waypoint and kill_node
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

def kill_node(nodename): 
  p=Popen(['rosnode','list'],stdout=PIPE) 
  p.wait() 
  nodelist=p.communicate() 
  nd=nodelist[0] 
  nd=nd.split("\n") 
  for i in range(len(nd)): 
    tmp=nd[i] 
    ind=tmp.find(nodename) 
    if ind == 1: 
      call(['rosnode','kill',nd[i]]) 
      break 

def find_node(nodename): 
  global flag
  flag = 0
  p=Popen(['rosnode','list'],stdout=PIPE) 
  p.wait() 
  nodelist=p.communicate() 
  nd=nodelist[0] 
  nd=nd.split("\n") 
  for i in range(len(nd)): 
    tmp=nd[i] 
    ind=tmp.find(nodename) 
    if ind == 1: 
      flag = 1
      break 

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('waypoint_command_send_node')
    pub_way = rospy.Publisher('waypoint_control', String, queue_size=1)

    print(msg)
    while(1):
        key = getKey()

        if key == 's' :
          global proc
          find_node('waypoint_rviz')
          if(flag == 0):
            proc = Popen(["rosrun", "raspicat_navigation", "waypoint_rviz"])
            rospy.loginfo("waypoint_rviz_set node RUN")
          elif(flag == 1):
            rospy.loginfo("already waypoint_rviz_set node RUN\n")
            
        elif key == 'j' :
          str = "remove"
          pub_way.publish(str)
          rospy.loginfo("way_point_remove...")

        elif key == 'g' :
          str = "goal"
          pub_way.publish(str)
          rospy.loginfo("goal_set......")

        elif key == 'c' :
          str = "corner"
          pub_way.publish(str)
          rospy.loginfo("corner_set...........")


        elif key == 'f' :
          str = "finish"
          pub_way.publish(str)
          #kill_node('waypoint_rviz')
          rospy.loginfo("kill_node waypoint_rviz")
          rospy.loginfo("Shutdown now ('o')/ bye bye~~~")
          sys.exit(0)

        else:
            if (key == '\x03'):
                break

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
