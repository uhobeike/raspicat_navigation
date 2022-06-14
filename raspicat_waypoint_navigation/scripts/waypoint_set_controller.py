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
Waypoint Set controller Menu:
---------------------------
Keyboard Layout: 
12345678      
    s  f    j  

Main Functions:
s  : Run waypoint_rviz_set Node 
j  : Remove waypoint 
f  : Finish

Add functionality to waypoint:
1  : Goal Function
2  : Stop Function
3  : Variable Waypoint Radius Function
4  : Waiting line Function
5  : Attention Speak Function
6  : Variable Speed Function
7  : Step Function
8  : Slop Function
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
  nd=nodelist[0].decode("utf-8")
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
        if not key == '':
          print(msg)
        if key == 's':
          global proc
          find_node('waypoint_rviz_set')
          if(flag == 0):
            args = sys.argv
            proc = Popen(["rosrun", "raspicat_navigation", "waypoint_rviz_set", args[1]])
            rospy.loginfo("waypoint_set node RUN")
          elif(flag == 1):
            rospy.loginfo("already waypoint_rviz_set node RUN\n")
            
        elif key == 'j':
          str = "remove"
          pub_way.publish(str)
          rospy.loginfo("way_point_remove...")

        elif key == 'f':
          str = "finish"
          pub_way.publish(str)
          #kill_node('waypoint_rviz')
          rospy.loginfo("kill_node waypoint_rviz")
          rospy.loginfo("Shutdown now ('o')/ bye bye~~~")
          sys.exit(0)

        elif key == '1':
          str = "goal"
          pub_way.publish(str)
          rospy.loginfo("Goal Function Set!!")
        
        elif key == '2':
          str = "stop"
          pub_way.publish(str)
          rospy.loginfo("Stop Function Set!!")
        
        elif key == '3':
          str = "variable_waypoint_radius"
          pub_way.publish(str)
          rospy.loginfo("Variable Waypoint Radius Function Set!!")

        elif key == '4':
          str = "waiting_line"
          pub_way.publish(str)
          rospy.loginfo("Waiting line Function Set!!")

        elif key == '5':
          str = "attention_speak"
          pub_way.publish(str)
          rospy.loginfo("Attention Speak Function Set!!")

        elif key == '6':
          str = "variable_speed"
          pub_way.publish(str)
          rospy.loginfo("Variable Speed Function Set!!")

        elif key == '7':
          str = "step"
          pub_way.publish(str)
          rospy.loginfo("Step Function Set!!")

        elif key == '8':
          str = "slop"
          pub_way.publish(str)
          rospy.loginfo("Slop Function Set!!")


        else:
            if (key == '\x03'):
                break

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
