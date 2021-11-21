#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def soundCallback(data):
  rospy.loginfo("Waypoint Passed, playing sound")
  soundhandle = SoundClient()
  rospy.sleep(1)

  num = 3
  volume = 1

  soundhandle.play(num, volume)

  rospy.sleep(1)

def listener():
  rospy.init_node('waypoint_play_sound', anonymous=True)

  rospy.Subscriber("waypoint_passed", Bool, soundCallback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
  print("Initialized Wayting for waypoint passed msg")
  listener()