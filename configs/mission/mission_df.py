#!/usr/bin/env python3

import mission_execution_control as mxc
import rospy
import time
'''
This is a simple mission, the drone takes off, follows a path and lands
1.startTask. Starts the task and continues.
2.executeTask. Starts the task and waits until it stops.
'''
def mission():
  print("Starting mission...")

  print("Taking off...")
  mxc.executeTask('TAKE_OFF')
  #mxc.startTask('FOLLOW_PATH')

  print("FOLLOW_PATH...")
  mxc.startTask('FOLLOW_PATH')
  mxc.executeTask('SEND_PATH', path = [ [1, -1, 1] , [1, 1, 1] , [-1, 1, 1] , [-1, -1, 1], [0, 0, 1] ], speed = 0.5)
  #mxc.executeTask('SEND_PATH', path = [ [1, -1, 1] , [1, 1, 1] , [-1, 1, 1] , [-1, -1, 1], [0, 0, 1] ], speed = 0.5)

  print("LAND...")
  mxc.executeTask('LAND', speed = 0.3)

  print('Finish mission...')
