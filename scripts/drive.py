#!/usr/bin/env python

import setup_path 
import airsim
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------

Moving around:
w:forward (+x)
s:backwards (-x)
d:right (+y)
a:left(-y)
r : up (+z)
f : down (-z)

anything else : stop

q/e : increase/decrease max speeds by 1 m/s

CTRL-C to quit
"""

moveBindings = {
	'w':(1,0,0),
       	's':(-1,0,0),
	'd':(0,1,0),
	'a':(0,-1,0),
	'r':(0,0,1),
	'f':(0,0,-1),
    }

speedBindings={
        'q':(1,0,0),
        'e':(-1,0,0),
        
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
   
    return key
x= 0
y=0
z=0
speed=5
while True:
    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = x + moveBindings[key][0]
                y = y + moveBindings[key][1]
                z = z+ moveBindings[key][2]
		client.moveToPositionAsync(x, y, z, speed).join()
		client.hoverAsync().join()
            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
		client.moveToPositionAsync(x, y, z, speed).join()
		client.hoverAsync().join()
            else :
		client.moveToPositionAsync(x, y, z, speed).join()
		client.hoverAsync().join()
		break

    except Exception as e:
        print(e)

    finally:

	client.moveToPositionAsync(x, y, z, speed).join()
	client.hoverAsync().join()
       
