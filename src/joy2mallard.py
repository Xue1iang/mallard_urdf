#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState


# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical
# axis 0 aka left stick horizonal

def callback(data):
 global pub
 cmd_vel_y = float(data.axes[0])
 cmd_vel_x = float(data.axes[1])
 new_msg = JointState()
 new_msg.header.stamp = rospy.Time.now()
 new_msg.name = ['x_thr_left', 'x_thr_right', 'y_thr_left', 'y_thr_right']
 new_msg.effort = [cmd_vel_x, cmd_vel_x, cmd_vel_y, cmd_vel_y]
 print new_msg
 pub.publish(new_msg)

def start():
 rospy.init_node('Joy2thr')
 global pub
 pub = rospy.Publisher("/mallard/thruster_command", JointState, queue_size=10)
 rospy.Subscriber("joy", Joy, callback)

if __name__ == '__main__':
 start()
 rospy.spin()
