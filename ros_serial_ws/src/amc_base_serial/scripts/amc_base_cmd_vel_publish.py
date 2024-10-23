#!/usr/bin/env python
#coding=utf-8

import rospy

from geometry_msgs.msg import Twist
import tty, termios, sys,select

if __name__ == "__main__":
    rospy.init_node("amc_cmd_vel_publisher")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        walk_vel_ = 0.5
        yaw_rate_ = 0
        
        max_tv = walk_vel_
        max_rv = yaw_rate_
        
        property_list = termios.tcgetattr(sys.stdin)
        property_list[3] = property_list[3] & ~termios.ICANON & ~termios.ECHO
        tty.setraw(sys.stdin.fileno())
        can_read, _, _ = select.select([sys.stdin], [], [], 0.1)
        if can_read:
            ch = sys.stdin.read(1)
        else:
            ch = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, property_list)
        
        if ch == 'w':
            max_tv = walk_vel_
            max_rv = 0
        elif ch == 's':
            max_tv = - walk_vel_
            max_rv = 0
        elif ch == 'a':
            max_tv = walk_vel_ 
            max_rv = yaw_rate_
            
        elif ch == 'd':
            max_tv = walk_vel_ 
            max_rv = -yaw_rate_
        elif ch == 'x':
            max_tv = 0
            max_rv = -yaw_rate_
        elif ch == 'z':
            max_tv = 0
            max_rv = yaw_rate_
        elif ch == 'q':
            exit()
        else:
            max_tv = 0
            max_rv = 0

        #发送消息
        cmd = Twist()
        cmd.linear.x = float(max_tv)
        cmd.angular.z = float(max_rv)
        pub.publish(cmd)
        rospy.loginfo("Publishing : 线速度:%f   角速度: %f" % (cmd.linear.x,cmd.angular.z))
        rate.sleep()
