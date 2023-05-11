#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Float64

current_pose = PoseWithCovarianceStamped()
# goal_xy = []
goal_xx = []
goal_yy = []
current_x = 0.0
current_y= 0.0
current_z = 0.0

class GotoPoint():
    def __init__(self):
        rospy.init_node('map_navigation_kim', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.callback)
        # self.num_list = rospy.Subscriber('my_topic', Float64MultiArray, self.callback1)
        self.coord_1_x_sub = rospy.Subscriber('coord_1_x', Float64MultiArray, self.callback_x)
        self.coord_1_y_sub = rospy.Subscriber('coord_1_y', Float64MultiArray, self.callback_y)

    def run(self):
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)

        if(len(goal_xx) != 0 and len(goal_yy) != 0 and current_x != 0.0):
            index = 0
            while not rospy.is_shutdown():
                print("index number : ", index)
                               
                goal_x = goal_xx[index]
                goal_y = goal_yy[index]
                goal_x1 = goal_xx[index+1]
                goal_y1 = goal_yy[index+1]
                goal_x2 = goal_xx[index+2]
                goal_y2 = goal_yy[index+2]
                goal_x3 = goal_xx[index+3]
                goal_y3 = goal_yy[index+3]
                target_X = goal_xx[index+10]
                target_Y = goal_yy[index+10]
                # print("target_X, target_Y : ", target_X, " ,", target_Y)


                distance = sqrt(pow(target_X - current_x, 2) + pow(target_Y - current_y, 2))
                goal_z = atan2(target_Y - current_y, target_X - current_x)

                if(goal_z < 0):
                    goal_z = goal_z + 2.0*pi
                
                angle_difference = 0.0
                if(goal_z > current_z):
                    if(goal_z > current_z + pi):
                        angle_difference = 2*pi - goal_z + current_z
                    else:
                        angle_difference = goal_z - current_z
                else:
                    if (goal_z+pi < current_z):
                        angle_difference = 2*pi - current_z + goal_z
                    else :
                        angle_difference = current_z - goal_z
                
                if ((angle_difference > 0.4*pi) and distance > 0.15):
                    self.cmd_vel.publish(Twist())
                    
                    while ((angle_difference > 0.05*pi)and(not rospy.is_shutdown())):
                        
                        print("angle_difference : ", angle_difference)

                        goal_z = atan2(target_Y - current_y, target_X - current_x)
                        
                        if(goal_z < 0):
                            goal_z = goal_z + 2.0*pi

                        max_z = 0.2

                        if(goal_z > current_z):
                            if(goal_z > current_z + pi):
                                move_cmd.linear.x = 0.00
                                move_cmd.angular.z = -max((2*pi - goal_z + current_z)/3.0,max_z)
                                # move_cmd.angular.z = -max_z
                            else:
                                move_cmd.linear.x = 0.00
                                move_cmd.angular.z = max((goal_z - current_z)/3.0,max_z)
                                # move_cmd.angular.z = max_z
                        else:
                            if (goal_z+pi < current_z):
                                move_cmd.linear.x = 0.00
                                move_cmd.angular.z = max((2*pi-current_z+goal_z)/3.0,max_z)
                                # move_cmd.angular.z = max_z
                            else :
                                move_cmd.linear.x = 0.00
                                move_cmd.angular.z = -max((current_z-goal_z)/3.0,max_z)
                                # move_cmd.angular.z = -max_z

                        if(goal_z > current_z):
                            if(goal_z > current_z + pi):
                                angle_difference = 2*pi - goal_z + current_z
                            else:
                                angle_difference = goal_z - current_z
                        else:
                            if (goal_z+pi < current_z):
                                angle_difference = 2*pi - current_z + goal_z
                            else :
                                angle_difference = current_z - goal_z

                        self.cmd_vel.publish(move_cmd)


                goal_z = atan2(target_Y - current_y, target_X - current_x)  
                target_Z = atan2(target_Y-goal_y,target_X-goal_x)

                if(target_Z < 0):
                    target_Z = target_Z + 2.0*pi
                if(goal_z < 0):
                    goal_z = goal_z + 2.0*pi

                weight1 = 1.0
                weight2 = 5.0

                rotation_z = 0
                if(goal_z > current_z):
                    if(goal_z > current_z + pi):
                        rotation_z = weight1*(-(2*pi - goal_z + current_z))
                        # print("right11 : ", -(2 - goal_z + rotation))
                    else:
                        rotation_z = weight1*(goal_z - current_z)
                        # print("left11 : ", (goal_z - rotation))
                else:
                    if (goal_z+pi < current_z):
                        rotation_z = weight1*(2*pi-current_z+goal_z)
                        # print("left22 : ", (2-rotation+goal_z))
                    else :
                        rotation_z = weight1*(-(current_z-goal_z))
                        # print("right22 : ", -(rotation-goal_z))
                
                if(goal_x2 == goal_x1):
                    d = abs(goal_y1 - current_y)
                else:
                    a = -(goal_y2-goal_y1)/(goal_x2-goal_x1)
                    b = 1
                    c = (goal_y2-goal_y1)/(goal_x2-goal_x1)*goal_x1-goal_y1
                    d = abs((a*current_x + b*current_y + c)/sqrt(a*a+b*b))

                print("distance error : ", d)

                if(goal_z > target_Z):
                    if(goal_z > target_Z + pi):
                        move_cmd.angular.z = rotation_z - weight2*d
                        # print("right11 : ", weight2*(-(2 - goal_z + target_Z)))
                    else:
                        move_cmd.angular.z = rotation_z + weight2*d
                        # print("left11 : ", weight2*((goal_z - target_Z)))
                else:
                    if (goal_z+1 < target_Z):
                        move_cmd.angular.z = rotation_z + weight2*d
                        # print("left22 : ", weight2*((2-target_Z+goal_z)))
                    else :
                        move_cmd.angular.z = rotation_z + -weight2*d
                        # print("right22 : ", weight2*(-(target_Z-goal_z)))
                

                distance = sqrt(pow((target_X - current_x), 2) + pow((target_Y - current_y), 2))
                
                distance0 = sqrt(pow((goal_x - current_x), 2) + pow((goal_y - current_y), 2))
                distance1 = sqrt(pow((goal_x1 - current_x), 2) + pow((goal_y1 - current_y), 2))
                distance2 = sqrt(pow((goal_x2 - current_x), 2) + pow((goal_y2 - current_y), 2))
                distance3 = sqrt(pow((goal_x3 - current_x), 2) + pow((goal_y3 - current_y), 2))

                index_reset = min(distance0,distance1,distance2,distance3)

                if(index_reset == distance1):
                    index = index+1
                elif(index_reset == distance2):
                    index = index+2
                elif(index_reset == distance3):
                    index = index+3

                if(distance > 0.8):
                    print("miss")

                move_cmd.linear.x = min(distance/2, 0.15)
                # print("distance : ", distance)
                self.cmd_vel.publish(move_cmd)
                r.sleep()


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def callback(self, msg):
        global current_pose
        global current_x
        global current_y
        global rotation
        global current_z
        # print("callback on")
        if(msg != current_pose):
            current_pose = msg
            current_x = current_pose.pose.pose.position.x
            current_y = current_pose.pose.pose.position.y
            rotation = current_pose.pose.pose.orientation
            current_z = self.quaternion_to_euler_angle(rotation)
            
        
    def callback_x(self, msg):
        global goal_xx
        goal_xx = msg.data 
        # print("goal_x : " , goal_x)
        
    def callback_y(self, msg):
        global goal_yy
        goal_yy = msg.data
        # print("goal_y : ", goal_y)
          
    def quaternion_to_euler_angle(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        ysqr = y * y
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = atan2(t3, t4)

        if(Z < 0):
            Z = Z + 2*pi
        
        return Z
    

if __name__ == '__main__':
    try:
        gp=GotoPoint()

        while not rospy.is_shutdown():
            print("waiting for coord_xy")
            gp.run()
            # rospy.spin()

    except:
        rospy.loginfo("shutdown program.")