#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import json
import time
# from constants import *

def velocity_publisher():
    # file_name = 'action_velocity.json'
    # with open(file_name) as json_file:
    #   params = json.load(json_file)
    LIN = 0
    ANG = 1
    params = {"velocity": [[0.017278759594743863, 0.21598449493429828], [0.034557519189487726, 0.0], [0.043196898986859654, -0.10799224746714917], [0.05183627878423159, 0.0], [0.05183627878423159, 0.0], [0.05183627878423159, 0.0], [0.034557519189487726, 0.0], [0.05183627878423159, 0.0], [0.043196898986859654, 0.10799224746714917], [0.043196898986859654, -0.10799224746714917], [0.034557519189487726, 0.0], [0.05183627878423159, 0.0], [0.05183627878423159, 0.0], [0.017278759594743863, -0.21598449493429828], [0.043196898986859654, 0.10799224746714917], [0.034557519189487726, 0.0], [0.05183627878423159, 0.0], [0.043196898986859654, -0.10799224746714917], [0.043196898986859654, -0.10799224746714917], [0.017278759594743863, 0.21598449493429828], [0.017278759594743863, -0.21598449493429828], [0.034557519189487726, 0.0], [0.025918139392115794, 0.3239767424014474], [0.025918139392115794, -0.3239767424014474], [0.017278759594743863, -0.21598449493429828], [0.017278759594743863, 0.21598449493429828], [0.05183627878423159, 0.0], [0.034557519189487726, 0.0], [0.017278759594743863, -0.21598449493429828], [0.043196898986859654, 0.10799224746714917], [0.034557519189487726, 0.0], [0.043196898986859654, -0.10799224746714917], [0.043196898986859654, 0.10799224746714917], [0.043196898986859654, -0.10799224746714917], [0.043196898986859654, 0.10799224746714917], [0.05183627878423159, 0.0], [0.05183627878423159, 0.0], [0.05183627878423159, 0.0], [0.05183627878423159, 0.0]], "delta_time": 3}




    action_velocity_list = params['velocity']
    delta_time = params['delta_time']
    print('dT: '+str(delta_time))

    #Initialize publisher node and publish message object
    rospy.init_node('vel_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    vel_msg = Twist()

    #i = 0

    while not rospy.is_shutdown():
        i = 0
        start_time = time.clock()
        print('Steps taken :'+str(len(action_velocity_list)))
        print('Simulating motion ..')
        while i < len(action_velocity_list):
            # current_time = time.clock()
            # elapsed_time = current_time - start_time
            # if elapsed_time >= delta_time:
                #start_time = time.clock()
            # if i > len(action_velocity_list):
            #   break

            linear_vel = action_velocity_list[i][LIN]
            angular_vel = action_velocity_list[i][ANG]
            #print('Publishing : Linear Velocity:= '+str(linear_vel)+' Angular Velocity:= '+str(angular_vel))
            vel_msg.linear.x = linear_vel
            vel_msg.angular.z = angular_vel
            print('Publishing : Linear Velocity:= '+str(vel_msg.linear.x)+' Angular Velocity:= '+str(vel_msg.angular.z))
            pub.publish(vel_msg)
            rospy.sleep(delta_time) #Sleeps for T = delta_time. After publishing
            i += 1
        print('Stopping..')
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        pub.publish(vel_msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass

