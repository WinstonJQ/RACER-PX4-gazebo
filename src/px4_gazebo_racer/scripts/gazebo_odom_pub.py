#!/usr/bin/env python3
"""
Publish nav_msgs/Odometry from Gazebo ground truth for each drone.
Bypasses MAVROS local_position/odom to avoid SITL high-load dropouts.

Topics published: /iris_X/ground_truth/odom  (nav_msgs/Odometry, 60 Hz)
"""

import sys
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])

odom_pubs = [None] * vehicle_num
odom_msgs = [Odometry() for _ in range(vehicle_num)]
received = [False] * vehicle_num


def model_states_cb(msg):
    for i in range(vehicle_num):
        name = vehicle_type + '_' + str(i)
        try:
            idx = msg.name.index(name)
        except ValueError:
            continue

        o = odom_msgs[i]
        o.header.frame_id = 'world'
        o.child_frame_id = name + '/base_link'

        o.pose.pose = msg.pose[idx]
        # Gazebo twist is in world frame
        o.twist.twist = msg.twist[idx]

        received[i] = True


if __name__ == '__main__':
    rospy.init_node('gazebo_odom_pub')
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb, queue_size=1)

    for i in range(vehicle_num):
        topic = '/' + vehicle_type + '_' + str(i) + '/ground_truth/odom'
        odom_pubs[i] = rospy.Publisher(topic, Odometry, queue_size=1)
        rospy.loginfo('Publishing ground truth odom on %s', topic)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for i in range(vehicle_num):
            if received[i]:
                odom_msgs[i].header.stamp = now
                odom_pubs[i].publish(odom_msgs[i])
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
