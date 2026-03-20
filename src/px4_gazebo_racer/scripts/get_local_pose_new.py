import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import sys
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion
import numpy as np
import math
import tf

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])
multi_pose_pub = [None] * vehicle_num
multi_camera_pose_pub = [None] * vehicle_num  # 新增用于发布camera_pose的Publisher
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_camera_pose = [PoseStamped() for i in range(vehicle_num)]  # 新增用于存储camera_pose的变量
quaternion = tf.transformations.quaternion_from_euler(-math.pi / 2, 0, -math.pi / 2)
q = Quaternion([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

def gazebo_model_state_callback(msg):
    for vehicle_id in range(vehicle_num):
        id = msg.name.index(vehicle_type + '_' + str(vehicle_id))
        multi_local_pose[vehicle_id].header.stamp = rospy.Time().now()
        multi_local_pose[vehicle_id].header.frame_id = 'world'
        multi_local_pose[vehicle_id].pose = msg.pose[id]

        # 更新camera_pose
        update_camera_pose(vehicle_id)

def update_camera_pose(vehicle_id):
    multi_camera_pose[vehicle_id].header.frame_id = 'world'
    multi_camera_pose[vehicle_id].pose.position.x = multi_local_pose[vehicle_id].pose.position.x + 0.1
    multi_camera_pose[vehicle_id].pose.position.y = multi_local_pose[vehicle_id].pose.position.y
    multi_camera_pose[vehicle_id].pose.position.z = multi_local_pose[vehicle_id].pose.position.z

    q_ = Quaternion(multi_local_pose[vehicle_id].pose.orientation.w,
                    multi_local_pose[vehicle_id].pose.orientation.x,
                    multi_local_pose[vehicle_id].pose.orientation.y,
                    multi_local_pose[vehicle_id].pose.orientation.z)
    q_ = q_ * q
    multi_camera_pose[vehicle_id].pose.orientation.w = q_[0]
    multi_camera_pose[vehicle_id].pose.orientation.x = q_[1]
    multi_camera_pose[vehicle_id].pose.orientation.y = q_[2]
    multi_camera_pose[vehicle_id].pose.orientation.z = q_[3]

if __name__ == '__main__':
    rospy.init_node(vehicle_type + '_get_pose_groundtruth')
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback, queue_size=1)
    for i in range(vehicle_num):
        multi_pose_pub[i] = rospy.Publisher(vehicle_type + '_' + str(i) + '/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        multi_camera_pose_pub[i] = rospy.Publisher(vehicle_type + '_' + str(i) + '/camera_pose', PoseStamped, queue_size=1)  # 新增Publisher
        print("Get " + vehicle_type + "_" + str(i) + " groundtruth pose and camera pose")
    rate = rospy.Rate(60)  # 修改频率为60Hz以匹配B.py的频率

    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            pose = PoseStamped()
            pose.pose.position.x = multi_local_pose[i].pose.position.x
            pose.pose.position.y = multi_local_pose[i].pose.position.y
            pose.pose.position.z = multi_local_pose[i].pose.position.z

            q_ = Quaternion(multi_local_pose[i].pose.orientation.w, multi_local_pose[i].pose.orientation.x, multi_local_pose[i].pose.orientation.y, multi_local_pose[i].pose.orientation.z)
            pose.pose.orientation.w = q_.w
            pose.pose.orientation.x = q_.x
            pose.pose.orientation.y = q_.y
            pose.pose.orientation.z = q_.z
            pose.header.stamp = rospy.Time().now()
            pose.header.frame_id = 'world'

            multi_pose_pub[i].publish(pose)

            # 发布camera_pose
            multi_camera_pose[i].header.stamp = rospy.Time().now()
            multi_camera_pose_pub[i].publish(multi_camera_pose[i])

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break