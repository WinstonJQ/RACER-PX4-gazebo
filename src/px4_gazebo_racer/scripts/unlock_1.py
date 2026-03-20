#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardController:
    def __init__(self):
        self.current_state = State()
        self.namespace = "iris_1"
        
        # 初始化ROS节点
        rospy.init_node('offboard_control_node_1', anonymous=True)
        
        # 带命名空间的订阅者和服务客户端
        state_sub = rospy.Subscriber(
            f'/{self.namespace}/mavros/state', 
            State, 
            self.state_callback
        )
        self.set_mode_client = rospy.ServiceProxy(
            f'/{self.namespace}/mavros/set_mode', 
            SetMode
        )
        self.arming_client = rospy.ServiceProxy(
            f'/{self.namespace}/mavros/cmd/arming', 
            CommandBool
        )
        self.pose_pub = rospy.Publisher(
            f'/{self.namespace}/mavros/setpoint_position/local', 
            PoseStamped, 
            queue_size=10
        )

    def state_callback(self, data):
        self.current_state = data

    def run(self):
        rate = rospy.Rate(20)  # PX4建议最小2Hz
        
        # 等待飞控连接
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        
        # 创建初始设定点（保持当前高度）
        pose = PoseStamped()
        pose.pose.position.z = 0  # 保持当前高度
        
        # 预发送设定点
        for _ in range(100):
            if rospy.is_shutdown():
                return
            self.pose_pub.publish(pose)
            rate.sleep()
        
        # 尝试切换模式
        last_request = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "OFFBOARD" and (now - last_request > 2.0):
                if self.set_mode_client(0, "OFFBOARD").mode_sent:
                    rospy.loginfo("Offboard模式已激活")
                last_request = now
            else:
                if not self.current_state.armed and (now - last_request > 2.0):
                    if self.arming_client(True).success:
                        rospy.loginfo("无人机已解锁")
            
            # 持续发布设定点保持模式
            self.pose_pub.publish(pose)
            rate.sleep()

if __name__ == "__main__":
    try:
        controller = OffboardController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

