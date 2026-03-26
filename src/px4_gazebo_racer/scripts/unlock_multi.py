#!/usr/bin/env python
"""
多机手动解锁和起飞脚本
功能：逐架确认解锁和起飞 iris_0~3，所有无人机就绪后提示触发探索
"""

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from quadrotor_msgs.msg import TakeoffLand


class DroneUnlocker:
    def __init__(self, namespace):
        self.namespace = namespace
        self.current_state = State()

        rospy.Subscriber(f'/{namespace}/mavros/state', State, self._state_cb)
        self.arming_client = rospy.ServiceProxy(f'/{namespace}/mavros/cmd/arming', CommandBool)
        self.takeoff_pub = rospy.Publisher(f'/{namespace}/takeoff_land', TakeoffLand, queue_size=1)

    def _state_cb(self, msg):
        self.current_state = msg

    def wait_connected(self):
        rospy.loginfo(f"[{self.namespace}] 等待飞控连接...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo(f"[{self.namespace}] 飞控已连接")

    def arm(self):
        try:
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo(f"[{self.namespace}] 解锁成功")
                return True
            else:
                rospy.logwarn(f"[{self.namespace}] 解锁被拒绝")
                return False
        except Exception as e:
            rospy.logerr(f"[{self.namespace}] 解锁服务调用失败: {e}")
            return False

    def send_takeoff(self):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self.takeoff_pub.publish(msg)
        rospy.loginfo(f"[{self.namespace}] 起飞命令已发送")

    def wait_offboard(self, timeout=30):
        rospy.loginfo(f"[{self.namespace}] 等待进入 OFFBOARD 模式...")
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn(f"[{self.namespace}] 等待 OFFBOARD 超时")
                return False
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                rospy.loginfo(f"[{self.namespace}] 已进入 OFFBOARD 悬停模式")
                return True
            rate.sleep()
        return False


def ask(prompt):
    return input(prompt + " (y/n): ").strip().lower() == "y"


def main():
    rospy.init_node('unlock_multi_node', anonymous=True)

    namespaces = ["iris_0", "iris_1", "iris_2", "iris_3"]
    drones = [DroneUnlocker(ns) for ns in namespaces]

    # 等待所有飞控连接
    for d in drones:
        d.wait_connected()

    # 逐架解锁和起飞
    for d in drones:
        ns = d.namespace

        if not ask(f"\n是否解锁 {ns}？"):
            rospy.loginfo(f"[{ns}] 跳过")
            continue

        if not d.arm():
            rospy.logwarn(f"[{ns}] 解锁失败，跳过起飞")
            continue

        rospy.sleep(1.0)

        if not ask(f"是否起飞 {ns}？"):
            rospy.loginfo(f"[{ns}] 跳过起飞")
            continue

        d.send_takeoff()

        if not d.wait_offboard():
            rospy.logwarn(f"[{ns}] 未能确认进入 OFFBOARD，请检查 px4ctrl 状态")

    rospy.loginfo("")
    rospy.loginfo("=" * 60)
    rospy.loginfo("所有无人机处理完毕")
    rospy.loginfo("在 RViz 中点击 '2D Nav Goal' 触发探索算法")
    rospy.loginfo("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
