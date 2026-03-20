/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
visualization_msgs::Marker trackpoint;
ros::Publisher *pubMarkerPointer;
unsigned short velocity_mask = VELOCITY2D_CONTROL;
mavros_msgs::PositionTarget current_goal;
mavros_msgs::RCIn rc;
int rc_value, flag = 0, flag1 = 0;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped target_pos;
mavros_msgs::State current_state;
float position_x_begin, position_y_begin, position_z_begin, yaw_begin;
bool get_first_pos = false;
float position_x, position_y, position_z, current_yaw, targetpos_x, targetpos_y;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
bool receive = false; //触发轨迹的条件判断
float pi = 3.14159265;

float global_offset_x = 0;
float global_offset_y = -4.0;
float global_offset_z = 0.0;

// read RC 5 channel pwm, estimate auto plan in certain range
void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
    rc = *msg;
    rc_value = rc.channels[4];
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// read vehicle odometry
void position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_msg = *msg;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat); // 把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (!get_first_pos)
    {
        position_x_begin = position_msg.pose.pose.position.x;
        position_y_begin = position_msg.pose.pose.position.y;
        position_z_begin = position_msg.pose.pose.position.z;
        yaw_begin = yaw;
        get_first_pos = true;
    }
    position_x = position_msg.pose.pose.position.x - position_x_begin + global_offset_x;
    position_y = position_msg.pose.pose.position.y - position_y_begin + global_offset_y;
    position_z = position_msg.pose.pose.position.z - position_z_begin + global_offset_z;
    current_yaw = yaw;
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) // 读取rviz的航点
{
    target_pos = *msg;
    targetpos_x = target_pos.pose.position.x;
    targetpos_y = target_pos.pose.position.y;
}

// 读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
quadrotor_msgs::PositionCommand ego;
void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) // ego的回调函数
{
    ROS_INFO("收到ego的数据了");
    receive = true;
    ego = *msg;
    ego_pos_x = ego.position.x;
    ego_pos_y = ego.position.y;
    ego_pos_z = ego.position.z;
    ego_vel_x = ego.velocity.x;
    ego_vel_y = ego.velocity.y;
    ego_vel_z = ego.velocity.z;
    ego_a_x = ego.acceleration.x;
    ego_a_y = ego.acceleration.y;
    ego_a_z = ego.acceleration.z;
    ego_yaw = ego.yaw + yaw_begin;
    ego_yaw_rate = ego.yaw_dot;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_iris_0");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    std::cout << "start" << std::endl;

    // 初始化订阅者和服务客户端
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 1);
    ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
    pubMarkerPointer = &pubMarker;
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/iris_0/mavros/rc/in", 10, rc_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
    ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd_1", 10, twist_cb);
    ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 10, target_cb);
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/iris_0/mavros/local_position/odom", 10, position_cb);
    ros::Rate rate(50.0);

    // 等待飞控连接
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // 切换到OFFBOARD模式并解锁
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 预发送设定点
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    // 切换模式并解锁
    ros::Time last_request = ros::Time::now();
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard模式已激活");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("无人机iris_0已解锁");
                }
            }
        }

        // 发布设定点保持模式
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();

        // 模式切换和解锁完成后，执行控制逻辑
        if (current_state.mode == "OFFBOARD" && current_state.armed)
        {
            break;
        }
    }

    // 控制逻辑
    while (ros::ok())
    {
        // take off 1m
        if (!receive)
        {
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_goal.header.stamp = ros::Time::now();
            current_goal.type_mask = velocity_mask;
            current_goal.velocity.x = (global_offset_x - position_x) * 1;
            current_goal.velocity.y = (global_offset_y - position_y) * 1;
            current_goal.velocity.z = (global_offset_z + 1 - position_z) * 1;
            current_goal.yaw = current_yaw;
            ROS_INFO("请等待");
            local_pos_pub.publish(current_goal);
            ros::spinOnce();
            rate.sleep();
        }

        // if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
        if (receive)
        {
            float yaw_erro;
            yaw_erro = (ego_yaw - current_yaw);
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_goal.header.stamp = ros::Time::now();
            current_goal.type_mask = velocity_mask;
            current_goal.velocity.x = (ego_pos_x - position_x) * 2;
            current_goal.velocity.y = (ego_pos_y - position_y) * 2;
            current_goal.velocity.z = (ego_pos_z - position_z) * 2;
            current_goal.yaw = ego_yaw;
            ROS_INFO("EGO规划速度：vel_x = %.2f", sqrt(pow(current_goal.velocity.x, 2) + pow(current_goal.velocity.y, 2)));
        }
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}