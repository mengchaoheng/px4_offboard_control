#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "my_offboard_node/CircularTrajectory.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// 全局状态变量
mavros_msgs::State current_state;
geometry_msgs::PoseWithCovarianceStamped dataset_pose;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void data_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
    dataset_pose = *pose_msg;
    // ROS_INFO("data_cb: %f",dataset_pose.pose.pose.position.x);
}
typedef enum POSITION_TARGET_TYPEMASK
{
   POSITION_TARGET_TYPEMASK_X_IGNORE=1, /* Ignore position x | */
   POSITION_TARGET_TYPEMASK_Y_IGNORE=2, /* Ignore position y | */
   POSITION_TARGET_TYPEMASK_Z_IGNORE=4, /* Ignore position z | */
   POSITION_TARGET_TYPEMASK_VX_IGNORE=8, /* Ignore velocity x | */
   POSITION_TARGET_TYPEMASK_VY_IGNORE=16, /* Ignore velocity y | */
   POSITION_TARGET_TYPEMASK_VZ_IGNORE=32, /* Ignore velocity z | */
   POSITION_TARGET_TYPEMASK_AX_IGNORE=64, /* Ignore acceleration x | */
   POSITION_TARGET_TYPEMASK_AY_IGNORE=128, /* Ignore acceleration y | */
   POSITION_TARGET_TYPEMASK_AZ_IGNORE=256, /* Ignore acceleration z | */
   POSITION_TARGET_TYPEMASK_FORCE_SET=512, /* Use force instead of acceleration | */
   POSITION_TARGET_TYPEMASK_YAW_IGNORE=1024, /* Ignore yaw | */
   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=2048, /* Ignore yaw rate | */
   POSITION_TARGET_TYPEMASK_ENUM_END=2049, /*  | */
} POSITION_TARGET_TYPEMASK;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~"); //have to be ("~") since need to setup param
    bool _setpoint_raw_mode;
    nh.param<bool>("setpoint_raw_mode", _setpoint_raw_mode, false);
    // ROS_INFO("_setpoint_raw_mode: %d",_setpoint_raw_mode);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("state", 20, state_cb);
    ros::Subscriber data_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/datapose", 20, data_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 20);
    ros::Publisher local_pos_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("setpoint_raw/local", 20);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("set_mode");

    // 设置发布速率大于2Hz
    ros::Rate rate(20.0);

    // 等待与飞控的连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 创建轨迹对象
    CircularTrajectory trajectory(2, 0.0, 0.0, 2.5, 0.5);
    LissajousTrajectory Lissajous(2, 0.0, 0.0, 2.5, 0.5);

    // 发送几个设置点以启动
    geometry_msgs::PoseStamped pose;
    mavros_msgs::PositionTarget target_point;
    // pose = trajectory.calculate_pose(0); // 使用初始位置
    target_point = Lissajous.calculate_target_point(0); // inital point
    for(int i = 30; ros::ok() && i > 0; --i){
        ROS_INFO("Inital");
        if (!_setpoint_raw_mode){
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position=target_point.position;
            // pose.pose.orientation ignore
            local_pos_pub.publish(pose);
        }
        else{
            target_point.header.stamp = ros::Time::now();
            target_point.header.frame_id = "map";
            //ToDo
            // 64	POSITION_TARGET_TYPEMASK_AX_IGNORE	Ignore acceleration x
            // 128	POSITION_TARGET_TYPEMASK_AY_IGNORE	Ignore acceleration y
            // 256	POSITION_TARGET_TYPEMASK_AZ_IGNORE	Ignore acceleration z
            // 1024	POSITION_TARGET_TYPEMASK_YAW_IGNORE	Ignore yaw
            // 2048	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE	Ignore yaw rate
            // target_point.type_mask = 3576; // Ignore Velocity, Acceleration and Yaw
            // target_point.type_mask = 2552; // Ignore Velocity, Acceleration
            // target_point.type_mask = 2496; // Ignore Acceleration
            // target_point.type_mask = 3520; // Ignore Acceleration and Yaw
            // target_point.type_mask = 3072; // Ignore Yaw
            // target_point.type_mask = 2048;
            target_point.type_mask = 3576;  // Ignore Velocity, Acceleration and Yaw in inital step.
            target_point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            local_pos_raw_pub.publish(target_point);
        }
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                // ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    // ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(current_state.mode == "OFFBOARD" && current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            double t = (ros::Time::now() - last_request - ros::Duration(5.0)).toSec();
            // ROS_INFO("Running");
            // pose = trajectory.calculate_pose(t);
            target_point = Lissajous.calculate_target_point(t);
            // When in position control mode, send only waypoints
            if (!_setpoint_raw_mode)
            {
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position=target_point.position;
                // pose.pose.orientation ignore

                local_pos_pub.publish(pose);
            }
            // Send to setpoint_raw, which gives more freedom to what settings to control
            else
            {
                target_point.header.stamp = ros::Time::now();
                target_point.header.frame_id = "map";
                target_point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                // position
                if (!std::isfinite(target_point.position.x)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_X_IGNORE;
                }

                if (!std::isfinite(target_point.position.y)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_Y_IGNORE;
                }

                if (!std::isfinite(target_point.position.z)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE;
                }

                // velocity
                if (!std::isfinite(target_point.velocity.x)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_VX_IGNORE;
                }

                if (!std::isfinite(target_point.velocity.y)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_VY_IGNORE;
                }

                if (!std::isfinite(target_point.velocity.z)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_VZ_IGNORE;
                }

                // acceleration
                if (!std::isfinite(target_point.acceleration_or_force.x)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_AX_IGNORE;
                }

                if (!std::isfinite(target_point.acceleration_or_force.y)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_AY_IGNORE;
                }

                if (!std::isfinite(target_point.acceleration_or_force.z)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_AZ_IGNORE;
                }

                // yaw
                if (!std::isfinite(target_point.yaw)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;
                }

                // yaw rate
                if (!std::isfinite(target_point.yaw_rate)) {
                    target_point.type_mask |= POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
                }
                local_pos_raw_pub.publish(target_point);
            }
        }else{
            // ROS_INFO("Try to armed and flight to inital point");
            if (!_setpoint_raw_mode){
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position=target_point.position;
                // pose.pose.orientation ignore
                local_pos_pub.publish(pose);
            }
            else{
                target_point.header.stamp = ros::Time::now();
                target_point.header.frame_id = "map";
                //ToDo
                // 64	POSITION_TARGET_TYPEMASK_AX_IGNORE	Ignore acceleration x
                // 128	POSITION_TARGET_TYPEMASK_AY_IGNORE	Ignore acceleration y
                // 256	POSITION_TARGET_TYPEMASK_AZ_IGNORE	Ignore acceleration z
                // 1024	POSITION_TARGET_TYPEMASK_YAW_IGNORE	Ignore yaw
                // 2048	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE	Ignore yaw rate
                // target_point.type_mask = 3576; // Ignore Velocity, Acceleration and Yaw
                // target_point.type_mask = 2552; // Ignore Velocity, Acceleration
                // target_point.type_mask = 2496; // Ignore Acceleration
                // target_point.type_mask = 3520; // Ignore Acceleration and Yaw
                // target_point.type_mask = 3072; // Ignore Yaw
                // target_point.type_mask = 2048;
                target_point.type_mask = 3576;  // Ignore Velocity, Acceleration and Yaw in inital step.
                target_point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                local_pos_raw_pub.publish(target_point);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

