#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include "my_offboard_node/CircularTrajectory.h"

mavros_msgs::State current_state;
geometry_msgs::PoseStamped actual_pose;
bool actual_pose_received = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void actual_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    actual_pose = *msg;
    actual_pose_received = true;
}

void actual_cov_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    actual_pose.header = msg->header;
    actual_pose.pose = msg->pose.pose;
    actual_pose_received = true;
}

geometry_msgs::Quaternion yaw_to_quaternion(double yaw) {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

geometry_msgs::PoseStamped target_to_pose(const mavros_msgs::PositionTarget& target,
                                          const std::string& frame_id) {
    geometry_msgs::PoseStamped pose;
    pose.header = target.header;
    pose.header.frame_id = frame_id;
    pose.pose.position = target.position;
    pose.pose.orientation = yaw_to_quaternion(target.yaw);
    return pose;
}

nav_msgs::Path build_reference_path(const LissajousTrajectory& trajectory,
                                    const std::string& frame_id,
                                    double cycles,
                                    int samples) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id;

    samples = std::max(samples, 2);
    const double duration = cycles * trajectory.period();

    path.poses.reserve(static_cast<size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double t = ratio * duration;

        mavros_msgs::PositionTarget target = trajectory.calculate_target_point(t);
        target.header.stamp = path.header.stamp;
        target.header.frame_id = frame_id;

        path.poses.push_back(target_to_pose(target, frame_id));
    }

    return path;
}

void trim_path(nav_msgs::Path& path, int max_size) {
    if (max_size <= 0) {
        return;
    }

    while (static_cast<int>(path.poses.size()) > max_size) {
        path.poses.erase(path.poses.begin());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "offb_node_pva");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool setpoint_raw_mode;
    bool auto_arm;
    bool auto_offboard;
    bool use_yaw;

    double radius_x;
    double radius_y;
    double center_x;
    double center_y;
    double altitude;
    double angular_velocity;
    double z_amplitude;
    double z_harmonic;
    double publish_rate;
    double start_delay;
    double reference_path_cycles;

    int preflight_setpoint_count;
    int actual_tail_size;
    int reference_path_samples;

    std::string frame_id;
    std::string state_topic;
    std::string raw_setpoint_topic;
    std::string local_setpoint_topic;
    std::string arming_service;
    std::string set_mode_service;
    std::string actual_pose_topic;
    std::string actual_cov_pose_topic;

    pnh.param<bool>("setpoint_raw_mode", setpoint_raw_mode, true);
    pnh.param<bool>("auto_arm", auto_arm, true);
    pnh.param<bool>("auto_offboard", auto_offboard, true);
    pnh.param<bool>("use_yaw", use_yaw, true);

    pnh.param<double>("radius_x", radius_x, 4.0);
    pnh.param<double>("radius_y", radius_y, 4.0);
    pnh.param<double>("center_x", center_x, 0.0);
    pnh.param<double>("center_y", center_y, 0.0);
    pnh.param<double>("altitude", altitude, 2.5);
    pnh.param<double>("angular_velocity", angular_velocity, 0.35);
    pnh.param<double>("z_amplitude", z_amplitude, 0.20);
    pnh.param<double>("z_harmonic", z_harmonic, 4.0);
    pnh.param<double>("publish_rate", publish_rate, 20.0);
    pnh.param<double>("start_delay", start_delay, 3.0);
    pnh.param<double>("reference_path_cycles", reference_path_cycles, 1.0);

    pnh.param<int>("preflight_setpoint_count", preflight_setpoint_count, 100);
    pnh.param<int>("actual_tail_size", actual_tail_size, 600);
    pnh.param<int>("reference_path_samples", reference_path_samples, 240);

    pnh.param<std::string>("frame_id", frame_id, "map");
    pnh.param<std::string>("state_topic", state_topic, "/mavros/state");
    pnh.param<std::string>("raw_setpoint_topic", raw_setpoint_topic, "/mavros/setpoint_raw/local");
    pnh.param<std::string>("local_setpoint_topic", local_setpoint_topic, "/mavros/setpoint_position/local");
    pnh.param<std::string>("arming_service", arming_service, "/mavros/cmd/arming");
    pnh.param<std::string>("set_mode_service", set_mode_service, "/mavros/set_mode");
    pnh.param<std::string>("actual_pose_topic", actual_pose_topic, "/mavros/local_position/pose");
    pnh.param<std::string>("actual_cov_pose_topic", actual_cov_pose_topic, "/datapose");

    if (angular_velocity <= 0.0) {
        ROS_ERROR("angular_velocity must be positive.");
        return 1;
    }

    if (publish_rate < 2.0) {
        ROS_ERROR("publish_rate must be at least 2 Hz.");
        return 1;
    }

    LissajousTrajectory trajectory(radius_x,
                                   radius_y,
                                   center_x,
                                   center_y,
                                   altitude,
                                   angular_velocity,
                                   z_amplitude,
                                   z_harmonic,
                                   use_yaw);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(state_topic, 20, state_cb);
    ros::Subscriber actual_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(actual_pose_topic, 50, actual_pose_cb);
    ros::Subscriber actual_cov_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(actual_cov_pose_topic, 50, actual_cov_pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(local_setpoint_topic, 20);
    ros::Publisher local_pos_raw_pub = nh.advertise<mavros_msgs::PositionTarget>(raw_setpoint_topic, 20);

    ros::Publisher reference_pva_pub = nh.advertise<mavros_msgs::PositionTarget>("/reference_pva", 20);
    ros::Publisher reference_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/reference_pose", 20);
    ros::Publisher actual_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/actual_pose", 20);
    ros::Publisher reference_path_pub = nh.advertise<nav_msgs::Path>("/reference_path", 1, true);
    ros::Publisher actual_path_pub = nh.advertise<nav_msgs::Path>("/actual_path_tail", 20);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(arming_service);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(set_mode_service);

    ros::Rate rate(publish_rate);

    nav_msgs::Path reference_path = build_reference_path(trajectory, frame_id, reference_path_cycles, reference_path_samples);
    reference_path_pub.publish(reference_path);

    nav_msgs::Path actual_path;
    actual_path.header.frame_id = frame_id;

    while (ros::ok() && !current_state.connected) {
        reference_path.header.stamp = ros::Time::now();
        reference_path_pub.publish(reference_path);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget hold_target = trajectory.calculate_hold_target(0.0);
    hold_target.header.frame_id = frame_id;

    for (int i = 0; ros::ok() && i < preflight_setpoint_count; ++i) {
        hold_target.header.stamp = ros::Time::now();

        if (setpoint_raw_mode) {
            local_pos_raw_pub.publish(hold_target);
        } else {
            local_pos_pub.publish(target_to_pose(hold_target, frame_id));
        }

        reference_pva_pub.publish(hold_target);
        reference_pose_pub.publish(target_to_pose(hold_target, frame_id));

        reference_path.header.stamp = ros::Time::now();
        reference_path_pub.publish(reference_path);

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool trajectory_started = false;
    ros::Time trajectory_start_time;

    while (ros::ok()) {
        const ros::Time now = ros::Time::now();

        if (auto_offboard &&
            current_state.mode != "OFFBOARD" &&
            now - last_request > ros::Duration(2.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD requested.");
            }
            last_request = now;
        }

        if (auto_arm &&
            current_state.mode == "OFFBOARD" &&
            !current_state.armed &&
            now - last_request > ros::Duration(2.0)) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Arming requested.");
            }
            last_request = now;
        }

        const bool ready_for_trajectory =
            current_state.mode == "OFFBOARD" &&
            current_state.armed &&
            now - last_request > ros::Duration(start_delay);

        mavros_msgs::PositionTarget target;

        if (ready_for_trajectory) {
            if (!trajectory_started) {
                trajectory_started = true;
                trajectory_start_time = now;
                ROS_INFO("PVA trajectory started.");
            }

            const double t = (now - trajectory_start_time).toSec();
            target = trajectory.calculate_target_point(t);
        } else {
            target = trajectory.calculate_hold_target(0.0);
        }

        target.header.stamp = now;
        target.header.frame_id = frame_id;
        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        if (setpoint_raw_mode) {
            local_pos_raw_pub.publish(target);
        } else {
            local_pos_pub.publish(target_to_pose(target, frame_id));
        }

        reference_pva_pub.publish(target);
        reference_pose_pub.publish(target_to_pose(target, frame_id));

        reference_path.header.stamp = now;
        reference_path_pub.publish(reference_path);

        if (actual_pose_received) {
            geometry_msgs::PoseStamped actual = actual_pose;
            actual.header.stamp = now;
            actual.header.frame_id = frame_id;

            actual_pose_pub.publish(actual);

            actual_path.header.stamp = now;
            actual_path.header.frame_id = frame_id;
            actual_path.poses.push_back(actual);
            trim_path(actual_path, actual_tail_size);
            actual_path_pub.publish(actual_path);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
