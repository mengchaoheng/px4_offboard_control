#ifndef CIRCULAR_TRAJECTORY_H
#define CIRCULAR_TRAJECTORY_H

#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

class Trajectory {
public:
    virtual geometry_msgs::PoseStamped calculate_pose(double time) = 0;
    virtual ~Trajectory() = default;
};

class CircularTrajectory : public Trajectory {
    double radius_, center_x_, center_y_, altitude_, angular_velocity_;

public:
    CircularTrajectory(double r, double x, double y, double z, double w)
        : radius_(r),
          center_x_(x),
          center_y_(y),
          altitude_(z),
          angular_velocity_(w) {}

    geometry_msgs::PoseStamped calculate_pose(double time) override {
        geometry_msgs::PoseStamped pose;
        const double theta = angular_velocity_ * time;

        pose.pose.position.x = center_x_ + radius_ * std::sin(theta);
        pose.pose.position.y = center_y_ + radius_ * std::cos(theta);
        pose.pose.position.z = altitude_;

        return pose;
    }
};

class LissajousTrajectory : public Trajectory {
    double radius_x_;
    double radius_y_;
    double center_x_;
    double center_y_;
    double altitude_;
    double angular_velocity_;
    double z_amplitude_;
    double z_harmonic_;
    bool use_yaw_;

public:
    LissajousTrajectory(double radius_x,
                        double radius_y,
                        double center_x,
                        double center_y,
                        double altitude,
                        double angular_velocity,
                        double z_amplitude,
                        double z_harmonic,
                        bool use_yaw)
        : radius_x_(radius_x),
          radius_y_(radius_y),
          center_x_(center_x),
          center_y_(center_y),
          altitude_(altitude),
          angular_velocity_(angular_velocity),
          z_amplitude_(z_amplitude),
          z_harmonic_(z_harmonic),
          use_yaw_(use_yaw) {}

    geometry_msgs::PoseStamped calculate_pose(double time) override {
        geometry_msgs::PoseStamped pose;
        const mavros_msgs::PositionTarget target = calculate_target_point(time);

        pose.pose.position = target.position;
        pose.pose.orientation.w = 1.0;

        return pose;
    }

    mavros_msgs::PositionTarget calculate_target_point(double time) const {
        mavros_msgs::PositionTarget target;

        const double theta = angular_velocity_ * time;
        const double omega = angular_velocity_;
        const double k = z_harmonic_;

        const double sx = std::sin(theta);
        const double cx = std::cos(theta);
        const double sy = std::sin(2.0 * theta);
        const double cy = std::cos(2.0 * theta);
        const double sz = std::sin(k * theta);
        const double cz = std::cos(k * theta);

        // P
        target.position.x = center_x_ + radius_x_ * sx;
        target.position.y = center_y_ + radius_y_ * sy;
        target.position.z = altitude_ + z_amplitude_ * sz;

        // V
        target.velocity.x = radius_x_ * omega * cx;
        target.velocity.y = 2.0 * radius_y_ * omega * cy;
        target.velocity.z = z_amplitude_ * k * omega * cz;

        // A
        target.acceleration_or_force.x = -radius_x_ * omega * omega * sx;
        target.acceleration_or_force.y = -4.0 * radius_y_ * omega * omega * sy;
        target.acceleration_or_force.z = -z_amplitude_ * k * k * omega * omega * sz;

        // Yaw follows horizontal velocity direction.
        target.yaw = std::atan2(target.velocity.y, target.velocity.x);
        target.yaw_rate = 0.0;

        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        if (!use_yaw_) {
            target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
        }

        return target;
    }

    mavros_msgs::PositionTarget calculate_hold_target(double time) const {
        mavros_msgs::PositionTarget target = calculate_target_point(time);

        target.velocity.x = 0.0;
        target.velocity.y = 0.0;
        target.velocity.z = 0.0;

        target.acceleration_or_force.x = 0.0;
        target.acceleration_or_force.y = 0.0;
        target.acceleration_or_force.z = 0.0;

        target.yaw_rate = 0.0;

        target.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        if (!use_yaw_) {
            target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
        }

        return target;
    }

    double period() const {
        return 2.0 * M_PI / angular_velocity_;
    }
};

#endif  // CIRCULAR_TRAJECTORY_H
