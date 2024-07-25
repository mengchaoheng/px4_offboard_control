#ifndef CIRCULAR_TRAJECTORY_H
#define CIRCULAR_TRAJECTORY_H

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>

// 轨迹基类
class Trajectory {
public:
    virtual geometry_msgs::PoseStamped calculate_pose(double time) = 0;

};

// 圆形轨迹, Or can be change to lissajous
class CircularTrajectory : public Trajectory {
    double radius, center_x, center_y, altitude, angular_velocity;

public:
    CircularTrajectory(double r, double x, double y, double z, double w)
        : radius(r), center_x(x), center_y(y), altitude(z), angular_velocity(w) {}

    geometry_msgs::PoseStamped calculate_pose(double time) override {
        geometry_msgs::PoseStamped pose;
        double theta = angular_velocity * time;
        pose.pose.position.x = center_x + radius * sin(theta);
        // pose.pose.position.y = center_y + radius * cos(theta);  // Circular
        pose.pose.position.y = center_y + radius * sin(2*theta);   // lissajous
        pose.pose.position.z = altitude;
        return pose;
    }
};


// new 
// lissajous
class LissajousTrajectory {
    double radius, center_x, center_y, altitude, angular_velocity;

public:
    LissajousTrajectory(double r, double x, double y, double z, double w)
        : radius(r), center_x(x), center_y(y), altitude(z), angular_velocity(w) {}

    mavros_msgs::PositionTarget calculate_target_point(double time){
        mavros_msgs::PositionTarget target_point;
        // Lissajous
        double theta = angular_velocity * time;

        // x = r*sin((2*pi/T)*t);
        // y = r*sin(2*(2*pi/T)*t);
        target_point.position.x = center_x + radius*sin(theta);
        target_point.position.y = center_y + radius*sin(2* theta);
        target_point.position.z = altitude + 0.05*radius*sin(4*theta);

        // velocity
        target_point.velocity.x = (radius*(angular_velocity))*cos(theta);
        target_point.velocity.y = (radius*(2* angular_velocity))*cos(2* theta);
        target_point.velocity.z = NAN;


        // acc
        target_point.acceleration_or_force.x = NAN;//-(radius*(angular_velocity)*(angular_velocity)) * sin(theta);
        target_point.acceleration_or_force.y = NAN;//-(radius*(2* angular_velocity))*(2* angular_velocity) * sin(2* theta);
        target_point.acceleration_or_force.z = NAN;

        //yaw
        // target_point.yaw = 0.f;
        // _yawspeed_setpoint = NAN;
        target_point.yaw =  atan2f((radius*(2* angular_velocity))*cos(2* theta), (radius*(angular_velocity))*cos(theta));

        target_point.yaw_rate = NAN;

        return target_point;
    }
};


#endif // CIRCULAR_TRAJECTORY_H

