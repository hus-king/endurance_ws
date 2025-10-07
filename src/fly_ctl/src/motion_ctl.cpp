#include "fly_ctl/motion_control.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace fly_ctl {

MotionController::MotionController(ros::NodeHandle& nh) 
    : nh_(nh)
    , setpoint_topic_("/mavros/setpoint_raw/local") // 直接赋值
    , state_topic_("/mavros/state")
    , odom_topic_("/mavros/local_position/odom")
    , has_odom_(false)
    , has_init_position_(false)
    , control_rate_(20.0)
    , position_tolerance_(0.1)
    , max_yaw_rate_(1.5)
    , current_roll_(0.0)
    , current_pitch_(0.0)
    , current_yaw_(0.0)
{
    // 初始化发布器和订阅器
    setpoint_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(setpoint_topic_, 10);
    state_sub_ = nh_.subscribe(state_topic_, 10, &MotionController::stateCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MotionController::odomCallback, this);
    
    // 从参数服务器获取配置
    nh_.param("control_rate", control_rate_, 20.0);
    nh_.param("position_tolerance", position_tolerance_, 0.1);
    nh_.param("max_yaw_rate", max_yaw_rate_, 1.5);
}

// 理论上你有回调函数用了的话就不需要跑这个函数了
void MotionController::initialize() {
    // 等待有有效的里程计数据
    ros::Rate rate(10);
    while (ros::ok() && !has_odom_) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // 初始化起飞位置, 如果还没有初始化, 保险起见
    if (!has_init_position_ && current_odom_.pose.pose.position.z != 0) {
        init_x_ = current_odom_.pose.pose.position.x;
        init_y_ = current_odom_.pose.pose.position.y;
        init_z_ = current_odom_.pose.pose.position.z;
        init_yaw_ = current_yaw_;
        has_init_position_ = true;
        ROS_INFO("Initialized takeoff position: (%.2f, %.2f, %.2f), yaw: %.2f", 
                 init_x_, init_y_, init_z_, init_yaw_);
    }
}

// 回调函数接收无人机的状态信息
void MotionController::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

// 回调函数接收无人机的里程计信息
void MotionController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
    has_odom_ = true;
    
    // 提取当前姿态
    extractRPY(current_odom_.pose.pose.orientation, current_roll_, current_pitch_, current_yaw_);
    
    // 自动初始化起飞位置（如果还没初始化）
    if (!has_init_position_ && current_odom_.pose.pose.position.z != 0) {
        init_x_ = current_odom_.pose.pose.position.x;
        init_y_ = current_odom_.pose.pose.position.y;
        init_z_ = current_odom_.pose.pose.position.z;
        init_yaw_ = current_yaw_;
        has_init_position_ = true;
        ROS_INFO("Auto-initialized takeoff position: (%.2f, %.2f, %.2f), yaw: %.2f", 
                 init_x_, init_y_, init_z_, init_yaw_);
    }
}

// 提取欧拉角
void MotionController::extractRPY(const geometry_msgs::Quaternion& quat, 
                                  double& roll, double& pitch, double& yaw) {
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
}

bool MotionController::setOffboardMode() {
    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD mode enabled");
        return true;
    }
    ROS_WARN("Failed to set OFFBOARD mode");
    return false;
}

bool MotionController::arm() {
    ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone armed successfully");
        return true;
    }
    ROS_WARN("Failed to arm drone");
    return false;
}

// bool MotionController::disarm() {
//     ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = false;
    
//     if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
//         ROS_INFO("Drone disarmed successfully");
//         return true;
//     }
//     ROS_WARN("Failed to disarm drone");
//     return false;
// }

bool MotionController::takeoff(double height, double timeout) {
    if (!has_init_position_) {
        ROS_ERROR("Cannot takeoff: no initial position available");
        return false;
    }
    
    dronePose target_pose(init_x_, init_y_, init_z_ + height, init_yaw_);
    return moveToPosition(target_pose);
}

// bool MotionController::land(double timeout) {
//     if (!has_init_position_) {
//         ROS_ERROR("Cannot land: no initial position available");
//         return false;
//     }
    
//     // 降落到起飞点高度
//     dronePose land_pose(init_x_, init_y_, init_z_, current_yaw_);
//     return moveToPosition(land_pose);
// }

bool MotionController::hover(double timeout) {
    if (!has_odom_) {
        ROS_ERROR("Cannot hover: no odometry data available");
        return false;
    }
    if (hover_flag_ == false) {
        hover_flag_ = true;
        hover_start_time_ = ros::Time::now();
        hover_target_ = getCurrentPosition();
    }
    if ((ros::Time::now() - hover_start_time_).toSec() >= timeout) {
        hover_flag_ = false;
        ROS_INFO("Hover time completed");
        return true;
    }
    moveToPosition(hover_target_);
    return false;
}

bool MotionController::moveToPosition(const dronePose& target) {
    if (!has_odom_) {
        ROS_ERROR("Cannot move to position: no odometry data available");
        return false;
    }
        // 检查是否到达目标
        double dx = current_odom_.pose.pose.position.x - target.x;
        double dy = current_odom_.pose.pose.position.y - target.y;
        double dz = current_odom_.pose.pose.position.z - target.z;
        double dyaw = current_yaw_ - target.yaw;

        // 角度归一化到 [-π, π]
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;

        if (fabs(dx) < position_tolerance_ && fabs(dy) < position_tolerance_ && fabs(dz) < position_tolerance_ && fabs(dyaw) < position_tolerance_) {
            ROS_INFO("Reached target position");
            move_to_position_flag_ = false;
            return true;
        }
        if (move_to_position_flag_ == false) {
            move_to_position_flag_ = true;
            last_ctl_time_ = ros::Time::now();
        }

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_ctl_time_).toSec();
        double  new_yaw = current_yaw_;
        if (dt > 0.01) { // 避免除零
            new_yaw = updateYawWithRateLimit(target.yaw, dt);
            last_ctl_time_ = current_time;
        } else {
            dt = 0.05; // 假设一个小的dt
            new_yaw = updateYawWithRateLimit(target.yaw, dt);
            last_ctl_time_ = current_time;
        }
        // publishSetpoint(setpoint); 只在最后while后发布
        mavros_msgs::PositionTarget setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
        
        setpoint.position.x = target.x;
        setpoint.position.y = target.y;
        setpoint.position.z = target.z;
        setpoint.yaw = target.yaw;
        setpoint_raw_ = setpoint;
    return false;
}

// 封装了一下, 一坨
bool MotionController::moveToXYZ(double x, double y, double z, double yaw) {
    dronePose target(x, y, z, yaw);
    return moveToPosition(target);
}

bool MotionController::moveRelativeXYZ(double dx, double dy, double dz) {
    if (!moveRelative_flag_) {
        moveRelative_flag_ = true;
        setRelativeTarget(dx, dy, dz);
    }
    bool has_reached = moveToPosition(relative_target_);
    if (has_reached) {
        moveRelative_flag_ = false;
    }
    return has_reached;
    // return moveToPosition(relative_target_);
}

void setRelativeTarget(double &dx, double &dy, double &dz) {
    double target_x = current_odom_.pose.pose.position.x + dx * cos(current_yaw_) - dy * sin(current_yaw_);
    double target_y = current_odom_.pose.pose.position.y + dx * sin(current_yaw_) + dy * cos(current_yaw_);
    double target_z = current_odom_.pose.pose.position.z + dz;
    double target_yaw = current_yaw_;
    // 角度归一化到 [-π, π]
    while (target_yaw > M_PI) target_yaw -= 2 * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2 * M_PI;
    relative_target_ = dronePose(target_x, target_y, target_z, target_yaw);
}

void setRelativeStart(bool flag) {
    moveRelative_flag_ = flag;
}


// 角度直接用位置控制
bool MotionController::moveWithVelocity(const Velocity& vel, double target_yaw) {
    if (!vel_ctl_flag_) {
        vel_ctl_flag_ = true;
    }
     // 计算时间差
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_ctl_time_).toSec();
    if (dt < 0.01) dt = 0.05; //
    last_ctl_time_ = current_time;
    double new_yaw = updateYawWithRateLimit(target_yaw, dt);

    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = 1 + 2 + 4 + /*8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048; // 只控制速度和偏航角
    setpoint.velocity.x = vel.vx;
    setpoint.velocity.y = vel.vy;
    setpoint.velocity.z = vel.vz;
    setpoint.yaw = new_yaw;
    setpoint_raw_ = setpoint;
    return true;
}

bool MotionController::moveWithVelocity(double vx, double vy, double vz, double yaw_rate, double target_yaw) {
    Velocity vel(vx, vy, vz, yaw_rate);
    return moveWithVelocity(vel, target_yaw);
}

bool MotionController::isArmed() const {
    return current_state_.armed;
}

// ?
bool MotionController::isInAir() const {
    return has_odom_ && current_odom_.pose.pose.position.z > (init_z_ + 0.1);
}

dronePose MotionController::getCurrentPosition() const {
    return dronePose(current_odom_.pose.pose.position.x,
                     current_odom_.pose.pose.position.y,
                     current_odom_.pose.pose.position.z,
                     current_yaw_);
}

dronePose MotionController::getInitPosition() const {
    return dronePose(init_x_, init_y_, init_z_, init_yaw_);
}

double MotionController::getCurrentYaw() const {
    return current_yaw_;
}

void MotionController::publishSetpoint() {
    setpoint_raw_pub_.publish(setpoint_raw_);
}

void MotionController::updateYawWithRateLimit(double target_yaw, double dt) {
    double yaw_diff = target_yaw - current_yaw_;
    // 角度归一化到 [-π, π]
    while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
    
    double max_yaw_change = max_yaw_rate_ * dt;
    if (fabs(yaw_diff) > max_yaw_change) {
        yaw_diff = (yaw_diff > 0) ? max_yaw_change : -max_yaw_change;
    }
    return current_yaw_ + yaw_diff;
    // current_yaw_ += yaw_diff;
} // namespace fly_ctl

