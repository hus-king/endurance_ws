#ifndef FLY_CTL_MOTION_CONTROL_H
#define FLY_CTL_MOTION_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <cmath>

namespace fly_ctl {

// 位姿结构
struct dronePose {
    double x, y, z, yaw;
    dronePose(double x_=0, double y_=0, double z_=0, double yaw_=0) 
        : x(x_), y(y_), z(z_), yaw(yaw_) {}
};

// 速度指令结构
struct Velocity {
    double vx, vy, vz;
    Velocity(double vx_=0, double vy_=0, double vz_=0)
        : vx(vx_), vy(vy_), vz(vz_) {}
};

// 飞行控制器类
class MotionController {
private:
    // ROS句柄和发布器/订阅器
    ros::NodeHandle nh_;
    ros::Publisher setpoint_raw_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;

    // 控制状态
    mavros_msgs::PositionTarget setpoint_raw_;
    
    // 当前状态
    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_odom_;
    double current_roll_, current_pitch_, current_yaw_;
    bool has_odom_;
    
    // 起飞初始位置
    double init_x_, init_y_, init_z_, init_yaw_;
    bool has_init_position_;
    
    // 控制参数
    double control_rate_;
    double position_tolerance_;
    double max_yaw_rate_;


    // 时间记录
    ros::Time last_ctl_time_; // 目前用于限制yaw转速
    ros::Time hover_start_time_;

    // 控制标志
    bool move_to_position_flag_ = false;
    bool moveRelative_flag_ = false;
    bool hover_flag_ = false;
    bool vel_ctl_flag_ = false;

    // 相对位置控制参数:
    dronePose relative_target_;
    dronePose hover_target_;

    // 话题名称:
    std::string setpoint_topic_;
    std::string state_topic_;
    std::string odom_topic_;

public:
    // 构造函数
    MotionController(ros::NodeHandle& nh);
    
    // 初始化函数
    void initialize();
    
    // 状态回调

    // 对应state_cb
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    // 对应local_pos_cb
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // 飞行模式控制
    bool setOffboardMode();
    bool arm();
    // bool disarm();
    
    // 基础飞行控制
    bool takeoff(double height, double timeout = 10.0);
    bool land(double timeout = 10.0);
    bool hover(double timeout = 1.0);
    
    // 位置控制（世界坐标系，相对于起飞点）
    bool moveToPosition(const dronePose& target);
    bool moveToXYZ(double x, double y, double z, double yaw = 0.0);

    bool moveRelativeXYZ(double dx, double dy, double dz);
    void setRelativeTarget(double &dx, double &dy, double &dz);

    // 用这个函数来启动或停止相对移动
    void setRelativeStart(bool flag);

    // 速度控制（机体坐标系）
    bool moveWithVelocity(const Velocity& vel, double target_yaw = 0.0);
    bool moveWithVelocity(double vx, double vy, double vz, double yaw_rate = 0.0, double target_yaw = 0.0);
    
    // 状态查询
    bool isArmed() const;
    bool isInAir() const;
    dronePose getCurrentPosition() const;
    dronePose getInitPosition() const;
    double getCurrentYaw() const;
    
    // 辅助函数
    void updateYawWithRateLimit(double target_yaw, double dt);
    // void updateYawWithRateLimit(double target_yaw, double dt, double max_yaw_rate);
    bool waitForPosition(const dronePose& target, double tolerance, double timeout);
    void publishSetpoint(const mavros_msgs::PositionTarget& setpoint);
    
private:
    double updateYawWithRateLimit(double target_yaw, double dt);
    void extractRPY(const geometry_msgs::Quaternion& quat, double& roll, double& pitch, double& yaw);
};

} // namespace fly_ctl

#endif // FLY_CTL_MOTION_CONTROL_H
