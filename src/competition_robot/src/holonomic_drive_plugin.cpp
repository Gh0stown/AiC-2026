/*
 * holonomic_drive_plugin.cpp
 *
 * 全向(麦克纳姆)底盘驱动插件 —— 用来替代 gazebo_ros_planar_move。
 *
 * 为什么要自己写:
 *   gazebo_ros_planar_move 实测在实际自转只有指令的 0.637 倍 (线速度精确)。
 *   它内部调 Model::SetLinearVel / SetAngularVel, 而 Gazebo 文档说这两个函数
 *   会"设置模型及其所有链接"的速度 —— 那就把车轮的角速度锁死了(轮子不能自转),
 *   自转时轮子只能侧滑, 摩擦把角速度吃掉。
 *
 *   本插件把几种施加方式都做出来, 用 <velocityMode> 选择, 方便实测对比:
 *     model      = 和 planar_move 一样, 调 Model::SetLinearVel/AngularVel
 *     all_rigid  = 自己按刚体速度分布给每个 link 设速度  v_i = v + w × r_i
 *     canonical  = 只设 base(第一个 link), 让轮子通过关节自由滚动   <- 默认
 *
 * 同时发布一份真值里程计(位姿来自 Gazebo, 不是积分), 便于对比调试。
 */

#include <cmath>
#include <string>

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace competition_robot {

class HolonomicDrivePlugin : public gazebo::ModelPlugin {
 public:
  HolonomicDrivePlugin() = default;
  ~HolonomicDrivePlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;

    // ---------------- 参数 ----------------
    std::string ns = "/";
    if (sdf->HasElement("robotNamespace")) {
      ns = sdf->Get<std::string>("robotNamespace");
      if (ns.empty()) ns = "/";
    }
    cmd_topic_ = GetSdfStr(sdf, "commandTopic", "cmd_vel");
    odom_topic_ = GetSdfStr(sdf, "odometryTopic", "odom_groundtruth");
    odom_frame_ = GetSdfStr(sdf, "odometryFrame", "odom_groundtruth");
    base_frame_ = GetSdfStr(sdf, "robotBaseFrame", "base_footprint");
    mode_ = GetSdfStr(sdf, "velocityMode", "canonical");
    odom_rate_ = sdf->HasElement("odometryRate")
                     ? sdf->Get<double>("odometryRate") : 50.0;
    cmd_timeout_ = sdf->HasElement("cmdTimeout")
                       ? sdf->Get<double>("cmdTimeout") : 0.5;
    publish_odom_ = !sdf->HasElement("publishOdometry") ||
                    sdf->Get<bool>("publishOdometry");
    // ★ 默认**不**广播 odom_groundtruth -> base_footprint 的 TF。
    //   因为 mecanum_odometry.py 已经广播了 odom -> base_footprint,
    //   两个都发的话 base_footprint 会有两个父节点, TF 树会报警告、RViz 会认不出来。
    //   真值只通过 /odom_groundtruth 话题给, 需要 TF 调试时再打开。
    publish_tf_ = sdf->HasElement("publishTf") && sdf->Get<bool>("publishTf");

    if (!ros::isInitialized()) {
      ROS_FATAL("holonomic_drive: ROS 没初始化, 请先加载 libgazebo_ros_api_plugin.so");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(ns));

    cmd_sub_ = rosnode_->subscribe<geometry_msgs::Twist>(
        cmd_topic_, 1, &HolonomicDrivePlugin::CmdVelCallback, this);
    if (publish_odom_) {
      odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odom_topic_, 10);
      tf_broadcaster_.reset(new tf::TransformBroadcaster());
    }

    // 找 canonical link (第一个 link, URDF 里是根节点)
    const auto &links = model_->GetLinks();
    if (!links.empty()) canonical_link_ = links.front();

    last_cmd_time_ = model_->GetWorld()->SimTime();
    last_odom_time_ = last_cmd_time_;

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&HolonomicDrivePlugin::OnUpdate, this, _1));

    ROS_INFO("holonomic_drive: 已加载 (mode=%s, cmd=%s, odom=%s, links=%zu)",
             mode_.c_str(), cmd_topic_.c_str(), odom_topic_.c_str(), links.size());
  }

 private:
  std::string GetSdfStr(const sdf::ElementPtr &sdf, const std::string &key,
                        const std::string &def) {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : def;
  }

  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    lock_.lock();
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
    last_cmd_time_ = model_->GetWorld()->SimTime();
    lock_.unlock();
  }

  void OnUpdate(const gazebo::common::UpdateInfo &info) {
    const double t = info.simTime.Double();

    // 指令超时 -> 停车
    double vx, vy, wz;
    lock_.lock();
    if ((info.simTime - last_cmd_time_).Double() > cmd_timeout_) {
      vx = vy = wz = 0.0;
    } else {
      vx = cmd_vx_; vy = cmd_vy_; wz = cmd_wz_;
    }
    lock_.unlock();

    const ignition::math::Pose3d pose = model_->WorldPose();
    const double yaw = pose.Rot().Yaw();

    // 车体系指令 -> 世界系
    const ignition::math::Vector3d v_world(
        vx * std::cos(yaw) - vy * std::sin(yaw),
        vy * std::cos(yaw) + vx * std::sin(yaw),
        0.0);
    const ignition::math::Vector3d w_world(0.0, 0.0, wz);

    if (mode_ == "model") {
      model_->SetLinearVel(v_world);
      model_->SetAngularVel(w_world);
    } else if (mode_ == "all_rigid") {
      // 正确的刚体速度分布: v_i = v + w × r_i, ω_i = w
      for (const auto &link : model_->GetLinks()) {
        const ignition::math::Vector3d r =
            link->WorldPose().Pos() - pose.Pos();
        link->SetLinearVel(v_world + w_world.Cross(r));
        link->SetAngularVel(w_world);
      }
    } else {  // canonical: 只设根 link, 让轮子通过关节自由滚动
      if (canonical_link_) {
        canonical_link_->SetLinearVel(v_world);
        canonical_link_->SetAngularVel(w_world);
      }
    }

    // ---------------- 真值里程计 ----------------
    if (publish_odom_ && odom_rate_ > 0.0 &&
        (info.simTime - last_odom_time_).Double() > 1.0 / odom_rate_) {
      PublishOdometry(info, pose, vx, vy, wz);
      last_odom_time_ = info.simTime;
    }
  }

  void PublishOdometry(const gazebo::common::UpdateInfo &info,
                       const ignition::math::Pose3d &pose,
                       double vx, double vy, double wz) {
    (void)vx; (void)vy; (void)wz;
    const ros::Time stamp(info.simTime.sec, info.simTime.nsec);
    const ignition::math::Quaterniond q = pose.Rot();

    // ★ 关键: 这里报的必须是**实测**速度 (由位姿差分得到), 不是指令值。
    //   否则拿这个 topic 去校验驱动保真度等于自己跟自己比, 没有意义。
    double ax = 0.0, ay = 0.0, aw = 0.0;
    if (has_last_pose_) {
      const double dt = (info.simTime - last_odom_time_).Double();
      if (dt > 1e-6) {
        const ignition::math::Vector3d dp = pose.Pos() - last_odom_pose_.Pos();
        double dyaw = pose.Rot().Yaw() - last_odom_pose_.Rot().Yaw();
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        const double yaw = pose.Rot().Yaw();
        // 世界系差分 -> 车体系
        ax = std::cos(yaw) * dp.X() + std::sin(yaw) * dp.Y();
        ay = std::cos(yaw) * dp.Y() - std::sin(yaw) * dp.X();
        aw = dyaw;
        ax /= dt; ay /= dt; aw /= dt;
      }
    }
    last_odom_pose_ = pose;
    has_last_pose_ = true;

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = q.X();
    odom.pose.pose.orientation.y = q.Y();
    odom.pose.pose.orientation.z = q.Z();
    odom.pose.pose.orientation.w = q.W();
    odom.twist.twist.linear.x = ax;
    odom.twist.twist.linear.y = ay;
    odom.twist.twist.angular.z = aw;
    odom_pub_.publish(odom);

    tf::Transform tf;
    tf.setOrigin(tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
    tf.setRotation(tf::Quaternion(q.X(), q.Y(), q.Z(), q.W()));
    if (publish_tf_) {
      tf_broadcaster_->sendTransform(
          tf::StampedTransform(tf, stamp, odom_frame_, base_frame_));
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr canonical_link_;
  gazebo::event::ConnectionPtr update_conn_;

  boost::shared_ptr<ros::NodeHandle> rosnode_;
  ros::Subscriber cmd_sub_;
  ros::Publisher odom_pub_;
  boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

  boost::mutex lock_;
  double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;
  gazebo::common::Time last_cmd_time_, last_odom_time_;
  ignition::math::Pose3d last_odom_pose_;   // 用于位姿差分算**实测**速度
  bool has_last_pose_ = false;

  std::string cmd_topic_, odom_topic_, odom_frame_, base_frame_, mode_;
  double odom_rate_ = 50.0, cmd_timeout_ = 0.5;
  bool publish_odom_ = true;
  bool publish_tf_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(HolonomicDrivePlugin)

}  // namespace competition_robot
