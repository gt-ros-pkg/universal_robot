#ifndef UR_STATE_CONTROLLERS_H
#define UR_STATE_CONTROLLERS_H

#define FLOAT64

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

#include <ur_ctrl_client/ur_config_iface.h>

using realtime_tools::RealtimePublisher;
using boost::lexical_cast;

namespace ur_controllers
{

class StatePublishController: 
  public controller_interface::Controller<ur::URConfigInterface>
{
public:
  StatePublishController() {}

  bool init(ur::URConfigInterface* hw, ros::NodeHandle &n)
  {
    config_hdl_ = hw->getHandle("config_command");

    double pub_rate = 125.0;
    n.getParam("publish_rate", pub_rate);
    pub_period_ = 1.0/pub_rate;

    act_q_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "actual_q", 1, true));
    act_q_pub_->msg_.data.resize(6);
    act_qd_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "actual_qd", 1, true));
    act_qd_pub_->msg_.data.resize(6);
    act_i_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "actual_i", 1, true));
    act_i_pub_->msg_.data.resize(6);
    des_q_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "desired_q", 1, true));
    des_q_pub_->msg_.data.resize(6);
    des_qd_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "desired_qd", 1, true));
    des_qd_pub_->msg_.data.resize(6);
    des_qdd_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "desired_qdd", 1, true));
    des_qdd_pub_->msg_.data.resize(6);
    des_i_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "desired_i", 1, true));
    des_i_pub_->msg_.data.resize(6);
    acc_x_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "accel_x", 1, true));
    acc_x_pub_->msg_.data.resize(6);
    acc_y_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "accel_y", 1, true));
    acc_y_pub_->msg_.data.resize(6);
    acc_z_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "accel_z", 1, true));
    acc_z_pub_->msg_.data.resize(6);
    moment_des_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "moment_desred", 1, true));
    moment_des_pub_->msg_.data.resize(6);
    tcp_speed_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "tcp_speed", 1, true));
    tcp_speed_pub_->msg_.data.resize(6);
    tcp_force_pub_.reset(new RealtimePublisher<geometry_msgs::WrenchStamped>(
                            n, "tcp_force", 1, true));
    tcp_force_tmp.resize(6);
    tcp_force_pub_->msg_.header.frame_id = "/base_link";
    tcp_wrench_pub_.reset(new RealtimePublisher<geometry_msgs::WrenchStamped>(
                            n, "tcp_wrench", 1, true));
    tcp_wrench_tmp.resize(6);
    tcp_wrench_pub_->msg_.header.frame_id = "/base_link";
    tcp_pose_pub_.reset(new RealtimePublisher<std_msgs::Float64MultiArray>(
                            n, "tcp_pose", 1, true));
    tcp_pose_pub_->msg_.data.resize(6);

    force_scalar_pub_.reset(new RealtimePublisher<std_msgs::Float64>(
                          n, "tcp_force_scalar", 1, true));
    tcp_power_pub_.reset(new RealtimePublisher<std_msgs::Float64>(
                          n, "tcp_power", 1, true));
    tcp_payload_pub_.reset(new RealtimePublisher<std_msgs::Float64>(
                          n, "tcp_payload", 1, true));
    power_pub_.reset(new RealtimePublisher<std_msgs::Float64>(
                          n, "power", 1, true));

    mode_id_rt_pub_.reset(new RealtimePublisher<std_msgs::Int32>(
                          n, "robot_mode_id", 1, true));

    jnt_modes_rt_pub_.reset(new RealtimePublisher<std_msgs::Int32MultiArray>(
                            n, "joint_mode_ids", 1, true));
    jnt_modes_rt_pub_->msg_.data.resize(6);

    bool_rt_pubs_.resize(6);
    bool_rt_pubs_[0].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_power_on_robot", 1, true));
    bool_rt_pubs_[1].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_security_stopped", 1, true));
    bool_rt_pubs_[2].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_emergency_stopped", 1, true));
    bool_rt_pubs_[3].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_extra_button_pressed", 1, true));
    bool_rt_pubs_[4].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_power_button_pressed", 1, true));
    bool_rt_pubs_[5].reset(new RealtimePublisher<std_msgs::Bool>(
                           n, "is_safety_signal_such_that_we_should_stop", 1, true));
    cur_bool_states_.resize(bool_rt_pubs_.size());

    cur_joint_ids_.resize(6);

    return true;
  }

  void starting(const ros::Time& time) 
  { 
    first_update_ = true;
    last_pub_time_ = ros::Time();
  }

  void update(const ros::Time& time, const ros::Duration& period) 
  {
    if((time - last_pub_time_).toSec() < pub_period_)
      return;
    else
      last_pub_time_ = time;

    cur_bool_states_[0] = config_hdl_.isPowerOnRobot();
    cur_bool_states_[1] = config_hdl_.isSecurityStopped();
    cur_bool_states_[2] = config_hdl_.isEmergencyStopped();
    cur_bool_states_[3] = config_hdl_.isExtraButtonPressed();
    cur_bool_states_[4] = config_hdl_.isPowerButtonPressed();
    cur_bool_states_[5] = config_hdl_.isSafetySignalSuchThatWeShouldStop();

    // try to publish
    if(mode_id_rt_pub_->trylock()) {
      if(mode_id_rt_pub_->msg_.data != config_hdl_.getRobotModeID() || first_update_) {
        mode_id_rt_pub_->msg_.data = config_hdl_.getRobotModeID();
        mode_id_rt_pub_->unlockAndPublish();
      }
      else mode_id_rt_pub_->unlock();
    }

    if(act_q_pub_->trylock()) {
      config_hdl_.getActualQ(act_q_pub_->msg_.data);
      act_q_pub_->unlockAndPublish();
    }
    if(act_qd_pub_->trylock()) {
      config_hdl_.getActualQD(act_qd_pub_->msg_.data);
      act_qd_pub_->unlockAndPublish();
    }
    if(act_i_pub_->trylock()) {
      config_hdl_.getActualI(act_i_pub_->msg_.data);
      act_i_pub_->unlockAndPublish();
    }
    if(des_q_pub_->trylock()) {
      config_hdl_.getDesiredQ(des_q_pub_->msg_.data);
      des_q_pub_->unlockAndPublish();
    }
    if(des_qd_pub_->trylock()) {
      config_hdl_.getDesiredQD(des_qd_pub_->msg_.data);
      des_qd_pub_->unlockAndPublish();
    }
    if(des_qdd_pub_->trylock()) {
      config_hdl_.getDesiredQDD(des_qdd_pub_->msg_.data);
      des_qdd_pub_->unlockAndPublish();
    }
    if(des_i_pub_->trylock()) {
      config_hdl_.getDesiredI(des_i_pub_->msg_.data);
      des_i_pub_->unlockAndPublish();
    }
    if(acc_x_pub_->trylock()) {
      config_hdl_.getAccelX(acc_x_pub_->msg_.data);
      acc_x_pub_->unlockAndPublish();
    }
    if(acc_y_pub_->trylock()) {
      config_hdl_.getAccelY(acc_y_pub_->msg_.data);
      acc_y_pub_->unlockAndPublish();
    }
    if(acc_z_pub_->trylock()) {
      config_hdl_.getAccelZ(acc_z_pub_->msg_.data);
      acc_z_pub_->unlockAndPublish();
    }
    if(moment_des_pub_->trylock()) {
      config_hdl_.getMomentDes(moment_des_pub_->msg_.data);
      moment_des_pub_->unlockAndPublish();
    }
    if(tcp_speed_pub_->trylock()) {
      config_hdl_.getTCPSpeed(tcp_speed_pub_->msg_.data);
      tcp_speed_pub_->unlockAndPublish();
    }
    if(tcp_force_pub_->trylock()) {
      config_hdl_.getTCPForce(tcp_force_tmp);
      tcp_force_pub_->msg_.wrench.force.x = -tcp_force_tmp[0];
      tcp_force_pub_->msg_.wrench.force.y = -tcp_force_tmp[1];
      tcp_force_pub_->msg_.wrench.force.z = tcp_force_tmp[2];
      tcp_force_pub_->msg_.wrench.torque.x = -tcp_force_tmp[3];
      tcp_force_pub_->msg_.wrench.torque.y = -tcp_force_tmp[4];
      tcp_force_pub_->msg_.wrench.torque.z = tcp_force_tmp[5];
      tcp_force_pub_->msg_.header.stamp = time;
      tcp_force_pub_->unlockAndPublish();
    }
    if(tcp_wrench_pub_->trylock()) {
      config_hdl_.getTCPWrench(tcp_wrench_tmp);
      tcp_wrench_pub_->msg_.wrench.force.x = -tcp_wrench_tmp[0];
      tcp_wrench_pub_->msg_.wrench.force.y = -tcp_wrench_tmp[1];
      tcp_wrench_pub_->msg_.wrench.force.z = tcp_wrench_tmp[2];
      tcp_wrench_pub_->msg_.wrench.torque.x = -tcp_wrench_tmp[3];
      tcp_wrench_pub_->msg_.wrench.torque.y = -tcp_wrench_tmp[4];
      tcp_wrench_pub_->msg_.wrench.torque.z = tcp_wrench_tmp[5];
      tcp_wrench_pub_->msg_.header.stamp = time;
      tcp_wrench_pub_->unlockAndPublish();
    }
    if(tcp_pose_pub_->trylock()) {
      config_hdl_.getTCPPose(tcp_pose_pub_->msg_.data);
      tcp_pose_pub_->unlockAndPublish();
    }

    if(tcp_payload_pub_->trylock()) {
      tcp_payload_pub_->msg_.data = config_hdl_.getTCPPayload();
      tcp_payload_pub_->unlockAndPublish();
    }
    if(tcp_power_pub_->trylock()) {
      tcp_power_pub_->msg_.data = config_hdl_.getTCPPower();
      tcp_power_pub_->unlockAndPublish();
    }
    if(force_scalar_pub_->trylock()) {
      force_scalar_pub_->msg_.data = config_hdl_.getTCPForceScalar();
      force_scalar_pub_->unlockAndPublish();
    }
    if(power_pub_->trylock()) {
      power_pub_->msg_.data = config_hdl_.getPower();
      power_pub_->unlockAndPublish();
    }

    if(jnt_modes_rt_pub_->trylock()) {
      config_hdl_.getJointModeIDs(cur_joint_ids_);
      bool joint_ids_changed = false;
      for(int i=0;i<6;i++) {
        if(jnt_modes_rt_pub_->msg_.data[i] != cur_joint_ids_[i]) {
          joint_ids_changed = true;
          break;
        }
      }
      if(joint_ids_changed || first_update_) {
        std::copy(cur_joint_ids_.begin(), cur_joint_ids_.end(), jnt_modes_rt_pub_->msg_.data.begin());
        jnt_modes_rt_pub_->unlockAndPublish();
      }
      else jnt_modes_rt_pub_->unlock();
    }

    for(size_t i=0;i<cur_bool_states_.size();i++) {
      if(bool_rt_pubs_[i]->trylock()) {
        if(bool_rt_pubs_[i]->msg_.data != cur_bool_states_[i] || first_update_) {
          bool_rt_pubs_[i]->msg_.data = cur_bool_states_[i];
          bool_rt_pubs_[i]->unlockAndPublish();
        }
        else bool_rt_pubs_[i]->unlock();
      }
    }
    first_update_ = false;
  }

  void stopping(const ros::Time& time) 
  {
  }

private:
  ur::URConfigHandle config_hdl_;

  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > act_q_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > act_qd_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > act_i_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > des_q_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > des_qd_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > des_qdd_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > des_i_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > acc_x_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > acc_y_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > acc_z_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > moment_des_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > tcp_speed_pub_;
  boost::shared_ptr<RealtimePublisher<geometry_msgs::WrenchStamped> > tcp_force_pub_;
  boost::shared_ptr<RealtimePublisher<geometry_msgs::WrenchStamped> > tcp_wrench_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64MultiArray> > tcp_pose_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64> > force_scalar_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64> > tcp_power_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64> > tcp_payload_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Float64> > power_pub_;

  boost::shared_ptr<RealtimePublisher<std_msgs::Int32> > mode_id_rt_pub_;
  boost::shared_ptr<RealtimePublisher<std_msgs::Int32MultiArray> > jnt_modes_rt_pub_;
  std::vector<boost::shared_ptr<RealtimePublisher<std_msgs::Bool> > > bool_rt_pubs_;
  bool first_update_;
  ros::Time last_pub_time_;
  double pub_period_;
  std::vector<bool> cur_bool_states_;
  std::vector<int> cur_joint_ids_;

  std::vector<double> tcp_force_tmp;
  std::vector<double> tcp_wrench_tmp;
};

const char* JOINT_NAMES[6] = { 
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint"};

const char* ROBOT_MODES[11] = {
  "RUNNING",
  "FREEDRIVE",
  "READY",
  "INITIALIZING",
  "SECURITY_STOPPED",
  "EMERGENCY_STOPPED",
  "FATAL_ERROR",
  "NO_POWER",
  "NOT_CONNECTED",
  "SHUTDOWN",
  "SAFEGUARD_STOP"};

const char* JOINT_MODES[19] = {
  "PART_D_CALIBRATION",
  "BACKDRIVE",
  "POWER_OFF",
  "EMERGENCY_STOPPED",
  "CALVAL_INITIALIZATION",
  "ERROR",
  "FREEDRIVE",
  "SIMULATED 244",
  "NOT_RESPONDING",
  "MOTOR_INITIALISATION",
  "BOOTING",
  "PART_D_CALIBRATION_ERROR",
  "BOOTLOADER",
  "CALIBRATION",
  "SECURITY_STOPPED",
  "FAULT",
  "RUNNING",
  "INITIALISATION",
  "IDLE"};

class DiagnosticPublishController: 
  public controller_interface::Controller<ur::URConfigInterface>
{
public:
  DiagnosticPublishController() {}

  bool init(ur::URConfigInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    config_hdl_ = hw->getHandle("config_command");

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    diag_pub_.reset(new RealtimePublisher<diagnostic_msgs::DiagnosticArray>(
                     root_nh, "/diagnostics", 1));
    diag_pub_->msg_.status.resize(7);

    diag_pub_->msg_.status[0].name = "ur_arm: UR arm controller";
    diag_pub_->msg_.status[0].values.resize(8);
    diag_pub_->msg_.status[0].values[0].key = "Robot mode";
    diag_pub_->msg_.status[0].values[1].key = "Robot mode ID";
    diag_pub_->msg_.status[0].values[2].key = "Power on robot";
    diag_pub_->msg_.status[0].values[3].key = "Security stopped";
    diag_pub_->msg_.status[0].values[4].key = "Emergency stopped";
    diag_pub_->msg_.status[0].values[5].key = "Extra button pressed";
    diag_pub_->msg_.status[0].values[6].key = "Power button pressed";
    diag_pub_->msg_.status[0].values[7].key = "Safety signal such that we should stop";

    for(int i=0;i<6;i++) {
      diag_pub_->msg_.status[i+1].name = "ur_arm/" + std::string(JOINT_NAMES[i]) + ": UR arm joint";
      diag_pub_->msg_.status[i+1].values.resize(2);
      diag_pub_->msg_.status[i+1].values[0].key = "Joint mode";
      diag_pub_->msg_.status[i+1].values[1].key = "Joint mode ID";
    }
    
    cur_bool_states_.resize(6);
    cur_joint_ids_.resize(6);
    last_bool_states_.resize(6);
    last_joint_ids_.resize(6);

    return true;
  }

  void starting(const ros::Time& time) 
  { 
    if(publish_rate_ > 0.0)
      last_publish_time_ = time - ros::Duration(2.0/publish_rate_);
  }

  // only update at a rate of publish_rate_, or when the topics change
  void update(const ros::Time& time, const ros::Duration& period) 
  {
    bool update_diag = false;
    // limit rate of publishing
    if(publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
      update_diag = true;
    }

    cur_robot_mode_ = config_hdl_.getRobotModeID();
    cur_bool_states_[0] = config_hdl_.isPowerOnRobot();
    cur_bool_states_[1] = config_hdl_.isSecurityStopped();
    cur_bool_states_[2] = config_hdl_.isEmergencyStopped();
    cur_bool_states_[3] = config_hdl_.isExtraButtonPressed();
    cur_bool_states_[4] = config_hdl_.isPowerButtonPressed();
    cur_bool_states_[5] = config_hdl_.isSafetySignalSuchThatWeShouldStop();
    config_hdl_.getJointModeIDs(cur_joint_ids_);

    update_diag = update_diag || cur_robot_mode_ != last_robot_mode_;
    for(size_t i=0;i<cur_bool_states_.size();i++) 
      update_diag = update_diag || cur_bool_states_[i] != last_bool_states_[i];
    for(int i=0;i<6;i++)
      update_diag = update_diag || cur_joint_ids_[i] != last_joint_ids_[i];

    if(!update_diag) return;

    last_robot_mode_ = cur_robot_mode_;
    std::copy(cur_bool_states_.begin(), cur_bool_states_.end(), last_bool_states_.begin());
    std::copy(cur_joint_ids_.begin(), cur_joint_ids_.end(), last_joint_ids_.begin());

    if(diag_pub_->trylock()) {
      diag_pub_->msg_.header.stamp = time;

      // Robot modes
      if(cur_robot_mode_ >= 0 && cur_robot_mode_ < 2)
        diag_pub_->msg_.status[0].level = diag_pub_->msg_.status[0].OK;
      else if(cur_robot_mode_ >= 2 && cur_robot_mode_ < 4)
        diag_pub_->msg_.status[0].level = diag_pub_->msg_.status[0].WARN;
      else 
        diag_pub_->msg_.status[0].level = diag_pub_->msg_.status[0].ERROR;
      if(cur_robot_mode_ >= 0 && cur_robot_mode_ < 11)
        diag_pub_->msg_.status[0].message = ROBOT_MODES[cur_robot_mode_];
      else
        diag_pub_->msg_.status[0].message = "UNINITIALIZED";

      diag_pub_->msg_.status[0].values[0].value = diag_pub_->msg_.status[0].message;
      diag_pub_->msg_.status[0].values[1].value = lexical_cast<std::string>(cur_robot_mode_);
      for(size_t i=0;i<cur_bool_states_.size();i++) 
        diag_pub_->msg_.status[0].values[i+2].value = lexical_cast<std::string>(cur_bool_states_[i]);
      
      // Joint modes
      for(int i=0;i<6;i++) {
        if(cur_joint_ids_[i] == 16)
          diag_pub_->msg_.status[i+1].level = diag_pub_->msg_.status[i+1].OK;
        else if(cur_joint_ids_[i] == 13 ||cur_joint_ids_[i] == 17 || cur_joint_ids_[i] == 18)
          diag_pub_->msg_.status[i+1].level = diag_pub_->msg_.status[i+1].WARN;
        else 
          diag_pub_->msg_.status[i+1].level = diag_pub_->msg_.status[i+1].ERROR;
        if(cur_joint_ids_[i] >= 0 && cur_joint_ids_[i] < 19)
          diag_pub_->msg_.status[i+1].message = JOINT_MODES[cur_joint_ids_[i]];
        else
          diag_pub_->msg_.status[i+1].message = "UNINITIALIZED";
        diag_pub_->msg_.status[i+1].values[0].value = diag_pub_->msg_.status[i+1].message;
        diag_pub_->msg_.status[i+1].values[1].value = lexical_cast<std::string>(cur_joint_ids_[i]);
      }

      diag_pub_->unlockAndPublish();
    }
  }

  void stopping(const ros::Time& time) 
  {
  }

private:

  ur::URConfigHandle config_hdl_;
  boost::shared_ptr<RealtimePublisher<diagnostic_msgs::DiagnosticArray> > diag_pub_;

  int cur_robot_mode_;
  std::vector<bool> cur_bool_states_;
  std::vector<int> cur_joint_ids_;

  int last_robot_mode_;
  std::vector<bool> last_bool_states_;
  std::vector<int> last_joint_ids_;

  ros::Time last_publish_time_;
  double publish_rate_;
};



}

#endif
