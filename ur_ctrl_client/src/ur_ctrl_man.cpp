
#include <ros/ros.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>
#include <ur_ctrl_client/ur_robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace ur;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_controller_man");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  std::string robot_ip;
  if(!nh_priv.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Missing robot IP address (robot_ip)");
    return -1;
  }
  XmlRpc::XmlRpcValue v;
  if(!nh_priv.getParam("joint_names", v) || v.size() != 6) {
    ROS_ERROR("URRobotHW requires a list of the 6 joint names");
    return -1;
  }
  std::vector<std::string> joint_names;
  for(int i=0;i<6;i++)
    joint_names.push_back(v[i]);

  double ctrl_loop_rate = 125.0;
  nh_priv.getParam("ctrl_loop_rate", ctrl_loop_rate);

  urdf::Model urdf_model;
  if(!urdf_model.initParam("robot_description")) {
    ROS_ERROR("ur_ctrl_man requires a URDF in the robot_description parameter.");
    return -1;
  }

  joint_limits_interface::JointLimits ur_limits[6];
  joint_limits_interface::SoftJointLimits ur_soft_limits[6];
  for(int i=0;i<6;i++) {
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(joint_names[i]);
    bool urdf_found_limits = getJointLimits(urdf_joint, ur_limits[i]);
    bool param_srv_found_limits = getJointLimits(joint_names[i], nh_priv, ur_limits[i]);
    if(!urdf_found_limits && !param_srv_found_limits) {
      ROS_ERROR("Couldn't find limits for joint %s", joint_names[i].c_str());
      return -1;
    }
    ur_soft_limits[i].min_position = ur_limits[i].min_position;
    ur_soft_limits[i].max_position = ur_limits[i].max_position;
    ur_soft_limits[i].k_position = 10.0;
    getSoftJointLimits(joint_names[i], nh_priv, ur_soft_limits[i]);
  }

  URRobotHW ur_hw(nh, joint_names, ur_limits, ur_soft_limits);
  ur_hw.init(robot_ip);

  controller_manager::ControllerManager cm(&ur_hw, nh);

  ros::Duration period(1.0/ctrl_loop_rate);
  ros::Time now;
  while (ros::ok()) {
    now = ros::Time::now();
    ur_hw.read(now, period);
    cm.update(now, period);
    ur_hw.write(now, period);
    period.sleep();
  }
}
