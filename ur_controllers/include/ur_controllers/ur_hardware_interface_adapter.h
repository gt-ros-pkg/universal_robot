
#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <ur_ctrl_client/pos_vel_acc_joint_iface.h>
#include <trajectory_interface/pos_vel_acc_state.h>

/**
 * \brief Adapter for an velocity-controlled hardware interface. Maps position and velocity errors to velocity commands
 * through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "velocity_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint:
 *       trajectory: 0.05
 *       goal: 0.02
 *     head_2_joint:
 *       goal: 0.01
 *
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 * \endcode
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    return true;
  }

  void starting(const ros::Time& time)
  {
    if (!joint_handles_ptr_) {return;}

    // Reset PIDs, zero velocity commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& time) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         /*desired_state*/,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_) {return;}
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double eff_command = pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
      (*joint_handles_ptr_)[i].setCommand(eff_command*period.toSec());
    }
  }

private:
  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

/**
 * \brief Adapter for a position/velocity/acceleration-controlled hardware interface. 
 * Forwards desired positions/velocities/accelerations as commands.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelAccJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}

  bool init(std::vector<hardware_interface::PosVelAccJointHandle>& joint_handles, 
                        ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {
      (*joint_handles_ptr_)[i].setPosition(desired_state.position[i]);
      (*joint_handles_ptr_)[i].setVelocity(desired_state.velocity[i]);
      (*joint_handles_ptr_)[i].setAcceleration(desired_state.acceleration[i]);
    }
  }

private:
  std::vector<hardware_interface::PosVelAccJointHandle>* joint_handles_ptr_;
};
