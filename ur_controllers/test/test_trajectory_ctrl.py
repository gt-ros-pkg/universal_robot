#!/usr/bin/python

import copy
import sys

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray
import rospy
import actionlib

from ur_py_utils.arm_iface import ArmInterface
from ur_py_utils.ur_controller_manager import URControllerManager

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ['vel', 'pva']:
        print "test_trajectory_ctrl.py [joint mode = vel/pva]"
        return
    mode = sys.argv[1]
    rospy.init_node("test_traj_ctrl")
    cman = URControllerManager()
    cman.start_joint_controller('%s_trajectory_ctrl' % mode)
    act_cli = actionlib.SimpleActionClient(
            '/%s_trajectory_ctrl/follow_joint_trajectory' % mode, 
            FollowJointTrajectoryAction)
    if not act_cli.wait_for_server(rospy.Duration.from_sec(1.)):
        rospy.logerr("Can't find action server")
        return
    arm = ArmInterface()
    q_cur = arm.get_q().tolist()
    q_cur[2] += 0.2
    q_cur[5] += 0.2
    jtp1 = JointTrajectoryPoint()
    jtp1.positions = q_cur
    jtp1.velocities = 6*[0.0]
    jtp1.velocities[2] = 0.1
    jtp1.velocities[5] = 0.1
    jtp1.accelerations = 6*[0.0]
    jtp1.time_from_start = rospy.Duration.from_sec(5.)
    jtp2 = copy.deepcopy(jtp1)
    jtp2.positions[2] -= 0.2
    jtp2.positions[5] -= 0.2
    jtp2.velocities = 6*[0.0]
    jtp2.time_from_start = rospy.Duration.from_sec(10.)
    fjt = FollowJointTrajectoryGoal()
    fjt.trajectory.header.stamp = rospy.Time.now()
    fjt.trajectory.joint_names = arm.JOINT_NAMES
    fjt.trajectory.points = [jtp1, jtp2]

    act_cli.send_goal(fjt)
    rospy.loginfo("Starting trajectory using %s mode" % mode)
    act_cli.wait_for_result()
    rospy.loginfo("Trajectory complete")

if __name__ == "__main__":
    main()
