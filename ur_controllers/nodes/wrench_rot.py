#!/usr/bin/env python

import copy
import rospy
from tf import TransformListener
from geometry_msgs.msg import Vector3Stamped, WrenchStamped

class WrenchRot(object):
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "/ee_link")

        self.tf_list = TransformListener()
        self.ws_sub = rospy.Subscriber("old_wrench", WrenchStamped, self.trans_wrench_cb)
        self.ws_pub = rospy.Publisher("new_wrench", WrenchStamped)

    def trans_wrench_cb(self, ws_old):
        ws_new = WrenchStamped()
        ws_new.header = copy.deepcopy(ws_old.header)
        ws_new.header.frame_id = self.target_frame
        vec3_stamped = Vector3Stamped()
        vec3_stamped.header = ws_old.header

        try:
            self.tf_list.waitForTransform(self.target_frame, ws_old.header.frame_id, 
                                          ws_old.header.stamp, rospy.Duration(1.))
        except:
            rospy.logwarn("Timeout waiting for transform from %s to target frame %s" 
                          % (ws_old.header.frame_id, self.target_frame))
            return
        vec3_stamped.vector = ws_old.wrench.force
        ws_new.wrench.force = self.tf_list.transformVector3(self.target_frame, vec3_stamped).vector
        vec3_stamped.vector = ws_old.wrench.torque
        ws_new.wrench.torque = self.tf_list.transformVector3(self.target_frame, vec3_stamped).vector

        self.ws_pub.publish(ws_new)

def main():
    rospy.init_node("wrench_rot")
    wr = WrenchRot()
    rospy.spin()

if __name__ == "__main__":
    main()
