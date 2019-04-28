import rospy
import numpy as np
import wam_common


if __name__=='__main__':

    rospy.wait_for_service('/wam/joint_move')
    try:
        join_move = rospy.ServiceProxy('/wam/joint_move', wam_common.JointMove)
        return True
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e