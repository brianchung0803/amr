#!/usr/bin/env python

import tf
import rospy
from amr_alignment.srv import RobotTF, RobotTFResponse

listener = None

def robotTFCB(req):
  global listener
  goal_frame = req.goal_frame
  base_frame = req.base_frame

  trans = [0, 0, 0]
  quat = [0, 0, 0, 0]
  while not rospy.is_shutdown():
    now = rospy.Time.now()
    try:
      #(trans, quat) = listener.lookupTransform(base_frame, goal_frame, rospy.Time(0))
      listener.waitForTransform(base_frame, goal_frame, now, rospy.Duration(0.1))
      (trans, quat) = listener.lookupTransform(base_frame, goal_frame, now)
    except Exception as e: 
      #print e
      # Cannot response false, so I make trans[2] = -1 because this cannot happen for amr.
      trans[2] = -1

    return RobotTFResponse(trans, quat)


if __name__ == '__main__':
  rospy.init_node('align_tf_server')
  listener = tf.TransformListener()

  retreat_srv = rospy.Service('/align_tf_server', RobotTF, robotTFCB)
  rospy.spin()

