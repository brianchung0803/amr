#!/usr/bin/env python

import tf
import rospy
from amr_alignment.align import PatternDetector, AlignManager
from laser_line_extraction.msg import LineSegmentList

tf_br = None
pd = None
anchor_frame = None
laser_frame = None

def segCB(data):
  global pd, tf_br, anchor_frame, laser_frame
  if pd is None:
    return

  pose = pd.detect_pattern(data.line_segments)
  if pose is not None:
    # Publish TF
    tf_br.sendTransform((pose[0], pose[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, pose[2]), 
                rospy.Time.now(),
                anchor_frame, laser_frame)


if __name__ == '__main__':
  rospy.init_node('amr_alignment_node')

  ang1 = rospy.get_param('~pattern_angle1', -0.5) 
  ang2 = rospy.get_param('~pattern_angle2', 1.0) 
  ang3 = rospy.get_param('~pattern_angle3', -0.5) 
  group_dist_tolerance = rospy.get_param('~group_dist_tolerance', 0.15) 
  neighbor_dist_tolerance = rospy.get_param('~neighbor_dist_tolerance', 0.05) 
  detect_angle_tolerance = rospy.get_param('~detect_angle_tolerance', 0.1) 
  anchor_frame = rospy.get_param('~anchor_frame', 'anchor_link') 
  laser_frame = rospy.get_param('~laser_frame', 'laser_link') 
  base_frame = rospy.get_param('~base_frame', 'base_link') 
  minAlignPos_x = rospy.get_param('~min_alignment_position_x', 0.78) 
  minReadyPos_x = rospy.get_param('~min_ready_position_x', 0.75) 
  switchTrajPos_y = rospy.get_param('switch_trajectory_position_y', 0.3) 
  align_tf_server_topic = rospy.get_param('align_tf_server_topic', '/align_tf_server') 
  highRotateVel = rospy.get_param('~high_rotate_vel', 0.5)
  lowRotateVel = rospy.get_param('~low_rotate_vel', 0.1)
  highTransVel = rospy.get_param('~high_trans_vel', 0.25)
  lowTransVel = rospy.get_param('~low_trans_vel', 0.05)
  switchSpAngle = rospy.get_param('~switch_speed_angle', 0.5)
  switchSpDist= rospy.get_param('~switch_speed_dist', 0.3)
  angleTH = rospy.get_param('~angle_tolerance', 0.1)
  transTH = rospy.get_param('~distance_tolerance', 0.05)
  maxMissedCount = rospy.get_param('~max_missed_tolerance', 10)


  rospy.Subscriber('line_segments', LineSegmentList, segCB)
  tf_br = tf.TransformBroadcaster()

  pd = PatternDetector(ang1, ang2, ang3, group_dist_tolerance, neighbor_dist_tolerance, detect_angle_tolerance)
  am = AlignManager(minAlignPos_x, minReadyPos_x, switchTrajPos_y, align_tf_server_topic, \
                    base_frame, highTransVel, lowTransVel, highRotateVel, lowRotateVel, \
                    switchSpAngle, switchSpDist, angleTH, transTH, maxMissedCount)

  
  am.alignment(0.72, anchor_frame)

  rospy.spin()

