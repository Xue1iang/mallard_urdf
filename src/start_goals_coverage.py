#!/usr/bin/env python
import rospy
from visualization_msgs.msg import InteractiveMarkerFeedback

seq_num = 0

rospy.init_node('goals_coverage_once', anonymous=True) 
pub_goals = rospy.Publisher('/areaSelectionMarkerServer/feedback',InteractiveMarkerFeedback,queue_size=1)
# initialize pose with msg type and assign values:
pose_update = InteractiveMarkerFeedback()
pose_update.header.seq = seq_num
pose_update.header.stamp = rospy.Time.now()
pose_update.header.frame_id = "map"

pose_update.client_id = "/mallard/rviz/InteractiveMarkers"
pose_update.marker_name = "selectionMarker"
pose_update.control_name = "move_inPlane_u0"
pose_update.event_type = 2
# marker generic pose
pose_update.pose.position.x = 0.0
pose_update.pose.position.y = 0.0
pose_update.pose.position.z = 0.0
pose_update.pose.orientation.x = 0.0
pose_update.pose.orientation.y = 0.0
pose_update.pose.orientation.z = 0.0
pose_update.pose.orientation.w = 0.0
# menu entry 1 for 'Confirm convergence'; 2 for Cancel:
pose_update.menu_entry_id = 1
# mouse generic pose
pose_update.mouse_point.x = 0.0
pose_update.mouse_point.y = 0.0
pose_update.mouse_point.z = 0.0
pose_update.mouse_point_valid = True

rate = rospy.Rate(1)
finished = False
# seq_num += 1
d = 5
print("Wait for " + str(d) + " seconds")
rospy.sleep(d)

# keep publishing until first success
while not finished:
    connections = pub_goals.get_num_connections()
    if(connections>0):
        pub_goals.publish(pose_update)
        finished = True
        print("Goals published.")
    else:
        rate.sleep()
