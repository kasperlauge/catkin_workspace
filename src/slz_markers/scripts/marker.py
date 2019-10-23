#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from recon_msgs.msg import SLZCoordinates


def mycallback(msg):
    rospy.loginfo("Gothere")
    
    markerArray = MarkerArray()


    for i in range(len(msg.CoordinateData[0].x)):
        pos = msg.CoordinateData[0]
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = pos.x[i]
        marker.pose.position.y = pos.y[i]
        marker.pose.position.z = pos.z[i]
        markerArray.markers.append(marker)
    
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    pub.publish(markerArray)

if __name__ == '__main__':
    try:
        rospy.init_node('SLZ_rviz_markernode')
        pub = rospy.Publisher("mymarker",MarkerArray,queue_size=10)

        rospy.loginfo("Marker starting")

        rate = rospy.Rate(10)

        topic_listener = rospy.Subscriber("slz_coordinates",SLZCoordinates,mycallback)

        rospy.spin()

        # while not rospy.is_shutdown():
        #     marker = Marker()
        #     marker.header.frame_id = "/base_link"
        #     marker.type = marker.SPHERE
        #     marker.action = marker.ADD
        #     marker.scale.x = 0.2
        #     marker.scale.y = 0.2
        #     marker.scale.z = 0.2
        #     marker.color.a = 1.0
        #     marker.color.r = 1.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.pose.position.x = 1.
        #     marker.pose.position.y = 1. 
        #     marker.pose.position.z = 4. 

        #     pub.publish(marker)

        #     rate.sleep()

    except rospy.ROSInterruptException:
        pass