#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from recon_msgs.msg import SLZCoordinates
from sklearn.cluster import KMeans
import numpy as np

def mycallback(msg):
    rospy.loginfo("Gothere")
    
    markerArray = MarkerArray()
    rospy.loginfo(len(msg.CoordinateData[0].x))

    data = np.zeros((3,len(msg.CoordinateData[0].x)))
    
    pos = msg.CoordinateData[0]
    for i in range(len(msg.CoordinateData[0].x)):
        data[0][i] = pos.x[i]
        data[1][i] = pos.y[i]
        data[2][i] = pos.z[i]       

    kmeans = KMeans(n_clusters=2, random_state=0).fit(data)
    
    cluster1 = np.where(kmeans.labels_ == 0)
    cluster2 = np.where(kmeans.labels_ == 1)

    for i in data[cluster1]:
        marker = Marker()
        marker.header.frame_id = "iris_sensors_0/camera_red_iris_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = i[0]
        marker.pose.position.y = i[1]
        marker.pose.position.z = i[2]
        markerArray.markers.append(marker)
    
    for i in data[cluster2]:
        marker = Marker()
        marker.header.frame_id = "iris_sensors_0/camera_red_iris_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = i[0]
        marker.pose.position.y = i[1]
        marker.pose.position.z = i[2]
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