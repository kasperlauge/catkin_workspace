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

    rospy.loginfo(data.shape)
    
    nr_clusters = 7

    kmeans = KMeans(n_clusters=nr_clusters, random_state=0).fit(np.transpose(data))
    
    clusters = []
    for i in range(nr_clusters):
        clusters.append(np.where(kmeans.labels_ == i)[0])

    color = [0.1, 0.2,0.3,0.4,0.5,0.6,0.7,0.8]
    l = 0

    threshold = .1

    remove = []

    for c in clusters:
        a = np.max(data[2][c[:]])-np.min(data[2][c[:]])
        rospy.loginfo(a)
        if a > threshold:
            remove.append(c)

    for c in remove:
        clusters.remove(c)

    for c in clusters:
        for i in range(c.size):
            marker = Marker()
            marker.header.frame_id = "iris_sensors_0/camera_red_iris_link"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = color[l]
            marker.color.g = color[l]
            marker.color.b = color[l]
            marker.pose.position.x = data[0][c[i]]
            marker.pose.position.y = data[1][c[i]]
            marker.pose.position.z = data[2][c[i]]
            markerArray.markers.append(marker)
        l += 1
    
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

    except rospy.ROSInterruptException:
        pass