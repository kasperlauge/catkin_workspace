#!/usr/bin/env python
import tf

import tf2_ros
import geometry_msgs.msg
import rospy
import math

if __name__ == '__main__':
    try:

        """
        This node is responsiple for tranforming the placement of the camera to the base link.
        The tranform makes rviz display correct marker information
        """

        rospy.init_node('my_static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.frame_id = "base_link"
        static_transformStamped.child_frame_id = "iris_sensors_0/camera_red_iris_link"
    
        rospy.loginfo("message")

        """
        Transform from zero
        """
        static_transformStamped.transform.translation.x = float(0)
        static_transformStamped.transform.translation.y = float(0)
        static_transformStamped.transform.translation.z = float(0.07)

        quat = tf.transformations.quaternion_from_euler(
            float(-math.pi*0.5), float(0), float(-math.pi*0.5))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        broadcaster.sendTransform(static_transformStamped)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
