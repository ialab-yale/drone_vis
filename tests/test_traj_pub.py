#!/usr/bin/env python3

import rospy
from drone_vis.msg import Trajectory
import numpy as np

from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Point
import tf

import numpy as np

if __name__ == "__main__":
    rospy.init_node("traj_pub_node")
    agent_name="dude"

    br = tf.TransformBroadcaster()
    traj_pub = rospy.Publisher(agent_name + '/planned_traj', Trajectory, queue_size=10)


    traj_msg = Trajectory()
    traj_msg.name= agent_name + "_traj"

    x = np.random.uniform(-1,1,size=(3,))
    xf = np.random.uniform(-1,1,size=(3,))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # x = np.random.uniform(-1,1,size=(3,))
        # xf = np.random.uniform(-1,1,size=(3,))

        traj = np.linspace(x, xf)

        del traj_msg.points
        traj_msg.points = []
        for _tr in traj:
            pt = Point(_tr[0], _tr[1], _tr[2])
            traj_msg.points.append(pt)

        traj_pub.publish(traj_msg)

        br.sendTransform(
                (x[0], x[1], x[2]),
                (0.,0.,0.,1.),
                rospy.Time.now(),
                agent_name,
                "world"
            )
        rate.sleep()