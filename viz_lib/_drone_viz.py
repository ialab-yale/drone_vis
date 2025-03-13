import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from drone_env_viz.msg import Trajectory
from geometry_msgs.msg import Pose, Pose, Point
from tf import transformations as trans
import tf

from .vis_helpers import getHeader

class DroneViz(object):

    def __init__(self, agent_name, scale=0.1):

        rospy.init_node(agent_name + '_vis')
        self._agent_name = agent_name

        self._scale = scale
        # instantiatiate the publishers
        self._model_pub     = rospy.Publisher(agent_name + '/vis' + '/model', Marker, queue_size=10)
        self._traj_pub = rospy.Publisher(agent_name + '/vis' + '/traj', Marker, queue_size=10)

        self._model_msg = Marker()
        self._model_msg.ns = agent_name
        self._model_msg.id = 0

        self._traj_msg = Marker()
        self._traj_msg.ns = agent_name
        self._traj_msg.id = 1
        self._traj_sub = rospy.Subscriber(agent_name + '/planned_traj', Trajectory, self.callback_trajectory)


        self._tf_listener = tf.TransformListener()

        self.__build_rendering()

    def callback_trajectory(self, msg):
        # msg is of type Trajectory (see msg folder)
        # convert the points in the msg to a marker
        self._traj_msg.header = getHeader("world", rospy.Time(0))
        del self._traj_msg.points
        self._traj_msg.points = []
        for pt in msg.points:
            self._traj_msg.points.append(pt)        
        self._traj_pub.publish(self._traj_msg)

    def update_model_pose(self):
            self._model_msg.header = getHeader("world", rospy.Time(0))
            (trans, rot) = self._tf_listener.lookupTransform(
                "world", self._agent_name, rospy.Time(0)
            )
            self._model_msg.pose.position.x = trans[0]
            self._model_msg.pose.position.y = trans[1]
            self._model_msg.pose.position.z = trans[2]
            self._model_msg.pose.orientation.x = rot[0]
            self._model_msg.pose.orientation.y = rot[1]
            self._model_msg.pose.orientation.z = rot[2]
            self._model_msg.pose.orientation.w = rot[3]
            self._model_pub.publish(self._model_msg)

    def run(self):
        while not rospy.is_shutdown():
            try: 
                self.update_model_pose()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def __build_rendering(self):

        rgb = np.random.uniform(0,1, size=(3,))

        scale = 0.1
        self._model_msg.header.frame_id = "world"
        self._model_msg.header.stamp = rospy.Time(0)
        self._model_msg.ns = self._agent_name
        self._model_msg.id = 0
        self._model_msg.action = Marker.ADD
        self._model_msg.scale.x = self._scale
        self._model_msg.scale.y = self._scale
        self._model_msg.scale.z = self._scale
        self._model_msg.color.a = 1.0
        self._model_msg.color.r = rgb[0]
        self._model_msg.color.g = rgb[1]
        self._model_msg.color.b = rgb[2]

        self._model_msg.type = Marker.MESH_RESOURCE
        self._model_msg.mesh_resource = "package://drone_vis/mesh/quad_base.stl"

        self._traj_msg.header.frame_id = "world"
        self._traj_msg.header.stamp = rospy.Time(0)
        self._traj_msg.ns = self._agent_name
        self._traj_msg.id = 1
        self._traj_msg.action = Marker.ADD
        self._traj_msg.points = []
        self._traj_msg.scale.x = 0.01
        self._traj_msg.color.a = 1.0
        self._traj_msg.color.r = rgb[0]
        self._traj_msg.color.g = rgb[1]
        self._traj_msg.color.b = rgb[2]
        self._traj_msg.type = Marker.LINE_STRIP



