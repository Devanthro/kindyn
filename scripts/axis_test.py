import rospy
from sensor_msgs.msg import JointState
import numpy as np
rospy.init_node("axis_joint_test")
pub = rospy.Publisher("/roboy/oxford/simulation/joint_targets", JointState, queue_size=1)

axis0_limits = [0,1.0]
axis1_limits = [-0.52,0.0]
axis2_limits = [-0.3,0.0]

msg = JointState()
msg.name = ["axis0","axis1", "axis2"]
for i in range(5000):
    if rospy.is_shutdown():
        break
    msg.position = [np.random.uniform(axis0_limits[0], axis0_limits[1], 1),
                    np.random.uniform(axis1_limits[0], axis1_limits[1], 1),
                    np.random.uniform(axis2_limits[0], axis2_limits[1], 1)]
    pub.publish(msg)
    rospy.sleep(0.5)
