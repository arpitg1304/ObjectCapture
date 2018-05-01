import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from Controller import Controller
import numpy as np

class Node(object):
    def __init__(self,desiredPosition):
        self.controller = Controller(desiredPosition)

    def talker(self,torques):
        pub = rospy.Publisher('/torques', JointState, queue_size=1)
        #rospy.init_node('talker', anonymous=True)
        if not rospy.is_shutdown():
            #rospy.loginfo(torques)
            pub.publish(torques)

    def callback(self,data):
        torque, error = self.controller.pdController(data.position,data.velocity)
        tr = JointState()
        tr.effort = torque
        tr.position = error
        self.talker(tr)

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/sensorABB', JointState, self.callback)
        rospy.spin()
