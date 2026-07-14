import rospy
import numpy as np

from control_utils.msg import VectorStamped, ScalarStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage, MotorStatusMessage

class ConvertUnitsWriteNode:
    def __init__(self):
        rospy.init_node(name='convert_units_write', anonymous=True)
        self.init_topics()
        self.init_publishers()
        self.init_variables()
    
    def init_topics(self):
        self.motor_command_topic = 'Maxon_Motor/command'
        self.setpoint_topic = 'v_sp'

    def init_publishers(self):
        self.motor_command_pub = rospy.Publisher(self.motor_command_topic, MotorCtrlMessage, queue_size=1)
        self.command_msg = MotorCtrlMessage(operationMode=MotorCtrlMessage.MAXON_EPOS4_OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY)

    def init_variables(self):
        self.v_sp = 0.0
        self.time = 0

    def m_to_inc(self, m):
        """
        4096 [inc/rev] / 0.005 [m/rev] = 819200 [inc/m]
        """
        return int(m * 819200.0)

    def mps_to_rpm(self, mps):
        """
        0.005 [m/rev] / 60 [s/min] = 1/1200 [m/s / rpm]
        """
        return int(mps * 1200)


    def callback(self, msg):
        vel_mps = msg.scalar
        self.time = msg.header.stamp

        self.v_sp = self.mps_to_rpm(vel_mps)
        self.publish()

    def publish(self):
        self.command_msg.header.stamp = self.time
        self.command_msg.targetVelocity = self.v_sp

        self.motor_command_pub.publish(self.command_msg)

    def run(self):
        rospy.Subscriber(self.setpoint_topic, ScalarStamped, self.callback, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsWriteNode()
    node.run()