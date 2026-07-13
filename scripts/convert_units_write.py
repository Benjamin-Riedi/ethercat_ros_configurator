import rospy
import numpy as np

from control_utils.msg import VectorStamped, ScalarStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage, MotorStatusMessage

class ConvertUnitsWriteNode:
    def __init__(self):
        rospy.init_node(name='convert_units_write', anonymous=True)
        self.read_params()
        self.init_publishers()
        self.init_variables()

    def read_params(self):
        self.bottom_motor_topic = rospy.get_param('/topic/Maxon_Motor_bottom/command', '/ethercat_master/Maxon_Motor_bottom/command')
        self.top_motor_topic = rospy.get_param('/topic/Maxon_Motor_top/command', '/ethercat_master/Maxon_Motor_top/command')

        self.setpoint_bottom_topic = rospy.get_param('/topics/control/command/bottom', '/bottom/v_sp')
        self.setpoint_top_topic = rospy.get_param('/topics/control/command/top', '/top/v_sp')

    def init_publishers(self):
        self.pub_setpoint_bottom = rospy.Publisher(self.bottom_motor_topic, MotorCtrlMessage, queue_size=1)
        self.pub_setpoint_top = rospy.Publisher(self.top_motor_topic, MotorCtrlMessage, queue_size=1)

        self.setpoint_bottom_msg = MotorCtrlMessage(operationMode=MotorCtrlMessage.MAXON_EPOS4_OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY)
        self.setpoint_top_msg = MotorCtrlMessage(operationMode=MotorCtrlMessage.MAXON_EPOS4_OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY)

    def init_variables(self):
        self.v_sp_bottom = 0.0
        self.v_sp_top = 0.0

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


    def callback_bottom(self, msg):
        vel_mps = msg.scalar
        self.time = msg.header.stamp

        self.v_sp_bottom = self.mps_to_rpm(vel_mps)
        self.publish_bottom()
    
    def callback_top(self, msg):
        vel_mps = msg.scalar
        self.time = msg.header.stamp

        self.v_sp_top = self.mps_to_rpm(vel_mps)
        self.publish_top()

    def publish_bottom(self):
        self.setpoint_bottom_msg.header.stamp = self.time
        self.setpoint_bottom_msg.targetVelocity = self.v_sp_bottom

        self.pub_setpoint_bottom.publish(self.setpoint_bottom_msg)

    def publish_top(self):
        self.setpoint_top_msg.header.stamp = self.time
        self.setpoint_top_msg.targetVelocity = self.v_sp_top

        self.pub_setpoint_top.publish(self.setpoint_top_msg)

    def run(self):
        rospy.Subscriber(self.setpoint_bottom_topic, ScalarStamped , self.callback_bottom, queue_size=1)
        rospy.Subscriber(self.setpoint_top_topic, ScalarStamped, self.callback_top, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsWriteNode()
    node.run()