import rospy
import numpy as np

from control_utils.msg import VectorStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage, MotorStatusMessage

class ConvertUnitsReadNode:
    def __init__(self):
        rospy.init_node(name='convert_units_read', anonymous=True)
        self.read_params()
        self.init_publishers()
        self.init_variables()

    def read_params(self):
        self.bottom_motor_topic = rospy.get_param('/topic/Maxon_Motor_bottom/reading', '/ethercat_master/Maxon_Motor_bottom/reading')
        self.top_motor_topic = rospy.get_param('/topic/Maxon_Motor_top/reading', '/ethercat_master/Maxon_Motor_top/reading')

        self.motor_state_bottom_topic = rospy.get_param('/topics/Maxon_Motor_bottom/state', '/Maxon_Motor_bottom/state')
        self.motor_state_top_topic = rospy.get_param('/topics/Maxon_Motor_top/state', '/Maxon_Motor_top/state')

    def init_publishers(self):
        self.pub_state_bottom = rospy.Publisher(self.motor_state_bottom_topic, VectorStamped, queue_size=1)
        self.pub_state_top = rospy.Publisher(self.motor_state_top_topic, VectorStamped, queue_size=1)

        self.state_bottom_msg = VectorStamped()
        self.state_top_msg = VectorStamped()

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.xD = 0.0
        self.yD = 0.0

        self.time = 0

    def inc_to_m(self, inc):
        """
        4096 [inc/rev] / 0.005 [m/rev] = 819200 [inc/m]
        """
        return inc / 819200.0

    def rpm_to_mps(self, rpm):
        """
        0.005 [m/rev] / 60 [s/min] = 1/1200 [m/s / rpm]
        """
        return rpm / 1200


    def callback_bottom(self, msg):
        x_inc = msg.actualPosition
        u_rpm = msg.actualVelocity

        self.x = self.inc_to_m(x_inc)
        self.xD = self.rpm_to_mps(u_rpm)

        self.time = msg.header.stamp
        self.publish_bottom()
    
    def callback_top(self, msg):
        y_inc = msg.actualPosition
        v_rpm = msg.actualVelocity

        self.y = self.inc_to_m(y_inc)
        self.yD = self.rpm_to_mps(v_rpm)

        self.time = msg.header.stamp
        self.publish_top()

    def publish_bottom(self):
        self.state_bottom_msg.header.stamp = self.time
        state = np.array([self.x, self.xD])
        self.state_bottom_msg.vector = state

        self.pub_state_bottom.publish(self.state_bottom_msg)

    def publish_top(self):
        self.state_top_msg.header.stamp = self.time
        state = np.array([self.y, self.yD])
        self.state_top_msg.vector = state

        self.pub_state_top.publish(self.state_top_msg)

    def run(self):
        rospy.Subscriber(self.bottom_motor_topic, MotorStatusMessage, self.callback_bottom, queue_size=1)
        rospy.Subscriber(self.top_motor_topic, MotorStatusMessage, self.callback_top, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsReadNode()
    node.run()