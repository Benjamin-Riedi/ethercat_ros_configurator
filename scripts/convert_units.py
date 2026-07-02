import rospy
import numpy as np

from control_utils.msg import VectorStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage
from ethercat_motor_msgs.msg import MotorStatusMessage

class ConvertUnitsNode:
    def __init__(self):
        rospy.init_node(name='convert_units', anonymous=True)
        self.read_params()
        self.init_publishers()
        self.init_variables()

    def read_params(self):
        self.bottom_motor_topic = rospy.get_param('/topic/Maxon_Motor_bottom/reading', '/ethercat_master/Maxon_Motor_bottom/reading')
        self.top_motor_topic = rospy.get_param('/topic/Maxon_Motor_top/reading', '/ethercat_master/Maxon_Motor_top/reading')

        self.bottom_state_topic = rospy.get_param('/topics/Maxon_Motor_bottom/state', '/Maxon_Motor_bottom/state')
        self.top_state_topic = rospy.get_param('/topics/Maxon_Motor_top/state', '/Maxon_Motor_top/state')

    def init_publishers(self):
        self.pub_bottom_state = rospy.Publisher(self.bottom_state_topic, VectorStamped, queue_size=1)
        self.pub_top_state = rospy.Publisher(self.top_state_topic, VectorStamped, queue_size=1)

        self.bottom_state_msg = VectorStamped()
        self.top_state_msg = VectorStamped()

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.xD = 0.0
        self.yD = 0.0

        self.time = 0

    def inc_to_mm(self, inc):
        """
        4096 [inc/rev] / 5 [mm/rev] = 819.2 [inc/mm]
        """
        return inc / 819.2

    def rpm_to_mmps(self, rpm):
        """
        5 [mm/rev] / 60 [s/min] = 1/12 [mm/s / rpm]
        """
        return rpm / 12


    def callback_bottom(self, msg):
        x_inc = msg.actualPosition
        u_rpm = msg.actualVelocity

        self.x = self.inc_to_mm(x_inc)
        self.xD = self.rpm_to_mmps(u_rpm)

        self.time = msg.header.stamp
        self.publish_bottom()
    
    def callback_top(self, msg):
        y_inc = msg.actualPosition
        v_rpm = msg.actualVelocity

        self.y = self.inc_to_mm(y_inc)
        self.yD = self.rpm_to_mmps(v_rpm)

        self.time = msg.header.stamp
        self.publish_top()

    def publish_bottom(self):
        self.bottom_state_msg.header.stamp = self.time
        state = np.array([self.x, self.xD])
        self.bottom_state_msg.vector = state

        self.pub_bottom_state.publish(self.bottom_state_msg)

    def publish_top(self):
        self.top_state_msg.header.stamp = self.time
        state = np.array([self.y, self.yD])
        self.top_state_msg.vector = state

        self.pub_top_state.publish(self.top_state_msg)

    def run(self):
        rospy.Subscriber(self.bottom_motor_topic, MotorStatusMessage, self.callback_bottom, queue_size=1)
        rospy.Subscriber(self.top_motor_topic, MotorStatusMessage, self.callback_top, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsNode()
    node.run()