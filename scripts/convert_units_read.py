import rospy
import numpy as np

from control_utils.msg import VectorStamped
from pendulum_control.msg import ArrayStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage, MotorStatusMessage

class ConvertUnitsReadNode:
    def __init__(self):
        rospy.init_node(name='convert_units_read', anonymous=True)
        self.init_topics()
        self.init_publishers()
        self.init_variables()
    
    def init_topics(self):
        self.motor_state_topic = 'Maxon_Motor/state'
        self.motor_reading_topic = rospy.get_param('Maxon_Motor/reading')

    def init_publishers(self):
        self.pub_state = rospy.Publisher(self.motor_state_topic, ArrayStamped, queue_size=1)

        self.state_msg = ArrayStamped()

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
        0.005 [m/rev] / 60 [s/min] = 1/12000 [m/s / rpm]
        """
        return rpm / 12000


    def callback(self, msg):
        x_inc = msg.actualPosition
        u_rpm = msg.actualVelocity

        self.x = self.inc_to_m(x_inc)
        self.xD = self.rpm_to_mps(u_rpm)

        self.time = msg.header.stamp
        self.publish()

    def publish(self):
        self.state_msg.header.stamp = self.time
        state = np.array([self.x, self.xD])
        self.state_msg.vector = state

        self.pub_state.publish(self.state_msg)

    def run(self):
        rospy.Subscriber(self.motor_reading_topic, MotorStatusMessage, self.callback, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsReadNode()
    node.run()