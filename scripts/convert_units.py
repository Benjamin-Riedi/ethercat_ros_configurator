import rospy

from control_utils.msg import ScalarStamped
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

        self.top_position_topic = rospy.get_param('/topic/position/top', '/top/x')
        self.top_velocity_topic = rospy.get_param('/topic/velocity/top', '/top/xD')

        self.bottom_position_topic = rospy.get_param('/topic/position/bottom', '/bottom/x')
        self.bottom_velocity_topic = rospy.get_param('/topic/velocity/bottom', '/bottom/xD')

    def init_publishers(self):
        self.pub_x = rospy.Publisher(self.bottom_position_topic, ScalarStamped, queue_size=1)
        self.pub_y = rospy.Publisher(self.top_position_topic, ScalarStamped, queue_size=1)
        self.pub_xD = rospy.Publisher(self.bottom_velocity_topic, ScalarStamped, queue_size=1)
        self.pub_yD = rospy.Publisher(self.top_velocity_topic, ScalarStamped, queue_size=1)

        self.x_msg = ScalarStamped()
        self.y_msg = ScalarStamped()
        self.xD_msg = ScalarStamped()
        self.yD_msg = ScalarStamped()

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
        self.x_msg.header.stamp = self.time
        self.x_msg.scalar = self.x
        self.pub_x.publish(self.x_msg)

        self.xD_msg.header.stamp = self.time
        self.xD_msg.scalar = self.xD
        self.pub_xD.publish(self.xD_msg)

    def publish_top(self):
        self.y_msg.header.stamp = self.time
        self.y_msg.scalar = self.y
        self.pub_y.publish(self.y_msg)

        self.yD_msg.header.stamp = self.time
        self.yD_msg.scalar = self.yD
        self.pub_yD.publish(self.yD_msg)

    def run(self):
        rospy.Subscriber(self.bottom_motor_topic, MotorStatusMessage, self.callback_bottom, queue_size=1)
        rospy.Subscriber(self.top_motor_topic, MotorStatusMessage, self.callback_top, queue_size=1)

        rospy.spin()

if __name__ == '__main__':

    node = ConvertUnitsNode()
    node.run()