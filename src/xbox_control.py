#!/usr/bin/python
import rospy
import time
from sensor_msgs.msg import Joy
from kamigami_control.msg import KamigamiCommandMsg, KamigamiStateMsg

class XboxControl():

    def __init__(self):
        self.subscriber = rospy.Subscriber('joy', Joy, self.process_input)
        self.publisher = rospy.Publisher('kamigami_cmd', KamigamiCommandMsg, queue_size=10)

    def process_input(self, data):
        axes = data.axes
        input_dict = {'joy_left_y': 1, 'joy_right_y': 4}
        msg = KamigamiCommandMsg()
        msg.motor_left = axes[input_dict['joy_left_y']]
        msg.motor_right = axes[input_dict['joy_right_y']]
        msg.motor_left, msg.motor_right = round(msg.motor_left, 1), round(msg.motor_right, 1)
        if abs(msg.motor_left) <= .1:
            msg.motor_left = 0
        if abs(msg.motor_right) <= .1:
            msg.motor_right = 0
        self.publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(100)
        print('Running...')
        while not rospy.is_shutdown():
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        msg = KamigamiCommandMsg()
        msg.motor_left = 0
        msg.motor_right = 0
        self.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('xbox_control', anonymous=True)
    controller = XboxControl()
    controller.run()