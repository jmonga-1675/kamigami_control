import rospy
import time
import board
import busio
from adafruit_lsm6ds import LSM6DS33
from kamigami_control.msg import KamigamiCommandMsg, KamigamiStateMsg

class KamigamiInterface():

    def __init__(self):
        self.subscriber = rospy.Subscriber('kamigami_cmd', KamigamiCommandMsg, self.send_cmd)
        self.publisher = rospy.Publisher('kamigami_state', KamigamiStateMsg, queue_size=10)
        self.accelerometer_data = []
        # i2c = busio.I2C(board.SCL, board.SDA)
        # self.sensor = LSM6DS33(i2c)
        rospy.on_shutdown(self.shutdown)

    def send_cmd(self, data):
        self.motor_cmd(data.motor_left, data.motor_right)

    def motor_cmd(self, motor_left, motor_right):
        #TODO: Set Kamigami motor values appropriately
        print('Setting motor left to {}'.format(motor_left))
        print('Setting motor right to {}'.format(motor_right))

    def update_state(self):
        #TODO: Setup actual sensor reading support
        # angular_velocity = self.sensor.gyro
        # linear_acceleartion = self.sensor.acceleartion
        # state = KamigamiStateMsg()
        # state.angular_velocity.x, state,angular_velocity.y, state.angular_velocity.z = angular_velocity.x, angular_velocity.y, angular_velocity.z
        # state.linear_acceleration.x, state,linear_acceleration.y, state.linear_acceleration.z = linear_acceleartion.x, linear_acceleartion.y, linear_acceleartion.z
        # self.publisher.publish(KamigamiStateMsg)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.update_state()
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        self.motor_cmd(0, 0)
    
if __name__ = '__main__':
    rospy.init_node('kamigami_interface', anonymous=True)
    kamigami = KamigamiInterface()
    kamigami.run