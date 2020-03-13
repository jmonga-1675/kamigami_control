class KamigamiInterface():

    self.subscriber = rospy.Subscriber('kamigami_cmd', KamigamiCommandMsg, self.update_state)
    self.publisher = rospy.Publisher('kamigami_state', KamigamiStateMsg, queue_size=10)
