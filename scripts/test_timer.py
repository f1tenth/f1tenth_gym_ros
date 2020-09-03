import unittest
from gym_bridge import GymBridge
import rospy, rostopic 
from nav_msgs.msg import Odometry
import rostest
import roslaunch

PKG = 'test_timer'

class TestTimer(unittest.TestCase):

    def callback(self):
        window = 5000
        expected_fps = 100  # current setting is to have both timer & drive timer update at 250 Hz (every 0.004s)

        self.published = True
        self.counter += 1
        if self.counter==window-1:
            self.finish_time = rospy.Time.now()
            dt = (self.finish_time-self.start_time).to_sec()
            self.fps = window/dt
            self.assertAlmostEqual(self.fps, expected_fps)      

    def subscribe(self, topic):
        self.published = False
        rospy.init_node('test_timer')
        self.start_time = rospy.Time.now()
        self.counter = 0
        rospy.Subscriber(topic, rospy.AnyMsg, self.callback)

    def test_timer_fps(self):
        topic = '/odom'
        self.subscribe(topic)
    
    def test_drive_timer_fps(self):
        topic = '/race_info'
        self.subscribe(topic)

    def test_fps_dummy_agents(self):
        # roslaunch requires a package 'netifaces' to launch
        
        file_path = '../launch/agent_template.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
        launch.start()
        self.test_timer_fps()
        self.test_drive_timer_fps()

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_timer', TestTimer)