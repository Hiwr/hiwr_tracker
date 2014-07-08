#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib; roslib.load_manifest('hiwr_motors')
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64
#from hyve_msg.msg import EyesLook
from random import randint
from random import uniform
from sensor_msgs.msg import RegionOfInterest
def talker():
    rospy.init_node('random_motors', anonymous=False)
    r = rospy.Rate(0.5) # 10hz
    joints = [rospy.get_param("~pan_name", 'pan_joint'), rospy.get_param("~tilt_name", 'tilt_joint')]
    torque_enable = dict()
    servo_position = dict()

    #Configure joints
    for controller in sorted(joints):
        try:
            rospy.loginfo("controller is "+controller)

            # Torque enable/disable control for each servo
            torque_service = '/' + controller + '/torque_enable'
            rospy.wait_for_service(torque_service)
            torque_enable[controller] = rospy.ServiceProxy(torque_service, TorqueEnable)

            # Start each servo in the disabled state so we can move them by hand
            torque_enable[controller](False)

            # The position controllers
            servo_position[controller] = rospy.Publisher('/' + controller + '/command', Float64)
        except:
            rospy.logerr("Can't contact servo services!")

    while not rospy.is_shutdown():
        for controller in sorted(joints):
            rand_val = uniform(-2,2)
            #Send random float to each joint
            servo_position[controller].publish(rand_val)
        r.sleep()
    """
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('output_face_tracking', RegionOfInterest)
    rospy.loginfo("INIT RANDOM ROI")
    r = rospy.Rate(1) # 10hz

    loop = 0;

    x_offset =  randint(0,50)  ;
    y_offset =  randint(0,50)  ;
    while not rospy.is_shutdown():

        str = RegionOfInterest();
        loop +=1
        if loop == 10 :
          x_offset = randint(0,50)   ;
          y_offset = randint(0,50)  ;
          loop=0;

          rospy.loginfo("Giving New Random Mode")

        str.x_offset = x_offset;
        str.y_offset = y_offset;
        str.width = 640;
        str.height = 480;
        pub.publish(str)
#        rospy.loginfo(str)
#        pub.publish(str)
        r.sleep()
    """
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
