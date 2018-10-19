#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


twist =Twist();
pub=rospy.Publisher("/mobil_serial_topic", String, queue_size=50)


def callbackcmd(data):
    
    #taking the data from husky_controller/cmd_vel  topic
    twist.linear.x = data.linear.x 
    twist.angular.z = data.angular.z
    #if anglular speed is negative angular way is 0 else 1 
 


def main():

    rospy.init_node('cmd_sub_serial')
    rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callbackcmd)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	    if  twist.angular.z > 0 :
		left_wheel = "1"
		right_wheel = "0"

            if  twist.angular.z < 0 :
		left_wheel = "0"
		right_wheel = "1"
	    if twist.linear.x< 0:
		right_wheel = "0"
	 	left_wheel = "0"
	    if twist.linear.x> 0: 
		right_wheel = "1"
		left_wheel = "1"

	

	    if twist.linear.x> 0 and abs(twist.angular.z) > 0.3 :
	
		left_wheel = "1"
		right_wheel = "2"
            if twist.linear.x> 0 and twist.angular.z < -0.3 :
		left_wheel = "2"
		right_wheel = "1"
	
	
	    #if linear speed is negative angular way is 0 else 1 
	    if twist.linear.x==0 and  twist.angular.z==0 :
		right_wheel  = "2"
		left_wheel ="2"

	    rospy.loginfo( "I heard %s", 'S'+ ','+str(right_wheel)+',' +str(left_wheel ))
	   
	  #  pub.publish('s'+ ','+str(right_wheel)+','+ str(left_wheel )+',')
	    pub.publish('s'+ ','+str(right_wheel)+',' +str(left_wheel )+',')
	    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
