def turn(dir):
    
    cmd=Twist()
    if dir==1: ##left turn
        cmd.angular.z=angular_velocity_z
    else:
        cmd.angular.z=-1*angular_velocity_z
    rospy.loginfo("Turn commence")
    for i in range(5):
        pub.publish(cmd)
        rate.sleep()
    cmd=Twist()
    pub.publish(cmd)
    rospy.loginfo("Turn end")


def go_forward():
    cmd=Twist()
    cmd.linear.x=linear_velocity_x
    pub.publish(cmd)
    rate.sleep()
    cmd=Twist()
    pub.publish(cmd)
    

def turn_around():
    for i in range(10):
        pub.publish(cmd)
        rate.sleep()
    cmd=Twist()
    pub.publish(cmd)
    rospy.loginfo("Turn end")
   



