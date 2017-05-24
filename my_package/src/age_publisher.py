#!/usr/bin/env python
import rospy
from my_package.msg import Age

rospy.init_node('publish_age')
var=Age()
var.years=0
var.months=0
var.days=0
rate=rospy.Rate(10)
pub=rospy.Publisher('/AgeData',Age,queue_size=1)
while not rospy.is_shutdown():
    pub.publish(var)
    var.days+=1
    var.months+=var.days/30
    var.years=var.months/12
    rate.sleep()


