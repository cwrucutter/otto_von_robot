#!/usr/bin/env python

"""
   otto_enc_odom.py
   creates tf and odometry messages from encoder messages
   reference: http://rossum.sourceforge.net/papers/DiffSteer/
   Copyright 2018, Charles Hart
"""

import rospy
import roslib

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from snowmower_msgs.msg import EncMsg

from math import sin, cos, pi

class DiffTf:

    def __init__(self):
        rospy.init_node("snowmower_enc_odom")
        
        ### initialize from ros parameters 
        self.rate = rospy.get_param('~rate', 20.0)  # the rate at which to publish the odom->base_link transform
        self.tpm_left = float(rospy.get_param('~tpm_left', 27150)) # ticks per meter left
        self.tpm_right = float(rospy.get_param('~tpm_right', 26500)) # ticks per meter right
        self.base_width = float(rospy.get_param('~base_width', 0.55)) # The wheel base width in meters
        
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') 
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') 
        
        self.encoder_min = rospy.get_param('~encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('~encoder_max', 2147483647)
        self.encoder_low_wrap = rospy.get_param('~wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('~wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.encoder_topic = rospy.get_param('~encoder_topic', '/enc')
        self.odom_topic = rospy.get_param('~odom_topic', '/wheelencoder/odom')

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.first_update = True
        
        rospy.loginfo("Encoder config: {} tpm_left, {} tpm_right, {} base_width"\
                .format(self.tpm_left, self.tpm_right, self.base_width))
        rospy.loginfo("min : max ({} : {}), low : high ({} : {})"\
                .format(self.encoder_min, self.encoder_max, self.encoder_low_wrap, self.encoder_high_wrap))
        rospy.loginfo("Running at {} rate, with base \"{}\" and odom \"{}\""\
                .format(self.rate, self.base_frame_id, self.odom_frame_id))
        rospy.loginfo("Subscribing to \"{}\", Publishing to \"{}\""\
                .format(self.encoder_topic, self.odom_topic))


        # internal data
        self.enc_left = None
        self.enc_right = None
        self.init_lencoder = None
        self.init_rencoder = None
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.left = 0
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        # position in xy plane 
        self.x = 0                  
        self.y = 0
        self.th = 0
        # linear and angular velocity
        self.dx = 0
        self.dr = 0
        self.then = rospy.Time.now()
        
        # sub
        rospy.Subscriber(self.encoder_topic, EncMsg, self.encoderCallback)

        # pub
        self.odomPub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)

        # cast
        self.odomBroadcaster = TransformBroadcaster()
        
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.tpm_left
                d_right = (self.right - self.enc_right) / self.tpm_right
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            if (elapsed == 0):
                self.dx = 0
                self.dr = 0
            else:
                self.dx = d / elapsed
                self.dr = th / elapsed
           
             
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if (th != 0):
                self.th = self.th + th
                
            #if (self.first_update):
            #    self.x = 0
            #    self.y = 0
            #    self.th = 0
            #    self.dx = 0
            #    self.dr = 0
            #    print(".")
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
    ### runs every time the snowmower's enc message is published
    def encoderCallback(self, msg):
        l_enc = msg.left
        r_enc = msg.right

        if self.first_update:
            self.init_lencoder = l_enc
            self.init_rencoder = r_enc
            self.first_update = False

        if (l_enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
        if (r_enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        if (l_enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
        if (r_enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.left = 1.0 * (l_enc + self.lmult * (self.encoder_max - self.encoder_min) - self.init_lencoder) 
        self.right = 1.0 * (r_enc + self.rmult * (self.encoder_max - self.encoder_min) - self.init_rencoder)
        
        self.prev_lencoder = l_enc
        self.prev_rencoder = r_enc
        
            

if __name__ == '__main__':
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
