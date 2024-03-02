# -*- coding: utf-8 -*-
"""
Created on Tue Feb 20 08:38:32 2024
@author: ESHANT
"""
#!/usr/bin/env python3
import numpy as np
from scipy.integrate import solve_ivp
import rospy
import pyproj
import tf
from geometry_msgs.msg import Transform
from std_msgs.msg import  String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, Vector3Stamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

import tf2_ros, tf2_geometry_msgs
import matsya_mmg
import utils

class matsya_mmg_wrap():            
    
    def __init__(self,
                wind_flag=0,
                wind_speed=0,
                wind_dir=0,
                wave_flag=0,
                wave_height=0,
                wave_period=0,
                wave_dir=0,
                obs_state=None):
        
        self.wind_flag  = wind_flag
        self.wind_speed = wind_speed
        self.wind_dir   = wind_dir

        self.wave_flag = wave_flag
        self.wave_height = wave_height
        self.wave_period = wave_period
        self.wave_dir = wave_dir
        self.obs_state = obs_state                        # Dimensional state
        self.sim_state = obs_state                        # Non dimensional state
        rospy.loginfo('Observation state from init')
        rospy.loginfo(self.obs_state)

        self.scale = 75.5
        self.L = matsya_mmg.L/self.scale
        self.U = matsya_mmg.Fn * np.sqrt(matsya_mmg.g * self.L)

        self.rate_value = 100
        # self.time_step = 0.02144
        self.time_step = 0.00466  ################how you reached this value ?

        self.delta_c = 0        
        self.gps_datum = np.array(rospy.get_param('matsya/datum'))

        self.ASV_sub = rospy.Subscriber('/ASV', String, self.ASV_callback)
        self.ground_truth_pub = rospy.Publisher('/ground_truth',Odometry, queue_size=5)
        self.sbg_pub = rospy.Publisher('/sbg/imu/data',Imu, queue_size=5)
        self.gps_pub = rospy.Publisher('/ardusimple/gnss',NavSatFix, queue_size=5)

        self.ground_truth = Odometry()
        self.sbg_msg = Imu()
        self.ardusimple_msg = NavSatFix()

    def ssa(self,theta):
        if theta > np.pi: return theta - 2*np.pi
        elif theta < -np.pi: return  2*np.pi + theta
        return theta

    def ASV_callback(self,msg):
        command_string = msg.data
        try:
            self.delta_c = int(command_string.split(',')[4])*np.pi/180   #####what /ASV topic is publishing ?
        except:
            self.delta_c = 0     
    
    def step(self):

        tspan = (0, self.time_step)
        yinit = self.sim_state
        # self.delta_c = 0 # (35.0 / 180.0) * np.pi

        sol = solve_ivp(lambda t,v: matsya_mmg.matsya_ode(t,v,self.delta_c, wind_flag=self.wind_flag,
                                                wind_speed=self.wind_speed, wind_dir=self.wind_dir,
                                                wave_flag=self.wave_flag, wave_height=self.wave_height,
                                                wave_period=self.wave_period, wave_dir=self.wave_dir),
                        tspan, yinit, dense_output=True)
        
        u = sol.y[0, -1]*self.U
        v = sol.y[1, -1]*self.U
        r = sol.y[2, -1]*self.U/self.L
        x = sol.y[3, -1]*self.L
        y = sol.y[4, -1]*self.L
        psi_rad = sol.y[5, -1]  # psi
        psi = psi_rad % (2 * np.pi)
        delta = sol.y[6, -1]

        self.sim_state = np.array([sol.y[0, -1], sol.y[1, -1], sol.y[2, -1], sol.y[3, -1], sol.y[4, -1], psi, sol.y[6, -1]])
        obs_state = np.array([u, v, r, x, y, psi, delta])    # This state is in NED

        self.obs_state = obs_state

    ''' Function to get x,y coordinates of goal from GPS coordinates '''
    def xy_to_gps(self, origin_lat, origin_long, x, y):
    # Calculate distance and azimuth between GPS points
        azimuth1 = np.arctan2(x, y) * 180.0 / np.pi
        dist = np.sqrt(x**2 + y**2)
        geodesic = pyproj.Geod(ellps='WGS84')
        goal_long, goal_lat, backaz  = geodesic.fwd(origin_long, origin_lat, azimuth1, dist)
        return goal_lat, goal_long

    def vector_stamp(self, vec, stamp=None, frame_id=None):
        vec_stamped = Vector3Stamped()
        vec_stamped.vector = vec
        if stamp is not None:
            vec_stamped.header.stamp = stamp
        if frame_id is not None:
            vec_stamped.header.frame_id = frame_id
        return vec_stamped

    def pose_stamp(self, pose, stamp=None, frame_id=None):
        pose_stamped = PoseStamped()        
        pose_stamped.pose = pose
        if frame_id is not None:
            pose_stamped.header.frame_id = frame_id
        if stamp is not None:
            pose_stamped.header.stamp = stamp
        return pose_stamped

    def main_execute(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(self.rate_value)
        while not rospy.is_shutdown():
            
            # rospy.loginfo("Log Current Observation State")            
            self.step()
            # rospy.loginfo(self.obs_state)
            
            try:
                tf_ned_to_map = tfBuffer.lookup_transform('map', 'ned', rospy.Time())
                tf_base_ned_to_base = tfBuffer.lookup_transform('base_link', 'base_link_ned', rospy.Time())
                tf_base_to_base_ned = tfBuffer.lookup_transform('base_link_ned', 'base_link', rospy.Time())
                tf_base_to_imu = tfBuffer.lookup_transform('imu_link', 'base_link', rospy.Time())                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.loginfo("Transform exception has occured!!!")
                rate.sleep()
                continue

            ''' Publish the filtered states with the desired convention '''
            odom_quat = quaternion_from_euler(0, 0, self.obs_state[5])
            current_time = rospy.Time.now()
    
            self.ground_truth.header.stamp = current_time
            self.ground_truth.header.frame_id = "map"

            self.sbg_msg.header.stamp = current_time
            self.ardusimple_msg.header.stamp = current_time

            self.ground_truth.child_frame_id = "base_link_ned"
            self.ardusimple_msg.header.frame_id = "gps"
            self.sbg_msg.header.frame_id = "imu_link"
        
            # set the ground truth
            pose_ned = Pose(Point(self.obs_state[3], self.obs_state[4], 0.), Quaternion(*odom_quat))
            pose_ned_stamped = self.pose_stamp(pose_ned, frame_id='ned', stamp=current_time)            
            pose_map_stamped = tf2_geometry_msgs.do_transform_pose(pose_ned_stamped, tf_ned_to_map)

            lin_vel_base_ned = Vector3(self.obs_state[0], self.obs_state[1], 0)
            ang_vel_base_ned = Vector3(0, 0, self.obs_state[2])
            lin_vel_base_ned_stamped = self.vector_stamp(lin_vel_base_ned, stamp=current_time)
            ang_vel_base_ned_stamped = self.vector_stamp(ang_vel_base_ned, stamp=current_time)
            lin_vel_base_stamped = tf2_geometry_msgs.do_transform_vector3(lin_vel_base_ned_stamped, tf_base_ned_to_base)
            ang_vel_base_stamped = tf2_geometry_msgs.do_transform_vector3(ang_vel_base_ned_stamped, tf_base_ned_to_base)
            twist_base = Twist(lin_vel_base_stamped.vector, ang_vel_base_stamped.vector)

            self.ground_truth.pose.pose = pose_map_stamped.pose
            self.ground_truth.twist.twist = twist_base

            # set the ardusimple message
            lat, lon = self.xy_to_gps(self.gps_datum[0], self.gps_datum[1], self.ground_truth.pose.pose.position.x, self.ground_truth.pose.pose.position.y)
            self.ardusimple_msg.latitude = lat
            self.ardusimple_msg.longitude = lon
            self.ardusimple_msg.altitude = 0
            position_covariance = 0.03 * np.array([1, 0, 0, 0, 1, 0, 0, 0, 1])
            self.ardusimple_msg.position_covariance = position_covariance.tolist()
            self.ardusimple_msg.position_covariance_type = np.uint8(2)

            # set the sbg message
            sbg_quat = quaternion_from_euler(0, 0, self.ssa(-self.obs_state[5]))
            self.sbg_msg.orientation = Quaternion(*sbg_quat)
            sbg_ang_vel_stamped = tf2_geometry_msgs.do_transform_vector3(self.vector_stamp(self.ground_truth.twist.twist.angular, stamp=current_time), tf_base_to_imu)
            self.sbg_msg.angular_velocity = sbg_ang_vel_stamped.vector
            
            state_space_dot = matsya_mmg.matsya_ode(self.time_step, self.sim_state,self.delta_c, wind_flag=self.wind_flag,
                                                wind_speed=self.wind_speed, wind_dir=self.wind_dir,
                                                wave_flag=self.wave_flag, wave_height=self.wave_height,
                                                wave_period=self.wave_period, wave_dir=self.wave_dir)
            lin_vel_dot = np.zeros(3)
            lin_vel_dot[0:2] = state_space_dot[0:2]*((self.U)**2/self.L)    # Dimensionalizing derivative of linear velocity
            
            acc = utils.vector_add(Vector3(lin_vel_dot[0], lin_vel_dot[1],lin_vel_dot[2]), 
                                    utils.vector_cross(ang_vel_base_ned_stamped.vector, lin_vel_base_ned_stamped.vector))
            acc_stamped = self.vector_stamp(acc, stamp=current_time)
            acc_base_stamped = tf2_geometry_msgs.do_transform_vector3(acc_stamped, tf_base_ned_to_base)
            self.sbg_msg.linear_acceleration = acc_base_stamped.vector

            orientation_covariance = 3e-5 * np.array([1, 0, 0, 0, 1, 0, 0, 0, 1])
            linear_acceleration_covariance = 4e-6 * np.array([1, 0, 0, 0, 1, 0, 0, 0, 1])
            angular_velocity_covariance = 8e-7 * np.array([1, 0, 0, 0, 1, 0, 0, 0, 1])
            self.sbg_msg.orientation_covariance = orientation_covariance.tolist()
            self.sbg_msg.linear_acceleration_covariance = linear_acceleration_covariance.tolist()
            self.sbg_msg.angular_velocity_covariance = angular_velocity_covariance.tolist()

            # publish the messages
            self.ground_truth_pub.publish(self.ground_truth)
            self.sbg_pub.publish(self.sbg_msg)
            self.gps_pub.publish(self.ardusimple_msg)

            rate.sleep()

if __name__=="__main__":
    
    try:
        rospy.init_node('matsya_dynamics')
        sim = matsya_mmg_wrap(wind_flag=0,
                    wind_speed=0,
                    wind_dir=0,
                    wave_flag=0,
                    wave_height=0,
                    wave_period=0,
                    wave_dir=0,
                    obs_state=np.array([1e-6, 0, 0, 0, 0, -np.pi/2, 0]))          # initial state in NED
        sim.main_execute()
    except KeyboardInterrupt:
        rospy.loginfo("Hello from Keyboard!!")