# -*- coding: utf-8 -*-
"""
Created on Tue Feb 20 08:41:35 2024

@author: ESHANT
"""

#!/usr/bin/env python3
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Transform, TransformStamped, PoseStamped, Vector3Stamped, PointStamped
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import PyKDL
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

def stamp_vector_to_kdl(v):
    return PyKDL.Vector(v.vector.x, v.vector.y, v.vector.z)

def stamp_quat_to_kdl(q):
    return PyKDL.Rotation.Quaternion(q.quaternion.x, q.quaternion.y, q.quaternion.z, q.quaternion.w)

def stamp_transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                  t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x, 
                                     t.transform.translation.y, 
                                     t.transform.translation.z))

def vector_to_kdl(v):
    return PyKDL.Vector(v.x, v.y, v.z)

def quat_to_kdl(q):
    return PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x, t.rotation.y,
                                                  t.rotation.z, t.rotation.w),
                        PyKDL.Vector(t.translation.x, 
                                     t.translation.y, 
                                     t.translation.z))

def transform_point(point, transform):
    p = stamp_transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res

def transform_vector(vector, transform):
    p = stamp_transform_to_kdl(transform) * PyKDL.Vector(vector.vector.x, vector.vector.y, vector.vector.z)
    res = Vector3Stamped()
    res.vector.x = p[0]
    res.vector.y = p[1]
    res.vector.z = p[2]
    res.header = transform.header
    return res

def transform_pose(pose, transform):
    f = stamp_transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                           pose.pose.orientation.z, pose.pose.orientation.w),
                                            PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res

def transform_linear_velocity(lin_vel, ang_vel, transform):
    
    r = Vector3()
    r.x = -transform.translation.x
    r.y = -transform.translation.y
    r.z = -transform.translation.z

    v1 = vector_to_kdl(lin_vel)
    v2 = vector_to_kdl(ang_vel) * vector_to_kdl(r)

    vec = Vector3Stamped()
    vec.vector.x = v1[0] + v2[0]
    vec.vector.y = v1[1] + v2[1]
    vec.vector.z = v1[2] + v2[2]

    res = transform_vector(vec, transform)
    return res

def vector_cross(vec1, vec2):
    v = vector_to_kdl(vec1) * vector_to_kdl(vec2)
    res = Vector3()
    res.x = v[0]
    res.y = v[1]
    res.z = v[2]
    return res

def vector_add(vec1, vec2):    
    res = Vector3()
    res.x = vec1.x + vec2.x
    res.y = vec1.y + vec2.y
    res.z = vec1.z + vec2.z
    return res