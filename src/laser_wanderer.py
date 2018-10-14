#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/scan'
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
POSE_TOPIC = '/sim_car_pose/pose'
VIZ_TOPIC = '/laser_wanderer/rollouts'

MAX_PENALTY = 10000

class LaserWanderer:
  def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=1)
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb, queue_size=2)
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub, queue_size=1)
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size=1)
    
  def viz_sub(self, msg):
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
    
    trans = np.array([msg.pose.position.x, msg.pose.position.y]).reshape((2,1))
    car_yaw = utils.quaternion_to_angle(msg.pose.orientation)
    rot_mat = utils.rotation_matrix(car_yaw)
    for i in xrange(self.rollouts.shape[0]):
        robot_config = self.rollouts[i,-1,0:2].reshape(2,1)
        map_config = rot_mat*robot_config+trans
        map_config.flatten()
        pose = Pose()
        pose.position.x = map_config[0]
        pose.position.y = map_config[1]
        pose.position.z = 0.0
        pose.orientation = utils.angle_to_quaternion(self.rollouts[i,-1,2]+car_yaw)
        pa.poses.append(pose)
    self.viz_pub.publish(pa)
    
  def compute_cost(self, delta, rollout_pose, laser_msg):
    cost = np.abs(delta)
    
    # Compute the angle between this rollout and robot
    raycast_angle = np.arctan2(rollout_pose[1], rollout_pose[0])
    raycast_length = np.sqrt(rollout_pose[0]**2 + rollout_pose[1]**2)
    
    
    scan_idx = int((raycast_angle - laser_msg.angle_min)/laser_msg.angle_increment)
    
    if scan_idx < 0 or scan_idx >= len(laser_msg.ranges):
        return cost
    
    if (not math.isnan(laser_msg.ranges[scan_idx])) and (raycast_length > laser_msg.ranges[scan_idx] - np.abs(self.laser_offset)):
        cost += MAX_PENALTY
        
    return cost    
    
  def wander_cb(self, msg):
    start = rospy.Time.now().to_sec()
    
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float)
    traj_depth = 0
    while (rospy.Time.now().to_sec() - start < self.compute_time and 
           traj_depth < self.rollouts.shape[1]):
        for i in xrange(self.deltas.shape[0]):
            delta_costs[i] += self.compute_cost(self.deltas[i],
                                                self.rollouts[i,traj_depth,:],
                                                msg)
        traj_depth += 1

    delta_idx = np.argmin(delta_costs)
    #print np.array(delta_costs, dtype=np.int)
    #print str(self.deltas[delta_idx]) + ' ' + str(traj_depth) + ' ' + str(delta_costs[delta_idx])
    
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = self.deltas[delta_idx]
    ads.drive.speed = self.speed
    
    self.cmd_pub.publish(ads)

    
'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  x,y,theta = pose
  v,delta,dt = control
  
  if np.abs(delta) < 1e-2:
    dx = v*np.cos(theta)*dt
    dy = v*np.sin(theta)*dt
    dtheta = 0.0
  else:
    beta = np.arctan(0.5*np.tan(delta))
    sin2beta = np.sin(2*beta)
    dtheta = ((v/car_length) * sin2beta) * dt
    dx = (car_length/sin2beta)*(np.sin(theta+dtheta)-np.sin(theta))
    dy = (car_length/sin2beta)*(-1*np.cos(theta+dtheta)+np.cos(theta))
    
  while theta + dtheta > 2*np.pi:
    dtheta -= 2*np.pi
    
  while theta + dtheta < 2*np.pi:
    dtheta += 2*np.pi
    
  return np.array([x+dx, y+dy, theta+dtheta], dtype=np.float)
  
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  T = controls.shape[0]
  rollout = np.zeros((T, 3), dtype=np.float)
  
  cur_pose = init_pose[:]

  for t in xrange(T):
    control = controls[t, :]  
    rollout[t,:] = kinematic_model_step(cur_pose, control, car_length)
    cur_pose = rollout[t,:]   
    
  return rollout
   
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
    
  return rollouts, deltas

    

def main():

  rospy.init_node('laser_wanderer', anonymous=True)
  speed = rospy.get_param("~speed", 1.0)
  min_delta = rospy.get_param("~min_delta", -0.34)
  max_delta = rospy.get_param("~max_delta", 0.341)
  delta_incr = rospy.get_param("~delta_incr", 0.34/3)
  dt = rospy.get_param("~dt", 0.01)
  T = rospy.get_param("~T", 150)
  car_length = rospy.get_param("car_kinematics/car_length", 0.33)
  compute_time = rospy.get_param("~compute_time", 0.09)
  laser_offset = rospy.get_param("~laser_offset", 2.0)
  
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
                                           
  lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)
  rospy.spin()
  

if __name__ == '__main__':
  main()
