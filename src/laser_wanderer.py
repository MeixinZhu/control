#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan

SCAN_TOPIC = '/scan'
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

class LaserWanderer:
  def __init__(self, rollouts, deltas, speed, exec_time, laser_offset):
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.exec_time = exec_time
    self.laser_offset = laser_offset
    
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=1)
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb, queue_size=2)
    
  def compute_cost(self, delta, rollout_pose, laser_msg):
    cost = np.abs(delta)
    
    
    
  def wander_cb(self, msg):
    pass
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
  
  

if __name__ == '__main__':
  main()
