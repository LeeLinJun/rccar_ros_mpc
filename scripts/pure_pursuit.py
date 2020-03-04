#!/usr/bin/env python

import rospy, math
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry,Path
from tf.transformations import euler_from_quaternion
import numpy as np

class Controller:
  def __init__(self):
    self.v = 0.2
    self.wheelbase = 0.324
    self.max_steering = 0.34
    self.state = np.array([0,0,0])
    self.state_hat = np.array([0,0,0])
    self.global_goal = np.array([0,0,0])

    self.ackermann_cmd_topic = 'drive'
    self.frame_id = 'odom'
    rospy.Subscriber('/odom', Odometry, self.observe)
    # rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, self.set_goal, queue_size=1)
    rospy.Subscriber('/rrt_path', Path, self.set_goal)


  def observe(self,data):
    q = data.pose.pose.orientation;
    _, _, yaw = euler_from_quaternion ([q.x, q.y, q.z, q.w]) 
    self.state = np.array([data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  yaw])

  def set_goal(self, data):
    min_dis = np.inf
    for i in range(len(data.poses)):
      q = data.poses[i].pose.orientation;
      _, _, yaw = euler_from_quaternion ([q.x, q.y, q.z, q.w]) 
      p = np.array([data.poses[i].pose.position.x,
                    data.poses[i].pose.position.y,
                    yaw])
      dis = np.linalg.norm(self.state[:2]-p[:2])
      if dis < min_dis:
        min_dis = dis
        path_start = i
    self.goal = data.poses[path_start + 30] if len(data.poses) >= path_start + 30 else data.poses[-1]#data.poses[-1]#
    self.global_goal[0] = data.poses[i].pose.position.x
    self.global_goal[1] = data.poses[i].pose.position.y

    q = self.goal.pose.orientation;
    _, _, yaw = euler_from_quaternion ([q.x, q.y, q.z, q.w]) 
    self.state_hat = np.array([self.goal.pose.position.x,
                      self.goal.pose.position.y,
                      yaw])

  def control(self):
    print(self.state, self.state_hat)
    angle = self.state_hat[2] - self.state[2]
    if angle >  np.pi:
      angle -= 2 * np.pi
    elif angle < - np.pi:
      angle += 2 * np.pi
    print(angle)
    steering = np.arctan2(2 * self.wheelbase * np.sin(angle / 2), self.v * 1)
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = self.frame_id
    msg.drive.steering_angle = steering if np.abs(steering) <  self.max_steering else  self.max_steering * np.sign(steering)
    msg.drive.speed = self.v if np.linalg.norm(self.state[:2]-self.global_goal[:2]) > 0.2 else 0
    print(msg.drive.steering_angle, msg.drive.speed)
    return msg

if __name__ == '__main__': 
  try:
    drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)
    rospy.init_node('pure_pursuite_py')
    c = Controller()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      drive_pub.publish(c.control())
      # rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
      r.sleep()
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass