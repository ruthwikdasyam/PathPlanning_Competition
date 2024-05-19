#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import select
import tty
import termios
import time
import math
import numpy as np
# import acode as p1pygame
# import pygameold as p1pygame
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan


wheel_radius = 3.3 # r
track_width = 28.7 # L 
timestep = 1

# action_list = p1pygame.action_list
# points_list = p1pygame.points_list
points_list = [[50, 100, 0], (76, 100, 0), (94, 95, 11), (118, 84, 11), (141, 72, 11), (160, 68, 0), (178, 73, 1), (193, 86, 2), (208, 97, 1), (231, 109, 0), (255, 120, 1), (273, 124, 0), (292, 120, 11), (306, 107, 10), (322, 86, 10), (338, 66, 10), (353, 54, 11), (376, 43, 11), (395, 39, 0), (421, 39, 0), (447, 39, 0), (466, 44, 1), (480, 56, 1), (496, 77, 2), (512, 97, 2), (527, 109, 1), (550, 120, 1)]
# print(len(action_list))
print(len(points_list))
print(points_list)
points_list= [list(i) for i in points_list]

for i in range(len(points_list)-1):
    points_list[i+1][0] -= points_list[0][0]    
    points_list[i+1][1] -= points_list[0][1]    

points_list[0][0] -= points_list[0][0]    
points_list[0][1] -= points_list[0][1] 

print(points_list)


def parse_laser_data(laser_data):
    laser=[]
    i=0
    while i < 180:
        dist = laser_data[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90)
        laser.append([dist, angle])
        i +=1
    return laser

def laser_data_to_repulsive_vectors(laser_data):
    repulsive_vectors = []
    for distance, angle in laser_data:
        
        
        if angle>0 and angle<0.07:
          print(distance,end="|")
        if distance==0:
          distance=0.01
        # Calculate x and y components of the vector
        x_component =  np.cos(angle) / distance  # Inversely proportional to distance
        y_component =  np.sin(angle) / distance # Inversely proportional to distance
        repulsive_vectors.append([x_component, y_component])
    print("\n#######################")
    # Sum all repulsive vectors to get the resultant vector
    resultant_vector = -1* np.sum(repulsive_vectors, axis=0)
    
    return resultant_vector/20

def convert_to_velocity(resultant_vector, max_linear_speed, max_angular_speed):
    # Calculate linear velocity magnitude
    linear_velocity_magnitude = np.linalg.norm(resultant_vector)
    # Normalize the resultant vector to get its direction
    direction = resultant_vector / linear_velocity_magnitude if linear_velocity_magnitude > 0 else np.array([0, 0])

    # Calculate angular velocity magnitude based on the direction angle
    angle = np.arctan2(direction[1], direction[0])
    angular_velocity_magnitude = angle * max_angular_speed / np.pi

    # Scale linear velocity within the maximum speed limit
    linear_velocity =  min(linear_velocity_magnitude, max_linear_speed)

    return linear_velocity, angular_velocity_magnitude


def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel , y_rel
    
def reached_waypoint(point1, point2, threshold):
  # Calculate the Euclidean distance between the two points
  distance = np.linalg.norm(np.array(point1) - np.array(point2))
  
  # Check if the distance is within the threshold
  if distance <= threshold:
      return True
  else:
      return False
  

def scale_vector(vector, magnitude):
    # Calculate the current magnitude of the vector
    current_magnitude = np.linalg.norm(vector)

    # Scale the vector to the desired magnitude
    scaled_vector = vector * (magnitude / current_magnitude)

    return scaled_vector

max_linear_speed = 2.0
max_angular_speed = 2.0

class MyrobotController(Node):
    def __init__(self):
        super().__init__('MyrobotController')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_node = rclpy.create_node('OdomSubscriber')
        self.odom_subs = self.sub_node.create_subscription(Odometry, '/odom',  self.odom_callback, 10)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0

        self.sub_node2 = rclpy.create_node('ScanSubscriber')
        self.laser_subs = self.sub_node2.create_subscription(LaserScan, '/scan',  self.scan_callback, QoSProfile(depth=15, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.ranges = {}



    def odom_callback(self, msg):
        self.robot_x =  msg.pose.pose.position.x
        self.robot_y =  msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

    #     # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, self.robot_theta = self.quaternion_to_euler(x, y, z, w)


    def scan_callback(self, msg):
        self.ranges = msg.ranges
        self.num_ranges=len(self.ranges)
        print(self.num_ranges)

        # self.get_logger().info('Position - X: %f, Y: %f' % (self.robot_x, self.robot_y))
        # self.get_logger().info('Orientation Angle: %f degrees' % math.degrees(self.robot_theta))

    def quaternion_to_euler(self, x, y, z, w):
    #     # Conversion to Euler angles (roll, pitch, yaw)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        return roll, pitch, yaw


    def run(self):
        # rclpy.spin_once(self.sub_node)
        # rclpy.spin_once(self.sub_node2)
        # adjusting points list based on odom values
        # for i in range(len(points_list)):
        #     points_list[i][0] += 100*self.robot_x
        #     points_list[i][1] += 100*self.robot_y

        # correcting points list based on robot's orientation
        # print("robot theta ", self.robot_theta)
        # R_Matrix = [[math.cos(self.robot_theta), -math.sin(self.robot_theta)], [math.sin(self.robot_theta), math.cos(self.robot_theta)]]
        # for i in range(len(points_list)):
        #     points_list[i][0] = R_Matrix[0][0]*points_list[i][0] + R_Matrix[0][1]*points_list[i][1]
        #     points_list[i][1] = R_Matrix[1][0]*points_list[i][0] + R_Matrix[1][1]*points_list[i][1]

        # print(points_list)


        # velocity publisher
        velocity_message = Twist()
        linear_vel=0.0
        angular_vel=0.0
        
        min_thetadiff = 100
        max_thetadiff = -100

        for i,point in enumerate(points_list[:-1]):
            print("new point ", points_list[i+1])

            while True: 
                rclpy.spin_once(self.sub_node)
                rclpy.spin_once(self.sub_node2)
                # rclpy.spin_once(self.sub_node)
                # print(self.ranges)

                targetx = points_list[i+1][0]
                targety = points_list[i+1][1]

                robotx, roboty, robott = self.robot_x, self.robot_y, self.robot_theta 

                # linear_vel_cm = (wheel_radius/2)*(action[0]+action[1])
                # angular_vel_cm = (wheel_radius/track_width)*(action[1]-action[0])
                
                # theta_diff = self.robot_theta - math.atan2((points_list[i+1][1]-(self.robot_y*100)), (points_list[i+1][0]-(self.robot_x*100)))
                # if abs(theta_diff) >180:
                #      theta_diff = abs(theta_diff) - 360

                # print(theta_diff)
                # linear_vel = 0.25*(1.00 - 0.1*abs(theta_diff))
                # angular_vel = 2*-theta_diff

                # if theta_diff > angular_vel:
                #     max_thetadiff = angular_vel
                # if theta_diff < angular_vel:
                #     min_thetadiff = angular_vel

                #find difference between those 2 points
                dist_diff = ((points_list[i+1][1]-(self.robot_y*100))**2+(points_list[i+1][0]-(self.robot_x*100))**2)**0.5
                # print(dist_diff)
                if dist_diff < 5:
                    print("breaking...")
                    break
                if reached_waypoint([self.robot_x, self.robot_y], [targetx, targety], 5):
                    break
                way_x, way_y = absolute2relative(targetx, targety, robotx, roboty, robott)
                target_vector = scale_vector(np.array([way_x, way_y]), 5)
                laser = parse_laser_data(self.ranges)
                repulsion_vector = laser_data_to_repulsive_vectors(laser)
                direction_vector = target_vector + repulsion_vector

                linear_vel, angular_vel = convert_to_velocity(direction_vector, max_linear_speed, max_angular_speed)


                # Publish the twist message
                velocity_message.linear.x = linear_vel
                velocity_message.angular.z = angular_vel
                
                # publish velocities
                self.cmd_vel_pub.publish(velocity_message)



                # time.sleep(timestep)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = 0.0
        print("min theta ", min_thetadiff )
        print("max theta ", max_thetadiff )
        # publish velocities
        self.cmd_vel_pub.publish(velocity_message)

# main
def main(args=None):
    rclpy.init(args=args)
    node = MyrobotController()
    # rclpy.spin(node)

    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





