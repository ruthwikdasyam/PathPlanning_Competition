#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# import sys
# import select
# import tty
# import termios
import time
import math
start_time = time.time()

# import acode as p1pygame
import acode_npg as p1pygame
import numpy as np
# import pygameold as p1pygame

wheel_radius = 3.3 # r
track_width = 28.7 # L 
timestep = 1

# action_list = p1pygame.action_list
points_list = p1pygame.points_list
# points_list = [[20, 150, 0], (47, 140, 11), (77, 115, 11), (107, 91, 11), (134, 82, 0), (160, 92, 1), (190, 116, 1), (220, 141, 1), (251, 166, 1), (281, 190, 1), (308, 199, 0), (334, 189, 11), (348, 164, 9), (364, 140, 11), (394, 116, 11), (424, 91, 11), (451, 82, 0), (490, 82, 0), (516, 92, 1), (546, 117, 1)]


# print(len(action_list))
# print(len(points_list))
# print(points_list)
points_list= [list(i) for i in points_list]

for i in range(len(points_list)-1):
    points_list[i+1][0] -= points_list[0][0]    
    points_list[i+1][1] -= points_list[0][1]    

points_list[0][0] -= points_list[0][0]    
points_list[0][1] -= points_list[0][1] 

# print(points_list)


class MyrobotController(Node):
    def __init__(self):
        super().__init__('MyrobotController')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_node = rclpy.create_node('OdomSubscriber')
        self.odom_subs = self.sub_node.create_subscription(Odometry, '/odom',  self.odom_callback, 10)
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0

    def odom_callback(self, msg):
        self.robot_x =  msg.pose.pose.position.x
        self.robot_y =  msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, _, self.robot_theta = self.quaternion_to_euler(x, y, z, w)

        # self.get_logger().info('Position - X: %f, Y: %f' % (self.robot_x, self.robot_y))
        # self.get_logger().info('Orientation Angle: %f degrees' % math.degrees(self.robot_theta))

    def quaternion_to_euler(self, x, y, z, w):
    #     # Conversion to Euler angles (roll, pitch, yaw)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        return roll, pitch, yaw


    def run(self):
        rclpy.spin_once(self.sub_node)
        # adjusting points list based on odom values
              # correcting points list based on robot's orientation
        # print("robot theta ", self.robot_theta)
        # ang = -np.rad2deg(self.robot_theta)
        # if ang < 0:
        #     ang+=360
        R_Matrix = [[math.cos(self.robot_theta), -math.sin(self.robot_theta)], [math.sin(self.robot_theta), math.cos(self.robot_theta)]]
        for i in range(len(points_list)):
            x_new = R_Matrix[0][0]*points_list[i][0] + R_Matrix[0][1]*points_list[i][1]
            y_new = R_Matrix[1][0]*points_list[i][0] + R_Matrix[1][1]*points_list[i][1]
            points_list[i][0] = x_new
            points_list[i][1] = y_new
    
        for i in range(len(points_list)):
            points_list[i][0] += 100*self.robot_x
            points_list[i][1] += 100*self.robot_y

        # print("robot x ", self.robot_x)
        # print("robot y ", self.robot_y)
        # print("robot theta ", self.robot_theta)
        

  
        # print(points_list)


        # velocity publisher
        velocity_message = Twist()
        linear_vel=0.0
        angular_vel=0.0
        
        # min_thetadiff = 100
        # max_thetadiff = -100


        for i,point in enumerate(points_list[:-1]):
            print("new point ", points_list[i+1])

            while True: 
                rclpy.spin_once(self.sub_node)


                # linear_vel_cm = (wheel_radius/2)*(action[0]+action[1])
                # angular_vel_cm = (wheel_radius/track_width)*(action[1]-action[0])
                # limit self.robot_theta to o to 2pi
                # if self.robot_theta < 0:
                #     self.robot_theta += 6.28
                # if self.robot_theta > 6.28:
                #     self.robot_theta -= 6.28

                theta_diff = self.robot_theta - math.atan2((points_list[i+1][1]-(self.robot_y*100)), (points_list[i+1][0]-(self.robot_x*100)))
                if theta_diff >= 3.14:
                    theta_diff = theta_diff - 6.28
                elif theta_diff <= -3.14:
                    theta_diff = theta_diff + 6.28

                # print(theta_diff)
                linear_vel = 0.27*(1.00 - 0.1*abs(theta_diff))
                # linear_vel = 0.26
                angular_vel = 2*-theta_diff

                # print("theta diff: ", theta_diff)

                # if theta_diff > angular_vel:
                #     max_thetadiff = angular_vel
                # if theta_diff < angular_vel:
                #     min_thetadiff = angular_vel

                #find difference between those 2 points
                dist_diff = ((points_list[i+1][1]-(self.robot_y*100))**2+(points_list[i+1][0]-(self.robot_x*100))**2)**0.5
                # print("dist_diff: ", dist_diff)
                if dist_diff < 5:
                    # print("breaking...")
                    break
                
                # need to be corrected accoridng to gazebo's real time factor
                # p_linear = 1
                # p_angular = 1.05

                # linear_vel = (linear_vel_cm/100)*p_linear
                # angular_vel = (angular_vel_cm/1)*p_angular

                #printing actions and publishing velocities
                # print("\nAction ",i)
                # print("Steer Angle",angular_vel)
                # print("Linear Velocity",linear_vel)
                
                # Publish the twist message
                velocity_message.linear.x = linear_vel
                velocity_message.angular.z = angular_vel
                
                # publish velocities
                self.cmd_vel_pub.publish(velocity_message)



                # time.sleep(timestep)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = 0.0
        # print("min theta ", min_thetadiff )
        # print("max theta ", max_thetadiff )
        # publish velocities
        self.cmd_vel_pub.publish(velocity_message)
        print("Time taken: ", time.time()-start_time)

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