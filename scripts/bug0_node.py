#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class Bug0Navigator(Node):
    def __init__(self):
        super().__init__('bug0_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/tretabot/odometry', self.odom_callback, 10)
        self.create_subscription(PointCloud2, '/scan/points', self.scan_callback, 10)
        self.create_timer(0.1, self.control_loop)

        # Meta
        self.goal_x = -1.64 
        self.goal_y = 4.59

        # Estado
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.obstacle_front = False
        self.is_evading = False
        self.evade_timer = 0  # Memoria para seguir girando un poco más

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_callback(self, msg):
        found = False
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            # Aumentamos el rango de detección lateral (y) para no chocar las esquinas
            if 0.1 < point[0] < 0.8 and abs(point[1]) < 0.4 and point[2] > 0.05:
                found = True
                break
        self.obstacle_front = found

    def control_loop(self):
        cmd = Twist()
        
        dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_diff = angle_to_goal - self.current_yaw
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        if dist_to_goal < 0.3:
            self.get_logger().info("¡LLEGAMOS!")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # Lógica de evasión con memoria
        elif self.obstacle_front or self.evade_timer > 0:
            if self.obstacle_front:
                self.evade_timer = 15  # Si ve algo, reinicia el contador (1.5 segundos de giro)
            
            self.get_logger().warning("Esquivando...")
            cmd.linear.x = 0.05  # Un poquito de avance lateral para no estancarse
            cmd.angular.z = 0.9  # Gira fuerte
            self.evade_timer -= 1
        
        else:
            # Navegación normal
            if abs(angle_diff) > 0.3:
                cmd.angular.z = 0.6 if angle_diff > 0 else -0.6
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = 0.4
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = Bug0Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()