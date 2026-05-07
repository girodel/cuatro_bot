#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug1Navigator(Node):
    def __init__(self):
        super().__init__('bug1_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/tetrabot/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.create_timer(0.1, self.control_loop)

        # ====== META ======
        self.goal_x = -3.83 
        self.goal_y = -3.95
        # ==================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.obstacle_front = False
        self.obstacle_right = False
        
        # --- Variables de Memoria Bug 1 ---
        self.state = 'GO_TO_GOAL' # GO_TO_GOAL, CIRCUMNAVIGATE, GO_TO_MIN_POINT
        self.hit_point = {'x': 0.0, 'y': 0.0}
        self.leave_point = {'x': 0.0, 'y': 0.0}
        self.min_dist_to_goal = float('inf')
        
        # Para saber si ya dimos la vuelta (evitar que detecte el hit_point apenas empieza)
        self.left_hit_point = False 

        self.get_logger().info(f"🚀 Navegador BUG 1 Iniciado. Destino: X={self.goal_x}, Y={self.goal_y}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        min_front = 10.0
        min_right = 10.0
        
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.15 or r > 10.0:
                continue
            angle = angle_min + i * angle_inc
            
            if -0.78 <= angle <= 0.78:
                if r < min_front: min_front = r
            if -2.35 <= angle < -0.78:
                if r < min_right: min_right = r

        self.obstacle_front = min_front < 0.85
        self.obstacle_right = min_right < 0.6

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def control_loop(self):
        cmd = Twist()
        dist_to_goal = self.distance(self.current_x, self.current_y, self.goal_x, self.goal_y)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        
        angle_diff = angle_to_goal - self.current_yaw
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        if dist_to_goal < 0.3:
            self.get_logger().info("✅ ¡LLEGAMOS A LA META!", once=True)
            self.cmd_pub.publish(Twist())
            return

        # ==========================================
        #           MÁQUINA DE ESTADOS BUG 1
        # ==========================================
        
        if self.state == 'GO_TO_GOAL':
            if self.obstacle_front:
                self.get_logger().warning("🛑 Obstáculo. Guardando Hit-Point e iniciando vuelta completa.")
                # Guarda dónde chocó
                self.hit_point['x'] = self.current_x
                self.hit_point['y'] = self.current_y
                # Resetea la memoria de la mejor distancia
                self.min_dist_to_goal = dist_to_goal
                self.leave_point['x'] = self.current_x
                self.leave_point['y'] = self.current_y
                
                self.left_hit_point = False
                self.state = 'CIRCUMNAVIGATE'
            else:
                if abs(angle_diff) > 0.2:
                    cmd.linear.x = 0.1
                    cmd.angular.z = 0.8 if angle_diff > 0 else -0.8
                else:
                    cmd.linear.x = 0.25 
                    cmd.angular.z = 0.0

        elif self.state == 'CIRCUMNAVIGATE':
            # 1. Registrar el punto más cercano a la meta mientras damos la vuelta
            if dist_to_goal < self.min_dist_to_goal:
                self.min_dist_to_goal = dist_to_goal
                self.leave_point['x'] = self.current_x
                self.leave_point['y'] = self.current_y

            # 2. Comprobar si ya dimos la vuelta completa (volvimos al hit_point)
            dist_to_hit = self.distance(self.current_x, self.current_y, self.hit_point['x'], self.hit_point['y'])
            
            # Margen de seguridad: El robot debe alejarse al menos 0.5m del inicio para no activar esto inmediatamente
            if dist_to_hit > 0.5:
                self.left_hit_point = True
            
            if self.left_hit_point and dist_to_hit < 0.4:
                self.get_logger().info("🔄 Vuelta completa. Buscando el mejor punto de salida...")
                self.state = 'GO_TO_MIN_POINT'
            else:
                cmd = self.wall_follow()

        elif self.state == 'GO_TO_MIN_POINT':
            # Seguir la pared hasta llegar al punto óptimo guardado
            dist_to_leave = self.distance(self.current_x, self.current_y, self.leave_point['x'], self.leave_point['y'])
            
            if dist_to_leave < 0.35:
                self.get_logger().info("🎯 Punto de salida óptimo alcanzado. ¡Directo a la meta!")
                self.state = 'GO_TO_GOAL'
            else:
                cmd = self.wall_follow()

        self.cmd_pub.publish(cmd)

    def wall_follow(self):
        """Lógica independiente para bordear por la derecha"""
        t = Twist()
        if self.obstacle_front:
            t.linear.x = 0.0
            t.angular.z = 0.8  # Gira izquierda (se aleja de la pared)
        elif self.obstacle_right:
            t.linear.x = 0.25  # Avanza paralelo
            t.angular.z = 0.0
        else:
            t.linear.x = 0.15 
            t.angular.z = -0.6 # Gira derecha (busca la pared)
        return t

def main():
    rclpy.init()
    node = Bug1Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()