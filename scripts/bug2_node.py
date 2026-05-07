#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug2Navigator(Node):
    def __init__(self):
        super().__init__('bug2_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/tetrabot/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.create_timer(0.1, self.control_loop)

        # ====== META ======
        self.goal_x = 5.668214962097453
        self.goal_y = 3.3053792974265988
        # ==================

        # Posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Posición de inicio (Para trazar la Línea-M)
        self.start_x = None
        self.start_y = None
        
        self.obstacle_front = False
        self.obstacle_right = False
        
        # --- Variables de Memoria Bug 2 ---
        self.state = 'GO_TO_GOAL' # GO_TO_GOAL, WALL_FOLLOW
        self.hit_dist_to_goal = 0.0  # Distancia a la meta cuando chocó

        self.get_logger().info(f"🚀 Navegador BUG 2 Iniciado. Destino: X={self.goal_x}, Y={self.goal_y}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # Registrar el punto de inicio la primera vez que recibimos odometría
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f"📍 Punto de inicio registrado (Línea-M): X={self.start_x:.2f}, Y={self.start_y:.2f}")

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
            
            # Frente (-45 a +45 grados)
            if -0.78 <= angle <= 0.78:
                if r < min_front: min_front = r
            # Derecha (-135 a -45 grados)
            if -2.35 <= angle < -0.78:
                if r < min_right: min_right = r

        self.obstacle_front = min_front < 0.85
        self.obstacle_right = min_right < 0.6

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def dist_to_m_line(self):
        """Calcula a qué distancia está el robot de la Línea-M imaginaria"""
        if self.start_x is None: return 100.0
        
        # Fórmula de distancia de un punto (current) a una línea que pasa por (start) y (goal)
        num = abs((self.goal_x - self.start_x) * (self.start_y - self.current_y) - (self.start_x - self.current_x) * (self.goal_y - self.start_y))
        den = math.sqrt((self.goal_x - self.start_x)**2 + (self.goal_y - self.start_y)**2)
        
        if den == 0: return 0.0 # El inicio es igual a la meta
        return num / den

    def control_loop(self):
        if self.start_x is None:
            return # Esperar a tener odometría

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
        #           MÁQUINA DE ESTADOS BUG 2
        # ==========================================
        
        if self.state == 'GO_TO_GOAL':
            if self.obstacle_front:
                self.get_logger().warning("🛑 Obstáculo. Guardando Hit-Point y rodeando (Bug 2).")
                self.hit_dist_to_goal = dist_to_goal
                self.state = 'WALL_FOLLOW'
            else:
                if abs(angle_diff) > 0.2:
                    cmd.linear.x = 0.1
                    cmd.angular.z = 0.8 if angle_diff > 0 else -0.8
                else:
                    cmd.linear.x = 0.25 
                    cmd.angular.z = 0.0

        elif self.state == 'WALL_FOLLOW':
            dist_to_line = self.dist_to_m_line()
            
            # CONDICIÓN DE SALIDA BUG 2: 
            # 1. El robot cruza la Línea-M original (margen de 15cm)
            # 2. Está más cerca de la meta que cuando chocó inicialmente (margen de 25cm para no salir en el mismo punto)
            if dist_to_line < 0.15 and dist_to_goal < (self.hit_dist_to_goal - 0.25):
                self.get_logger().info("🟢 ¡Línea-M cruzada! Más cerca de la meta. Abandono la pared.")
                self.state = 'GO_TO_GOAL'
            else:
                # Bordeamos el obstáculo
                cmd = self.wall_follow()

        self.cmd_pub.publish(cmd)

    def wall_follow(self):
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
    node = Bug2Navigator()
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