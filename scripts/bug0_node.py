#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug0Navigator(Node):
    def __init__(self):
        super().__init__('bug0_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/tetrabot/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.create_timer(0.1, self.control_loop)

        # ====== NUEVA META ESTABLECIDA ======
        self.goal_x = 5.668214962097453
        self.goal_y = 3.3053792974265988
        # ====================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.obstacle_front = False
        self.obstacle_right = False
        self.state = 'GO_TO_GOAL'

        self.get_logger().info(f"🚀 Navegador Bug 0 Iniciado. Destino: X={self.goal_x}, Y={self.goal_y}")

    def odom_callback(self, msg):
        # Actualiza la posición y orientación basada en la odometría
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_callback(self, msg):
        ranges = msg.ranges
        
        # Filtro para ignorar lecturas basura (infinitos o el propio chasis del robot)
        def get_min(start, end):
            valid_ranges = [r for r in ranges[start:end] if 0.15 < r < 10.0]
            return min(valid_ranges) if valid_ranges else 10.0

        # VISIÓN PERIFÉRICA CORREGIDA (Hacia adelante: índices 135 a 225)
        dist_front = get_min(135, 225)
        
        # VISIÓN LATERAL DERECHA REAL (Índices 45 a 135)
        dist_right = get_min(45, 135)

        # PARÁMETROS DE REACCIÓN TEMPRANA
        self.obstacle_front = dist_front < 0.85  # Empieza a evadir a 85cm
        self.obstacle_right = dist_right < 0.6   # Mantiene la pared a 60cm

    def control_loop(self):
        cmd = Twist()
        
        # Cálculos hacia la meta
        dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        
        # Normalización del ángulo (-pi a pi)
        angle_diff = angle_to_goal - self.current_yaw
        while angle_diff > math.pi: angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi

        # 1. CONDICIÓN DE LLEGADA
        if dist_to_goal < 0.3:
            self.get_logger().info("✅ ¡LLEGAMOS A LA META!", once=True)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 2. MÁQUINA DE ESTADOS (BUG 0)
        if self.state == 'GO_TO_GOAL':
            if self.obstacle_front:
                self.get_logger().warning("🛑 Obstáculo detectado. Iniciando rodeo (Wall Follow)...")
                self.state = 'WALL_FOLLOW'
            else:
                # Se alinea hacia la meta antes de avanzar
                if abs(angle_diff) > 0.2:
                    cmd.linear.x = 0.1
                    cmd.angular.z = 0.8 if angle_diff > 0 else -0.8
                else:
                    # Avanza seguro
                    cmd.linear.x = 0.25 
                    cmd.angular.z = 0.0
                    
        elif self.state == 'WALL_FOLLOW':
            # CONDICIÓN DE SALIDA: El frente está libre Y apuntamos casi perfecto a la meta
            if not self.obstacle_front and abs(angle_diff) < 0.3:
                self.get_logger().info("🟢 Camino a la meta despejado. Despegando de la pared...")
                self.state = 'GO_TO_GOAL'
            else:
                # LÓGICA DE SEGUIMIENTO DE PARED (Bordeando por la derecha)
                if self.obstacle_front:
                    # Si hay muro enfrente, se detiene y gira rápido a la izquierda sobre su eje
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.8 
                elif self.obstacle_right:
                    # Si la pared está a su derecha, avanza en paralelo
                    cmd.linear.x = 0.25  
                    cmd.angular.z = 0.0
                else:
                    # Si pierde la pared de la derecha, gira suavemente para volver a encontrarla
                    cmd.linear.x = 0.15 
                    cmd.angular.z = -0.6

        # Publicar los comandos de velocidad calculados
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = Bug0Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Freno de emergencia al cerrar el nodo con Ctrl+C
        node.cmd_pub.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()