#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import os
import time

def speak(text):
    """Función para que el robot hable usando espeak"""
    print(f"🤖 Robot dice: {text}")
    # Instalación necesaria: sudo apt install espeak
    os.system(f'espeak -v es "{text}"')

def main():
    rclpy.init()
    nav = BasicNavigator()

    # --- Configuración de las Salas ---
    rooms = [
        {
            'name': 'Sala 1',
            'x': -5.274168140567955, 'y': 1.6410791274740495,
            'qz': 0.39906282392062925, 'qw': -0.9169235860007598
        },
        {
            'name': 'Sala de depósito',
            'x': 6.079612882957816, 'y': -4.391466473200874,
            'qz': -0.9066373487694229, 'qw': -0.42191079367130657
        },
        {
            'name': 'Sala de atención 2',
            'x': -3.982675852552293, 'y': 2.4858608729289147,
            'qz': 0.47691072057509853, 'qw': -0.8789517419065397
        },
        {
            'name': 'Sala de espera farmacia',
            'x': 12.37487906475744, 'y': -5.744170172265315,
            'qz': 0.8971317281648714, 'qw': 0.441763129199248
        },
        {
            'name': 'Sala de atención 3',
            'x': 5.96009412308221, 'y': 9.878888475937524,
            'qz': 0.46363006701734377, 'qw': -0.8860288714017694
        }
    ]

    # Esperar a que Nav2 esté completamente listo
    print("Esperando a que Nav2 se active...")
    nav.waitUntilNav2Active()

    for room in rooms:
        # Crear mensaje de posición
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = room['x']
        goal_pose.pose.position.y = room['y']
        goal_pose.pose.orientation.z = room['qz']
        goal_pose.pose.orientation.w = room['qw']

        print(f"🚀 Iniciando navegación hacia: {room['name']}")
        nav.goToPose(goal_pose)

        # Monitorear hasta llegar
        while not nav.isTaskComplete():
            time.sleep(1)

        # Verificar resultado
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            speak(f"He llegado a {room['name']}")
            time.sleep(2)  # Pausa de cortesía
        else:
            print(f"❌ No se pudo llegar a {room['name']}")

    print("🏁 Todas las posiciones han sido visitadas.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()