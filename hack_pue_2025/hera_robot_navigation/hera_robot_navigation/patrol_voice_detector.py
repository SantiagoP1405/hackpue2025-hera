import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf_transformations
import numpy as np
import time

class PatrolVoiceDetector(Node):
    def __init__(self):
        super().__init__('patrol_voice_detector')
        
        # Navegador
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Publisher para reportar actividad (mantener compatibilidad)
        self.activity_publisher = self.create_publisher(
            String,
            '/classroom_activity',
            10
        )
        
        # Publisher para simular que está "escuchando"
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Puntos de patrullaje en el aula
        self.patrol_points = [
            # Punto 1 - Esquina superior derecha
            {"name": "Zona_A", "x": 9.132625579833984, "y": -0.07695972919464111, "yaw": 0.0},
            
            # Punto 2 - Esquina inferior derecha  
            {"name": "Zona_B", "x": 9.182291030883789, "y": -5.916730880737305, "yaw": -np.pi/2},
            
            # Punto 3 - Centro-derecha inferior
            {"name": "Zona_C", "x": 5.571091175079346, "y": -5.538115501403809, "yaw": np.pi},
            
            # Punto 4 - Centro-izquierda superior (CORREGIDO: X positiva)
            {"name": "Zona_D", "x": 5.482376575469971, "y": -0.5175471901893616, "yaw": 3*np.pi/4},
            
            # Punto 5 - Zona izquierda superior
            {"name": "Zona_E", "x": 2.3139398097991943, "y": -0.45159393548965454, "yaw": np.pi},
            
            # Punto 6 - Zona izquierda inferior
            {"name": "Zona_F", "x": 2.0158159732818604, "y": -5.569348335266113, "yaw": -np.pi/2},
            
            # Punto 7 - Zona derecha inferior (cerca de Zona_B)
            {"name": "Zona_G", "x": 9.23240852355957, "y": -5.762628555297852, "yaw": 0.0},
            
            # Punto 8 - Zona derecha superior (cerca de Zona_A)
            {"name": "Zona_H", "x": 8.887739181518555, "y": -0.05140864849090576, "yaw": 3*np.pi/4},
            
            # Punto 9 - Centro superior
            {"name": "Zona_I", "x": 5.555480003356934, "y": -0.8591190576553345, "yaw": np.pi},
            
            # Punto 10 - Centro inferior
            {"name": "Zona_J", "x": 5.1709113121032715, "y": -5.740340232849121, "yaw": -np.pi/2},
            
            # Punto 11 - Zona izquierda inferior (cerca de Zona_F)
            {"name": "Zona_K", "x": 1.9948129653930664, "y": -5.362565994262695, "yaw": np.pi},

            # Punto 12 - Zona izquierda inferior
            {"name": "Zona_L", "x": 2.3139398097991943, "y": -0.45159393548965454, "yaw": 3*np.pi/4},
        ]
        
        # Variables de estado
        self.current_patrol_index = -1
        self.listening_time = 10.0  # segundos "escuchando" en cada punto
        self.current_zone = "Desconocida"
        
        # Iniciar patrullaje
        self.get_logger().info("🤖 Robot Asistente de Aula iniciado (Modo Demo - Sin Audio)")
        self.get_logger().info(f"📍 Puntos de patrullaje: {len(self.patrol_points)}")
        
        # Timer para controlar el patrullaje
        self.patrol_timer = self.create_timer(1.0, self.patrol_state_machine)
        
        # Estados del robot
        self.state = "MOVING"  # MOVING, LISTENING, ANALYZING
        self.listen_start_time = None

    def create_pose_from_point(self, point):
        """Crear PoseStamped desde un punto de patrullaje"""
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, point["yaw"])
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"] 
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        
        return pose

    def patrol_state_machine(self):
        """Máquina de estados para el patrullaje"""
        
        if self.state == "MOVING":
            # Verificar si llegó al destino
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                current_point = self.patrol_points[self.current_patrol_index]
                
                if result:
                    self.get_logger().info(f"✅ Llegué a {current_point['name']}")
                    self.current_zone = current_point['name']
                    self.state = "LISTENING"
                    self.listen_start_time = time.time()
                    
                    # Publicar que llegó a la zona
                    status_msg = String()
                    status_msg.data = f"ROBOT_EN_ZONA: {self.current_zone}"
                    self.status_publisher.publish(status_msg)
                    
                else:
                    self.get_logger().warn(f"❌ No pude llegar a {current_point['name']}")
                    self.move_to_next_point()
        
        elif self.state == "LISTENING":
            # "Escuchar" por un tiempo determinado (solo simular)
            elapsed_time = time.time() - self.listen_start_time
            remaining_time = self.listening_time - elapsed_time
            
            if remaining_time > 0:
                if int(remaining_time) % 3 == 0:  # Log cada 3 segundos
                    self.get_logger().info(f"🎧 Simulando escucha en {self.current_zone} - {remaining_time:.0f}s restantes")
                    
                    # Publicar estado de escucha
                    status_msg = String()
                    status_msg.data = f"ROBOT_ESCUCHANDO: {self.current_zone} - {remaining_time:.0f}s restantes"
                    self.status_publisher.publish(status_msg)
                    
            else:
                self.state = "ANALYZING"
        
        elif self.state == "ANALYZING":
            # Simular análisis
            self.simulate_zone_analysis()
            self.move_to_next_point()

    def simulate_zone_analysis(self):
        """Simular análisis de la zona (sin audio real)"""
        self.get_logger().info(f"📊 {self.current_zone}: Análisis de zona completado (modo demo)")
        
        # Publicar que terminó el análisis
        activity_msg = String()
        activity_msg.data = f"ANALISIS_COMPLETADO: {self.current_zone}"
        self.activity_publisher.publish(activity_msg)

    def move_to_next_point(self):
        """Moverse al siguiente punto de patrullaje"""
        self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
        next_point = self.patrol_points[self.current_patrol_index]
        
        self.get_logger().info(f"🚀 Navegando hacia {next_point['name']} ({next_point['x']}, {next_point['y']})")
        
        # Publicar próximo destino
        status_msg = String()
        status_msg.data = f"NAVEGANDO_A: {next_point['name']}"
        self.status_publisher.publish(status_msg)
        
        pose = self.create_pose_from_point(next_point)
        self.navigator.goToPose(pose)
        self.state = "MOVING"

    def start_patrol(self):
        """Iniciar el patrullaje"""
        self.get_logger().info("🔄 Iniciando patrullaje del aula (Modo Demo)...")
        
        # Publicar inicio de patrullaje
        status_msg = String()
        status_msg.data = "PATRULLAJE_INICIADO"
        self.status_publisher.publish(status_msg)
        
        self.move_to_next_point()

def main():
    rclpy.init()
    
    try:
        node = PatrolVoiceDetector()
        
        # Esperar un poco antes de iniciar
        time.sleep(2)
        node.start_patrol()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()