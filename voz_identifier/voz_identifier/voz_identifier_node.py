import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import torch
import torchaudio
import os
from speechbrain.inference.speaker import SpeakerRecognition
import tempfile

class VozIdentifier(Node):
    def __init__(self):
        super().__init__('voz_identifier_node')
        
        # Configuración
        self.referencias_dir = os.path.expanduser("~/.voz_identifier_refs/")
        self.fs = 16000
        self.duracion = 3  # segundos
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'voz_detectada', 10)
        
        # Inicializar modelo
        self.get_logger().info("Cargando modelo de reconocimiento de hablante...")
        try:
            self.model = SpeakerRecognition.from_hparams(
                source="speechbrain/spkrec-ecapa-voxceleb",
                savedir="./speechbrain_cache"
            )
            self.get_logger().info("Modelo cargado exitosamente")
        except Exception as e:
            self.get_logger().error(f"Error cargando modelo: {e}")
            return
        
        # Cargar embeddings de referencia
        self.embeddings_ref = self.cargar_embeddings()
        
        if not self.embeddings_ref:
            self.get_logger().warn("No se encontraron referencias de voz. Ejecuta primero grabador_referencia")
            return
        
        # Timer para detectar voz cada 6 segundos
        self.timer = self.create_timer(6.0, self.detectar_voz)
        self.get_logger().info("Nodo iniciado. Detectando voces cada 6 segundos...")

    def cargar_embeddings(self):
        """Cargar embeddings de las voces de referencia"""
        embeddings = {}
        
        if not os.path.exists(self.referencias_dir):
            self.get_logger().warn(f"Directorio de referencias no existe: {self.referencias_dir}")
            return embeddings
        
        for archivo in os.listdir(self.referencias_dir):
            if archivo.endswith(".wav"):
                nombre = archivo.replace(".wav", "")
                ruta_archivo = os.path.join(self.referencias_dir, archivo)
                
                try:
                    # Cargar audio y obtener embedding
                    audio = self.model.load_audio(ruta_archivo)
                    embedding = self.model.encode_batch(audio.unsqueeze(0))
                    embeddings[nombre] = embedding
                    self.get_logger().info(f"Cargado embedding para: {nombre}")
                except Exception as e:
                    self.get_logger().error(f"Error cargando {archivo}: {e}")
        
        return embeddings

    def detectar_voz(self):
        """Detectar y clasificar voz en tiempo real"""
        try:
            self.get_logger().info("Grabando audio...")
            
            # Grabar audio
            audio_np = sd.rec(
                int(self.duracion * self.fs), 
                samplerate=self.fs, 
                channels=1, 
                dtype='float32'
            )
            sd.wait()
            
            # Convertir a tensor PyTorch
            audio_tensor = torch.from_numpy(audio_np[:, 0])
            
            # Guardar temporalmente
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                torchaudio.save(tmp_file.name, audio_tensor.unsqueeze(0), self.fs)
                
                # Obtener embedding del audio de prueba
                test_audio = self.model.load_audio(tmp_file.name)
                test_embedding = self.model.encode_batch(test_audio.unsqueeze(0))
                
                # Comparar con referencias
                similitudes = {}
                for nombre, ref_embedding in self.embeddings_ref.items():
                    # Calcular similitud coseno
                    similarity = torch.cosine_similarity(
                        test_embedding.squeeze(), 
                        ref_embedding.squeeze(), 
                        dim=0
                    )
                    similitudes[nombre] = similarity.item()
                
                # Limpiar archivo temporal
                os.unlink(tmp_file.name)
            
            # Encontrar la mayor similitud
            if similitudes:
                identificado = max(similitudes, key=similitudes.get)
                max_similitud = similitudes[identificado]
                
                # Solo publicar si la similitud es suficientemente alta
                umbral = 0.5  # Ajusta este valor según necesites
                if max_similitud > umbral:
                    msg = String()
                    msg.data = f"Persona {identificado} detectada (similitud: {max_similitud:.3f})"
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Voz detectada: {msg.data}")
                else:
                    self.get_logger().info(f"Voz no reconocida (similitud máxima: {max_similitud:.3f})")
            
        except Exception as e:
            self.get_logger().error(f"Error en detección de voz: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VozIdentifier()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()