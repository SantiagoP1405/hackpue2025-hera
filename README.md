# hackpue2025-hera
HACKPUE 2025 - HORIZON'S H.E.R.A (Horizons' Educational Robotic Assistant)
#### DEMO VIDEO: 
https://youtu.be/szmfTx3GvAs

## ENG:
Contains the necessary resources and code for the simulation and navigation of the H.E.R.A. robot in ROS 2.
H.E.R.A. is a simulated mobile robot in ROS 2 designed for autonomous navigation and voice detection tasks (using specific words). The robot is part of the Horizons Educational Institutions Plan (H.E.I.P). It will be responsible for navigating a classroom (previously mapped) autonomously, while listening to the participation of the students in the class, by hearing keywords related to the discipline that each student likes, is interested in, and even finds difficult, on an individual basis. Its task is to follow a predefined route through the classroom while listening to the students. It will not listen to all conversations, only those related to the class. Once it listens, it collects the information for each student and stores it in the database. The teacher can use this information to advise each student on their career choice.

The positive impact of this system is reflected in several aspects:
- Personalized learning: H.E.R.A. collects information about the interests, difficulties, and academic preferences of each student, allowing the teacher to adapt guidance and support to individual needs.
- Fostering student participation: By recognizing and recording interventions related to the class topic, the robot encourages active and orderly participation, without being distracted by conversations outside the academic context.
- Support for decision-making: The information stored in the database offers the teacher a clearer picture to guide students in choosing their career or area of specialization, strengthening the link between their interests and their professional development.
- Educational innovation: The implementation of an autonomous robot with voice detection in the classroom introduces a novel technological resource that promotes interaction between students, teachers, and intelligent systems, bringing education closer to the future of automation and AI.

The structure and purpose of each component are described below:

## Folder Structure

### hera_robot_bringup/
Contains the necessary files to launch and configure the robot in simulation or on real hardware.
- **CMakeLists.txt, package.xml, LICENSE:** Standard ROS 2 files for package definition and licensing.
- **config/gz_bridge.yaml:** Configuration for the Gazebo-ROS bridge.
- **include/:** Header files (if applicable).
- **launch/gz_hera.launch.py:** Launch file to start the simulation in Gazebo.
- **maps/:** Environment maps, including:
- `classroom_map.pgm`, `classroom_map.yaml`: Classroom map for navigation.
- `.xcf` and `Zone.Identifier` files: Additional resources.
- **src/:** Additional source code (if applicable).
- **worlds/classroom.sdf:** Gazebo world for the simulation.

### hera_robot_description/
Contains the description of the HERA robot in URDF/XACRO format and related files.
- **CMakeLists.txt, package.xml, LICENSE:** Standard ROS 2 files.
- **include/:** Header files (if applicable).
- **launch/description.launch.py:** Launch file to load the robot's description.
- **rviz/config.rviz:** Configuration to visualize the robot in RViz.
- **src/:** Additional source code (if applicable).
- **urdf/:** XACRO/URDF files that define the robot's model: `camera.xacro`, `lidar.xacro`, `mobile_base.xacro`, etc.

### hera_robot_navigation/
Implements the autonomous navigation of the HERA robot.
- **package.xml, LICENSE, setup.cfg, setup.py:** Standard ROS 2 Python files.
- **hera_robot_navigation/:** Main source code:
  - `hera_nav2.py:` Main navigation node.
  - `patrol_voice_detector.py:` Node for patrol and voice detection.
  - `set_initial_pose.py:` Node to set the initial position.
- **resource/:** Package resources.
- **test/:** Unit and style tests

### voz_identifier/
Speaker identification by voice using SpeechBrain and ROS 2.
This package allows recording reference voices and then identifying people in real-time by their voice, publishing the result to a ROS 2 topic.

- [voz_identifier/grabador_referencia.py](hackpue2025-hera/voz_identifier/voz_identifier/grabador_referencia.py): Script to record reference voices and store them in ~/.voz_identifier_refs/.
- [voz_identifier/voz_identifier_node.py](hackpue2025-hera/voz_identifier/voz_identifier/voz_identifier_node.py): ROS 2 node that detects and identifies voices in real-time using SpeechBrain.
- [setup.py](hackpue2025-hera/voz_identifier/setup.py) y [setup.cfg](hackpue2025-hera/voz_identifier/setup.cfg): Configuration files to install the package.
- [package.xml](hackpue2025-hera/voz_identifier/package.xml): ROS 2 metadata and dependencies.

## Instalation
Compile the package with colcon:
```
colcon build --symlink-install --packages-select voz_identifier
source install/setup.bash
```

## ESP:
Contiene los recursos y código necesarios para la simulación y navegación del robot H.E.R.A en ROS 2. 
H.E.R.A. es un robot móvil simulado en ROS 2 diseñado para tareas de navegación autónoma y detección de voz (mediante palabras específicas). El robot es parte del Plan para Instituciones Educativas de Horizons (H.E.I.P). Se encargará de navegar en un salón de clases (previamente mapeado) de manera autónoma, mientras escucha las participaciones de los alumnos de clase, al escuchar palabras clave relacionadas con la disciplina que a cada alumno le guste, le interese, e incluso se le dificulte, de forma individual.  Su tarea es hacer un recorrido previamente definido por el salón de clases, mientras escucha a los alumnos. No esuchará todas las conversaciones, solamente lo relacionado a la clase. Una vez que escucha, recopila la información para cada estudiante y la almacena en la base de datos. El profesor podrá utilziar dicha información para asesorar a cada estudiante sobre su elección de carrera. 

El impacto positivo de este sistema se refleja en varios aspectos:
- Personalización del aprendizaje: H.E.R.A. recopila información sobre los intereses, dificultades y preferencias académicas de cada alumno, permitiendo al profesor adaptar la orientación y el acompañamiento a las necesidades individuales.
- Fomento de la participación estudiantil: Al reconocer y registrar las intervenciones relacionadas con el tema de la clase, el robot estimula la participación activa y ordenada, sin distraerse con conversaciones ajenas al contexto académico.
- Apoyo a la toma de decisiones: La información almacenada en la base de datos ofrece al docente un panorama más claro para orientar a los estudiantes en la elección de su carrera o área de especialización, fortaleciendo el vínculo entre sus intereses y su desarrollo profesional.
- Innovación educativa: La implementación de un robot autónomo con detección de voz en el aula introduce un recurso tecnológico novedoso que promueve la interacción entre estudiantes, docentes y sistemas inteligentes, acercando la educación al futuro de la automatización y la IA.

A continuación se describe la estructura y el propósito de cada componente:

## Estructura de carpetas

### hera_robot_bringup/
Contiene los archivos necesarios para lanzar y configurar el robot en simulación o hardware real.

- **CMakeLists.txt, package.xml, LICENSE**: Archivos estándar de ROS 2 para la definición y licencia del paquete.
- **config/gz_bridge.yaml**: Configuración para el puente Gazebo-ROS.
- **include/**: Archivos de cabecera (si aplica).
- **launch/gz_hera.launch.py**: Archivo de lanzamiento para iniciar la simulación en Gazebo.
- **maps/**: Mapas del entorno, incluyendo:
  - `classroom_map.pgm`, `classroom_map.yaml`: Mapa del aula para navegación.
  - Archivos `.xcf` y `Zone.Identifier`: Recursos adicionales.
- **src/**: Código fuente adicional (si aplica).
- **worlds/classroom.sdf**: Mundo de Gazebo para la simulación.

### hera_robot_description/
Contiene la descripción del robot HERA en formato URDF/XACRO y archivos relacionados.

- **CMakeLists.txt, package.xml, LICENSE**: Archivos estándar de ROS 2.
- **include/**: Archivos de cabecera (si aplica).
- **launch/description.launch.py**: Archivo de lanzamiento para cargar la descripción del robot.
- **rviz/config.rviz**: Configuración para visualizar el robot en RViz.
- **src/**: Código fuente adicional (si aplica).
- **urdf/**: Archivos XACRO/URDF que definen el modelo del robot:
  - `camera.xacro`, `lidar.xacro`, `mobile_base.xacro`, etc.

### hera_robot_navigation/
Implementa la navegación autónoma del robot HERA.

- **package.xml, LICENSE, setup.cfg, setup.py**: Archivos estándar de ROS 2 Python.
- **hera_robot_navigation/**: Código fuente principal:
  - `hera_nav2.py`: Nodo principal de navegación.
  - `patrol_voice_detector.py`: Nodo para patrulla y detección por voz.
  - `set_initial_pose.py`: Nodo para establecer la posición inicial.
- **resource/**: Recursos del paquete.
- **test/**: Pruebas unitarias y de estilo.

---

## Instalación
Se debe tener instalados los siguientes frameworks, considerando la distro **Humble** de ROS2:

- ROS2 Humble
- Ign Gazebo
- Nav2
- Slam
- Python 3.10
- Numpy
- Micrófono
- speechbrain
- Sound Device
- tf_transformations

##### ROS2 Humble: 
Seguir los pasos para la instalación:
* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
##### NAV2:
```
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```
##### Gazebo:
Seguir los pasos para la instalación:
* https://jkk-research.github.io/workshops/f1tenth_sim_a/

#### Pip:
```
sudo apt update
sudo apt install python3-pip
```

#### Numpy/Sounddevice:
```
pip install numpy sounddevice
```

#### Pytorch:
```
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cpu
```

#### Speechbrain:
```
pip install speechbrain
```

#### tf_transformations:
```
sudo apt install ros-humble-tf-transformations
```

(Instalación con *pip*):
```
pip install tf-transformations
```


---

## Ejecución
Crear la carpeta del proyecto:
```
cd
mkdir Workspace
cd Workspace
mkdir hera_proyect
cd hera_project
git clone https://github.com/SantiagoP1405/hackpue2025-hera.git
```
Una vez copiado este repositorio, ejecutar los siguientes comandos en su espacio de trabajo:
```
cd
cd Workspace/hera_proyect
colcon build --symlink-install --packages-select hera_robot_description hera_robot_bringup hera_robot_navigation hera_voice_identifier
source install/setup.bash
ros2 launch hera_robot_bringup gz_hera.launch.py 
```

### EXTRA: voice_identifier/
Identificación de personas por voz usando SpeechBrain y ROS 2.

Este paquete permite grabar voces de referencia y luego identificar en tiempo real a las personas por su voz, publicando el resultado en un tópico de ROS 2.
- [voz_identifier/grabador_referencia.py](hackpue2025-hera/voz_identifier/voz_identifier/grabador_referencia.py): Script para grabar voces de referencia y almacenarlas en ~/.voz_identifier_refs/.
- [voz_identifier/voz_identifier_node.py](hackpue2025-hera/voz_identifier/voz_identifier/voz_identifier_node.py): Nodo ROS 2 que detecta e identifica voces en tiempo real usando SpeechBrain.
- [setup.py](hackpue2025-hera/voz_identifier/setup.py) y [setup.cfg](hackpue2025-hera/voz_identifier/setup.cfg): Archivos de configuración para instalar el paquete.
- [package.xml](hackpue2025-hera/voz_identifier/package.xml): Metadatos y dependencias ROS 2.


En el mismo espacio de trabajo, compilar el paquete con colcon:

```
colcon build --symlink-install --packages-select voice_identifier
source install/setup.bash
```

* Uso

1. *Graba las voces de referencia*  
   Ejecuta el grabador y sigue las instrucciones:
   ```
   ros2 run voz_identifier grabador_referencia
   ```
   Esto guardará archivos .wav en ~/.voz_identifier_refs/.

2. *Ejecuta el nodo de identificación*  
   Inicia el nodo que detecta y publica la voz identificada:
   ```
   ros2 run voz_identifier voz_identifier_node
   ```
   El resultado se publica en el tópico voz_detectada (tipo std_msgs/String).
