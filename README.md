# hackpue2025-hera
HACKPUE 2025 - HORIZON'S H.E.R.A (Horizons' Educational Robotic Assistant)

Contiene los recursos y código necesarios para la simulación y navegación del robot H.E.R.A en ROS 2. 
H.E.R.A. es un robot móvil simulado en ROS 2 diseñado para tareas de navegación autónoma y detección de voz (mediante palabras específicas). El robot es parte del Plan para Instituciones Educativas de Horizons (H.E.I.P). Se encargará de navegar en un salón de clases (previamente mapeado) de manera autónoma, mientras escucha las participaciones de los alumnos de clase, al escuchar palabras clave relacionadas con la disciplina que a cada alumno le guste, le interese, e incluso se le dificulte, de forma individual.  Su tarea es hacer un recorrido previamente definido por el salón de clases, mientras escucha a los alumnos. No esuchará todas las conversaciones, solamente lo relacionado a la clase. Una vez que escucha, recopila la información para cada estudiante y la almacena en la base de datos. El profesor podrá utilziar dicha información para asesorar a cada estudiante sobre su elección de carrera. 

El impacto positivo de este sistema se refleja en varios aspectos:
- Personalización del aprendizaje: H.E.R.A. recopila información sobre los intereses, dificultades y preferencias académicas de cada alumno, permitiendo al profesor adaptar la orientación y el acompañamiento a las necesidades individuales.
- Fomento de la participación estudiantil: Al reconocer y registrar las intervenciones relacionadas con el tema de la clase, el robot estimula la participación activa y ordenada, sin distraerse con conversaciones ajenas al contexto académico.
- Apoyo a la toma de decisiones: La información almacenada en la base de datos ofrece al docente un panorama más claro para orientar a los estudiantes en la elección de su carrera o área de especialización, fortaleciendo el vínculo entre sus intereses y su desarrollo profesional.
- Innovación educativa: La implementación de un robot autónomo con detección de voz en el aula introduce un recurso tecnológico novedoso que promueve la interacción entre estudiantes, docentes y sistemas inteligentes, acercando la educación al futuro de la automatización y la IA.

A continuación se describe la estructura y el propósito de cada componente:

## Estructura de carpetas

- **hera_robot_description.zip**  
  Archivo comprimido con la descripción del robot HERA.

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

### hera_voice_identifier/
Implementa la identificación de voz para el robot HERA.

- **package.xml, LICENSE, setup.cfg, setup.py**: Archivos estándar de ROS 2 Python.
- **hera_voice_identifier/**: Código fuente principal:
  - `grabador_referencia.py`: Nodo para grabar referencia de voz.
  - `voz_identifier_node.py`: Nodo para identificar la voz.
- **resource/**: Recursos del paquete.
- **test/**: Pruebas unitarias y de estilo.

---

## Uso

1. **Instalación**: Clona el repositorio y compílalo con `colcon build`.
2. **Simulación**: Utiliza los archivos de lanzamiento en `hera_robot_bringup/launch` y `hera_robot_description/launch`.
3. **Navegación**: Ejecuta los nodos de navegación desde `hera_robot_navigation`.
4. **Identificación de voz**: Ejecuta los nodos desde `hera_voice_identifier`.

## Requisitos

- ROS 2 (Foxy, Humble, o superior)
- Gazebo
- Python 3.8+

## Créditos

Desarrollado por el equipo de hack_pue_2025.
