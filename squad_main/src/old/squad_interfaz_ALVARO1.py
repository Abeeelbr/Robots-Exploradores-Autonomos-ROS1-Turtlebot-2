#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TurtleBot State Manager

Este script integra una interfaz gráfica de usuario (GUI) construida con Tkinter para permitir la interacción
manual y visualización del estado actual del robot y la imagen de la cámara.

Dependencias:
    - rospy
    - smach
    - smach_ros
    - actionlib
    - std_msgs.msg
    - squad_exploracion.msg
    - sensor_msgs.msg
    - cv_bridge
    - PIL (Pillow)
    - cv2 (OpenCV)
    - move_base_msgs.msg
    - geometry_msgs.msg
    - threading
    - tkinter
"""

import rospy
import smach
import smach_ros
from std_msgs.msg import String
import actionlib
from squad_exploracion.msg import ObjectDetectionAction, ObjectDetectionGoal, ObjectDetectionFeedback
from squad_exploracion.msg import AutonomousControlAction, AutonomousControlGoal, AutonomousControlResult
from squad_exploracion.msg import ApproachControlAction, ApproachControlFeedback, ApproachControlResult,ApproachControlGoal
from squad_exploracion.msg import DetectedObject
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image, ImageTk
import cv2
import signal
import sys
from squad_planificacion.msg import StationInfo, StationsInfo



from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import threading
import tkinter as tk

almacen_estaciones = {"HOME": (0, 0)}

class InterfazManager:
    """
    Gestor de la Interfaz Gráfica de Usuario (GUI) para la Máquina de Estados del TurtleBot.

    Esta clase maneja la creación y actualización de la interfaz gráfica, la interacción con
    los tópicos de ROS para enviar comandos y recibir actualizaciones del estado del robot.
    También gestiona la visualización de imágenes de la cámara en tiempo real, cómo se va construyendo el mapa del entorno y la gestión dinámica
    de estaciones.

    Atributos:
        state_machine (StateMachine): Máquina de estados finita (SMACH) para gestionar los estados del robot.
        state_names (list): Lista de nombres de los estados definidos en la máquina de estados.
        current_state (str): Nombre del estado actual del robot.
        command_state_mapping (dict): Mapeo de comandos a nombres de estados.
        cmd_vel_pub (Publisher): Publicador al tópico '/cmd_vel' para enviar comandos de movimiento.
        state_pub (Publisher): Publicador al tópico '/command' para enviar comandos de estado.
        ejecucion_mode (str): Modo de ejecución actual ('REAL', 'GAZEBO' o 'STAGE').
        control_teleop_topic (str): Tópico de control de teleoperación que depende del modo de ejecución.
        control_mode_pub (Publisher): Publicador al tópico '/control_mode' para cambiar el modo de control.
        control_mode_sub (Subscriber): Suscriptor al tópico '/control_mode' para recibir actualizaciones del modo de control.
        bridge (CvBridge): Objeto para convertir mensajes de ROS Image a OpenCV.
        camera_sub (Subscriber): Suscriptor al tópico de la cámara.
        normal_camera_topic (str): Tópico de la cámara normal.
        action_camera_topic (str): Tópico de la cámara procesada para acciones.
        current_camera_topic (str): Tópico de la cámara actualmente suscrito.
        camera_image (ImageTk.PhotoImage): Imagen actual de la cámara para visualizar en la GUI.
        control_mode (str): Modo de control actual ('manual' o 'autonomous').
        window (Tk): Ventana principal de la GUI.
        state_buttons (dict): Diccionario de botones de estado en la GUI.
        exploration_controls_frame (Frame): Marco para los controles de exploración en la GUI.
        station_selection_frame (Frame): Marco para la selección de estaciones.
        station_label (Label): Etiqueta para el desplegable de estaciones.
        selected_station (StringVar): Variable para almacenar la estación seleccionada.
        station_dropdown (OptionMenu): Desplegable para seleccionar estaciones.
        go_button (Button): Botón para enviar el comando "Ir" a la estación seleccionada.
    """

    def __init__(self):
        """
        Inicializa el gestor de la interfaz gráfica, configurando publicadores, suscriptores,
        la GUI y los controles de exploración.

        Args:
            state_machine (StateMachine): Máquina de estados finita (SMACH) para gestionar los estados del robot.
        """
        rospy.init_node('intrefaz', anonymous=True)
        rospy.loginfo('Nodo interfaz inicializado')

        self.state_names = ['REPOSO', 'EXPLORACION', 'NAVEGACION']
        self.current_state = None

        # ************************************************
        #          MAPEADO DE COMANDOS A ESTADOS
        # ************************************************
        self.command_state_mapping = {
            'parar': 'REPOSO',
            'modo exploracion': 'EXPLORACION',
            'ir a': 'NAVEGACION',
            'acercarse_objetivo': 'APPROACH'
        }

        # ************************************************
        #       INICIALIZACIÓN DE PUBLICADORES Y SUSCRIPTORES
        # ************************************************
        # Publicador para comandos de movimiento
        self.ejecucion_mode = rospy.get_param('MODO_EJECUCION', 'REAL')
        self.control_teleop_topic = rospy.get_param(f'/{self.ejecucion_mode}/control_teleop_topic', '/cmd_vel')
        self.cmd_vel_pub = rospy.Publisher(self.control_teleop_topic, Twist, queue_size=1)

        # Publicador para enviar comandos de estado
        self.state_pub = rospy.Publisher('/command', String, queue_size=1)

        # Publicador y suscriptor para el modo de control
        self.control_mode_pub = rospy.Publisher('/control_mode', String, queue_size=1)
        self.control_mode_sub = rospy.Subscriber('/control_mode', String, self.control_mode_callback)

        # Suscriptor para obtener el estado actual del robot
        rospy.Subscriber('/current_state', String, self.current_state_callback)

        # Suscriptor para la cámara
        self.bridge = CvBridge()
        self.camera_sub = None
        self.normal_camera_topic = rospy.get_param('/image_topic', '/camera')
        self.action_camera_topic = rospy.get_param('/processed_image_topic', '/processed_image')
        self.map_topic = rospy.get_param('/TOPIC_MAPA', '/map')
        self.current_camera_topic = self.normal_camera_topic
        self.camera_image = None

        # Suscriptor para estaciones
        rospy.Subscriber('/stations_info', StationsInfo, self.stations_callback)  # Ajusta el nombre del tópico si es necesario

        # ************************************************
        #              MODOS DE EXPLORACIÓN
        # ************************************************
        self.control_mode = 'manual'

        # ************************************************
        #           INICIALIZACIÓN DE LA INTERFAZ
        # ************************************************
        # Configurar la interfaz gráfica
        self.setup_gui()

        # Iniciar la suscripción a la cámara
        self.subscribe_to_camera(self.current_camera_topic)

        # Iniciar la suscripción al mapa
        #self.subscribe_to_camera(self.map_topic)

        # Iniciar el spinning de ROS en un hilo separado
        threading.Thread(target=rospy.spin).start()

        # Manejar el evento de cierre de la ventana
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Manejar la señal Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # Iniciar el spinning de ROS en un hilo separado
        self.ros_thread = threading.Thread(target=rospy.spin)
        self.ros_thread.start()

        # Iniciar el bucle principal de la GUI
        self.window.mainloop()

    def stations_callback(self, msg):
        """
        Callback para el suscriptor del tópico de estaciones.

        Actualiza la variable global 'almacen_estaciones' con los datos recibidos.

        Args:
            msg (Stations): Mensaje con los datos de estaciones.
        """
        global almacen_estaciones
        try:
            estaciones = almacen_estaciones
            for station in msg.stations:
                station_id = station.station_id
                coordinates = (station.coordinates.x, station.coordinates.y)
                # Puedes incluir el polígono si es necesario
                # polygon = [(point.x, point.y) for point in station.polygon]
                estaciones[station_id] = coordinates
            almacen_estaciones = estaciones
            rospy.loginfo(f"Actualizado almacen_estaciones: {almacen_estaciones}")
        except AttributeError as e:
            rospy.logerr(f"Error en el acceso a atributos del mensaje de estaciones: {e}")
        except Exception as e:
            rospy.logerr(f"Error inesperado en stations_callback: {e}")

    def current_state_callback(self, msg):
        """
        Callback para el suscriptor del estado actual.

        Actualiza el estado actual y resalta el botón correspondiente en la GUI.

        Args:
            msg (String): Mensaje con el nombre del estado actual.
        """
        self.current_state = msg.data
        # Actualizar la GUI para resaltar el estado actual
        self.update_state_highlight()

    def setup_gui(self):
        """
        Configura la interfaz gráfica de usuario utilizando Tkinter.

        Crea la ventana principal, los botones de control, la visualización de la cámara
        y los controles de exploración.
        """
        self.window = tk.Tk()
        self.window.title("Interfaz de la Máquina de Estados")
        self.window.configure(bg='#f0f0f0')
        self.window.geometry("1200x800")  # Tamaño ajustable según necesidad

        # Título
        self.title_label = tk.Label(
            self.window,
            text="Interfaz Gráfica",
            font=('Arial', 14, 'bold'),
            bg='#f0f0f0'
        )
        self.title_label.pack(pady=10)

        # # Crear un marco principal para las tres columnas
        # self.main_frame = tk.Frame(self.window, bg='#f0f0f0')
        # self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)

        # # Columna Izquierda: Botones de Control
        # self.left_frame = tk.Frame(self.main_frame, bg='#e0e0e0', width=200)
        # self.left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # # Columna Central: Imagen de la Cámara
        # self.center_frame = tk.Frame(self.main_frame, bg='#d0d0d0')
        # self.center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)

        # # Columna Derecha: Secciones Futuras
        # self.right_frame = tk.Frame(self.main_frame, bg='#c0c0c0', width=200)
        # self.right_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10)



        # ** Marco superior **
        self.top_frame = tk.Frame(self.window, bg="#f0f0f0", height=20)
        self.top_frame.pack(fill=tk.X, padx=10, pady=10)  # Llena todo el ancho horizontalmente

        # ** Marco inferior **
        self.bottom_frame = tk.Frame(self.window, bg="#e0e0e0")
        self.bottom_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)  # Llena el espacio restante

        # Dentro del marco inferior, dividimos en dos sub-marcos
        # ** Marco izquierdo **
        self.left_frame = tk.Frame(self.bottom_frame, bg="#d0d0d0", width=400)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)  # Expande proporcionalmente en la mitad izquierda

        # ** Marco derecho **
        self.right_frame = tk.Frame(self.bottom_frame, bg="#c0c0c0", width=400)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)  # Expande proporcionalmente en la mitad derecha

        # Dentro del marco derecho, dividimos en dos sub-marcos
        # ** Marco superior derecho **
        self.right_top_frame = tk.Frame(self.right_frame, bg="#b0b0b0")
        self.right_top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)  # Ocupa espacio proporcionalmente hacia arriba

        # ** Marco inferior derecho **
        self.right_bottom_frame = tk.Frame(self.right_frame, bg="#a0a0a0", height=100)
        self.right_bottom_frame.pack(side=tk.BOTTOM, fill=tk.BOTH)  # Altura fija hacia abajo

        # ************************************************
        #              BOTONES DE CONTROL
        # ************************************************
        self.state_buttons = {}
        for state_name in self.state_names:
            command = self.get_command_for_state(state_name)
            initial_color = '#2196F3' if state_name == 'REPOSO' else '#4CAF50'

            button = tk.Button(
                self.top_frame,
                text=state_name,
                command=lambda c=command: self.on_state_button_click(c),
                width=8,  # Botones más pequeños
                height=1,  # Altura ajustada
                bg=initial_color,
                fg='white',
                font=('Arial', 11)
            )
            button.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)  # Distribuye equitativamente
            self.state_buttons[state_name] = button


        # ************************************************
        #                       MAPA
        # ************************************************




        # ************************************************
        #        CONTROLES DE EXPLORACIÓN
        # ************************************************
        self.exploration_controls_frame = tk.Frame(
            self.right_bottom_frame,
            bg='#d0d0d0',
            padx=20,
            pady=20
        )
        self.exploration_controls_frame.pack(pady=10)

        # Crear controles de exploración
        self.create_exploration_controls()

        # ************************************************
        #              SELECCIÓN DE ESTACIONES
        # ************************************************
        self.station_selection_frame = tk.Frame(
            self.right_bottom_frame,
            bg='#d0d0d0',
            padx=20,
            pady=20
        )
        self.station_selection_frame.pack(pady=10)

        self.station_label = tk.Label(
            self.station_selection_frame,
            text="Seleccionar Estación",
            font=('Arial', 14),
            bg='#d0d0d0'
        )
        self.station_label.pack(pady=5)

        self.selected_station = tk.StringVar()
        self.selected_station.set("Seleccionar...")  # Valor por defecto

        self.station_dropdown = tk.OptionMenu(
            self.station_selection_frame,
            self.selected_station,
            *almacen_estaciones.keys()
        )
        self.station_dropdown.pack(pady=5)

        self.go_button = tk.Button(
            self.station_selection_frame,
            text="Ir",
            command=self.go_to_station,
            width=10,
            height=1,
            bg='#2196F3',
            fg='white',
            font=('Arial', 12)
        )
        self.go_button.pack(pady=5)

        # Iniciar la actualización periódica de la lista de estaciones
        self.update_station_list()

        # ************************************************
        #             VISUALIZACIÓN DE LA CÁMARA
        # ************************************************
        self.camera_label = tk.Label(self.left_frame, bg='#d0d0d0')
        self.camera_label.pack(fill=tk.BOTH, expand=True)

    # Función para alternar el estado de los estados de exploración
    def set_switch_state(self, state):
        self.switch_state.set(state)
        if state == "manual":
            self.control_mode = 'autonomous'
            self.button1.config(bg="lightgray")
            self.button2.config(bg="white")
            self.control_mode_pub.publish('autonomous')
        else:
            self.control_mode = 'manual'
            self.button1.config(bg="white")
            self.button2.config(bg="lightgray")
            self.control_mode_pub.publish('manual')


    def create_exploration_controls(self):
        """
        Crea los controles de exploración en la interfaz gráfica.

        Incluye un botón para cambiar el modo de exploración y las instrucciones para el
        control manual mediante el teclado.
        """

        self.exploration_label = tk.Label(
            self.exploration_controls_frame,
            text="Controles de Exploración",
            font=('Arial', 14),
            bg='#d0d0d0'
        )
        self.exploration_label.pack(pady=10)

        # Crear variable para controlar el estado actual
        self.switch_state = tk.StringVar(value="manual")

        # Crear los dos botones
        self.button1 = tk.Button(
            self.exploration_controls_frame,
            text="Modo Manual",
            command=lambda: self.set_switch_state("manual"),
            bg="lightgray"
        )
        self.button1.pack(side="left", padx=5)

        self.button2 = tk.Button(
            self.exploration_controls_frame,
            text="Modo Automático",
            command=lambda: self.set_switch_state("automatico"),
            bg="white"
        )
        self.button2.pack(side="left", padx=5)

        # Vincular teclas de flecha para control manual
        self.window.bind('<Up>', self.on_arrow_up)
        self.window.bind('<Down>', self.on_arrow_down)
        self.window.bind('<Left>', self.on_arrow_left)
        self.window.bind('<Right>', self.on_arrow_right)
        self.window.bind('<KeyRelease>', self.on_key_release)

    def get_command_for_state(self, state_name):
        """
        Obtiene el comando correspondiente para un nombre de estado dado.

        Args:
            state_name (str): Nombre del estado.

        Returns:
            str or None: Comando asociado al estado o None si no hay mapeo.
        """
        for command, state in self.command_state_mapping.items():
            if state == state_name:
                return command
        return None

    def on_state_button_click(self, command):
        """
        Maneja el evento de clic en un botón de estado.

        Publica el comando correspondiente al tópico '/command' para cambiar el estado
        de la máquina de estados.

        Args:
            command (str): Comando a publicar.
        """
        if command:
            self.state_pub.publish(command)
            rospy.loginfo("Comando publicado: %s", command)
        else:
            rospy.logwarn("No hay comando mapeado para este estado.")

    def update_state_highlight(self):
        """
        Actualiza la interfaz gráfica para resaltar el botón correspondiente al estado actual
        y mostrar u ocultar los controles de exploración según sea necesario.
        """
        rospy.loginfo(f"Estado actual: {self.current_state}")
        for state_name, button in self.state_buttons.items():
            if state_name == self.current_state:
                button.config(bg='#2196F3')
            else:
                button.config(bg='#4CAF50')

        # Mostrar u ocultar controles de exploración según el estado actual
        if self.current_state in ["EXPLORACION", "APPROACH"]:
            self.exploration_controls_frame.pack(padx=20, pady=20)
            # Cambiar la suscripción de la cámara si es necesario
            self.switch_camera_topic(self.action_camera_topic)
        else:
            self.exploration_controls_frame.pack_forget()
            # Cambiar la suscripción de la cámara al tópico normal
            self.switch_camera_topic(self.normal_camera_topic)

    def go_to_station(self):
        """
        Publica el comando "ir a {estacion}" al tópico '/command' para navegar hacia la estación seleccionada.
        """
        station = self.selected_station.get()
        if station in almacen_estaciones:
            command = f"ir a {station}"
            self.state_pub.publish(command)
            rospy.loginfo(f"Comando publicado: {command}")
        else:
            rospy.logwarn(f"Estación seleccionada '{station}' no es válida.")

    def update_station_list(self):
        """
        Actualiza las opciones del desplegable de estaciones según el contenido de 'almacen_estaciones'.
        Esta función se llama periódicamente para reflejar cualquier cambio en las estaciones.
        """
        # Limpiar las opciones actuales del desplegable
        menu = self.station_dropdown["menu"]
        menu.delete(0, "end")
        # Añadir las estaciones actuales al desplegable
        for station in almacen_estaciones.keys():
            menu.add_command(label=station, command=lambda value=station: self.selected_station.set(value))
        # Si la estación seleccionada ya no existe, restablecer la selección
        if self.selected_station.get() not in almacen_estaciones:
            self.selected_station.set("Seleccionar...")
        # Programar la siguiente actualización después de 1 segundo (1000 ms)
        self.window.after(1000, self.update_station_list)

    def on_arrow_up(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia arriba para mover el robot hacia adelante.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.linear.x = 0.5  # Ajusta la velocidad según sea necesario
        self.cmd_vel_pub.publish(twist)

    def on_arrow_down(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia abajo para mover el robot hacia atrás.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.linear.x = -0.5
        self.cmd_vel_pub.publish(twist)

    def on_arrow_left(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia la izquierda para girar el robot a la izquierda.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def on_arrow_right(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia la derecha para girar el robot a la derecha.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

    def on_key_release(self, event):
        """
        Maneja el evento de liberación de cualquier tecla para detener el movimiento del robot.

        Args:
            event: Evento de la tecla liberada.
        """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def control_mode_callback(self, msg):
        """
        Callback para el suscriptor del modo de control.

        Actualiza el modo de control actual y realiza acciones adicionales si es necesario.

        Args:
            msg (String): Mensaje con el modo de control ('manual' o 'autonomous').
        """
        self.control_mode = msg.data

    def change_exploration_mode(self):
        """
        Cambia el modo de exploración entre 'manual' y 'autonomous'.

        Actualiza la GUI y publica el nuevo modo de control al tópico '/control_mode'.
        """
        # Lógica para cambiar el modo de exploración
        if self.control_mode == 'manual':
            self.control_mode = 'autonomous'
            self.change_exploration_button.config(text="Cambiar a Modo Manual", bg='#2196F3')
            self.instructions_label.config(text="Control Autónomo Activado")
            self.control_mode_pub.publish('autonomous')
        else:
            self.control_mode = 'manual'
            self.change_exploration_button.config(text="Cambiar a Modo Autónomo", bg='#4CAF50')
            self.instructions_label.config(text="Usa las flechas para mover el robot")
            self.control_mode_pub.publish('manual')

    # ************************************************
    #                SUSCRIPCIÓN A LA CÁMARA
    # ************************************************

    def subscribe_to_camera(self, topic):
        """
        Suscribe al tópico de la cámara especificado.

        Desuscribe del tópico de cámara anterior si existe y suscribe al nuevo tópico.

        Args:
            topic (str): Tópico de la cámara al que suscribirse.
        """
        if self.camera_sub:
            self.camera_sub.unregister()

        self.current_camera_topic = topic
        self.camera_sub = rospy.Subscriber(topic, ROSImage, self.camera_callback)

    def switch_camera_topic(self, new_topic):
        """
        Cambia el tópico de la cámara a uno nuevo si es diferente al actual.

        Args:
            new_topic (str): Nuevo tópico de la cámara al que suscribirse.
        """
        if self.current_camera_topic != new_topic:
            self.subscribe_to_camera(new_topic)

    def camera_callback(self, msg):
        """
        Callback para el suscriptor de la cámara.

        Convierte la imagen recibida de ROS a un formato compatible con Tkinter y la
        muestra en la GUI.

        Args:
            msg (ROSImage): Mensaje de imagen recibido del tópico de la cámara.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convertir la imagen de BGR a RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Convertir la imagen a PIL
            pil_image = Image.fromarray(cv_image)
            # Redimensionar la imagen para ajustarla al tamaño del label
            pil_image = pil_image.resize((600, 400), Image.LANCZOS)
            # Convertir la imagen a ImageTk
            tk_image = ImageTk.PhotoImage(image=pil_image)
            # Actualizar la imagen en el label
            self.camera_label.configure(image=tk_image)
            self.camera_label.image = tk_image
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen de la cámara: {e}")
        except Exception as e:
            rospy.logerr(f"Error inesperado en camera_callback: {e}")

    # ************************************************
    #            GESTION CIERRE PROGRAMA
    # ************************************************

    def on_closing(self):
        rospy.loginfo("Cerrando la interfaz gráfica.")
        rospy.signal_shutdown("Interfaz cerrada por el usuario.")
        self.window.destroy()
        # Asegurarse de que los hilos se detengan
        if self.ros_thread.is_alive():
            self.ros_thread.join()
        sys.exit()

    def signal_handler(self, sig, frame):
        rospy.loginfo("Señal de interrupción recibida. Cerrando la interfaz.")
        self.window.quit()
        rospy.signal_shutdown("Interfaz cerrada por señal de interrupción.")
        # Asegurarse de que los hilos se detengan
        if self.ros_thread.is_alive():
            self.ros_thread.join()
        sys.exit()


if __name__ == '__main__':
    try:
        manager = InterfazManager()
    except rospy.ROSInterruptException:
        pass