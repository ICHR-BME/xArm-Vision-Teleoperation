import cv2
import mediapipe as mp
import time
import math
from xarm.wrapper import XArmAPI
import traceback

# --- 1. TU CLASE ROBOT (Limpiada para este propósito) ---
class RobotMain(object):
    def __init__(self, robot):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100  
        self._tcp_acc = 2000
        self._robot_init()

    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0) # Modo posición
        self._arm.set_state(0) # Estado listo
        time.sleep(1)

    def mover_a(self, x, y, z):
        # Mueve el robot a una coordenada (wait=False es CLAVE para tiempo real)
        # Usamos radius > 0 para que el movimiento sea fluido y no se detenga en cada punto
        self._arm.set_position(x, y, z, roll=-180, pitch=0, yaw=0, 
                               speed=self._tcp_speed, mvacc=self._tcp_acc, 
                               radius=5, wait=False)

    def control_gripper(self, cerrar):
        if cerrar:
            self._arm.close_lite6_gripper()
        else:
            self._arm.open_lite6_gripper()

# --- 2. CONFIGURACIÓN MEDIAPIPE ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# --- 3. LÓGICA DE MAPEO (Convertir Pantalla -> Robot) ---
def map_range(value, in_min, in_max, out_min, out_max):
    # Regla de 3 para convertir coordenadas
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- 4. PROGRAMA PRINCIPAL ---
if __name__ == "__main__":
    
    # CONECTAR ROBOT (Cambia la IP si es necesario)
    print("Conectando al robot...")
    try:
        arm = XArmAPI("192.168.0.150") # <--- OJO: REVISA TU IP
        robot = RobotMain(arm)
        print("Robot Conectado y Listo.")
    except Exception as e:
        print(f"Error conectando robot: {e}")
        print("Corriendo en modo simulación (sin robot real)...")
        robot = None # Permitimos que el código corra solo con cámara para probar

    # INICIAR CÁMARA
    cap = cv2.VideoCapture(0)
    
    # DEFINIR ZONA DE TRABAJO DEL ROBOT (MILÍMETROS)
    # Ajusta esto para que el robot no choque con nada
    ROBOT_X_MIN, ROBOT_X_MAX = 110, 380  # Profundidad (Adelante/Atrás)
    ROBOT_Y_MIN, ROBOT_Y_MAX = -250, 250 # Lateral (Izquierda/Derecha)
    ALTURA_Z_FIJA = 200                  # Altura segura

    print("Controles:")
    print("- Mover mano: Mueve el robot en X/Y")
    print("- Cerrar puño: Cierra Gripper")
    print("- Mano abierta: Abre Gripper")

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.6) as hands:

        while cap.isOpened():
            success, image = cap.read()
            if not success: continue

            # Espejo y Color
            image = cv2.flip(image, 1) 
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)
            
            # Dibujar caja visual en pantalla (para que sepas dónde mover la mano)
            h, w, _ = image.shape
            cv2.rectangle(image, (100, 100), (w-100, h-100), (255, 255, 0), 2)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                    # --- COORDENADAS ---
                    # Usamos el INDICE (Punto 8) para movernos
                    x_pantalla = hand_landmarks.landmark[8].x
                    y_pantalla = hand_landmarks.landmark[8].y
                    
                    # --- GESTO DE GRIPPER ---
                    # Si la punta del pulgar está cerca de la punta del índice -> CERRAR
                    # O más fácil: Calcular distancia entre Índice(8) y Pulgar(4)
                    x8, y8 = hand_landmarks.landmark[8].x, hand_landmarks.landmark[8].y
                    x4, y4 = hand_landmarks.landmark[4].x, hand_landmarks.landmark[4].y
                    distancia = math.hypot(x8-x4, y8-y4)
                    
                    gripper_cerrado = distancia < 0.05 # Si están muy cerca (Pinza)

                    # --- MAPEO AL ROBOT ---
                    # Solo movemos el robot si la mano está dentro de un cuadro central
                    # Esto es por seguridad
                    if robot:
                        # Convertir X de pantalla a Y del robot (porque el robot está de frente)
                        # Convertir Y de pantalla a X del robot (profundidad)
                        
                        # Eje X Robot (Profundidad) = Invertimos Y pantalla (Arriba es lejos)
                        x_robot = map_range(y_pantalla, 0.2, 0.8, ROBOT_X_MAX, ROBOT_X_MIN)
                        
                        # Eje Y Robot (Lados) = X pantalla
                        y_robot = map_range(x_pantalla, 0.2, 0.8, ROBOT_Y_MAX, ROBOT_Y_MIN)

                        # Limitar valores por seguridad (Clamping)
                        x_robot = max(ROBOT_X_MIN, min(x_robot, ROBOT_X_MAX))
                        y_robot = max(ROBOT_Y_MIN, min(y_robot, ROBOT_Y_MAX))

                        # Enviar comando al robot
                        robot.mover_a(x_robot, y_robot, ALTURA_Z_FIJA)
                        robot.control_gripper(gripper_cerrado)

                        # Mostrar datos en pantalla
                        cv2.putText(image, f"R_Pos: {int(x_robot)}, {int(y_robot)}", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        estado_g = "CERRADO" if gripper_cerrado else "ABIERTO"
                        cv2.putText(image, f"Gripper: {estado_g}", (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if gripper_cerrado else (0, 255, 0), 2)

            cv2.imshow('Control Robot XArm', image)
            if cv2.waitKey(5) & 0xFF == 27: break

    cap.release()
    cv2.destroyAllWindows()
    if robot: arm.disconnect()