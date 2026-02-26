import cv2
import mediapipe as mp
import pyautogui
import time

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

ultimo_cambio = 0
cooldown_segundos = 1.5

with mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,              
    min_detection_confidence=0.5, 
    min_tracking_confidence=0.5) as hands:

    while cap.isOpened():
        success, image = cap.read()
        if not success:
            continue

        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                y_punta_indice = hand_landmarks.landmark[8].y
                y_nudillo_indice = hand_landmarks.landmark[6].y
                y_punta_medio = hand_landmarks.landmark[12].y
                y_nudillo_medio = hand_landmarks.landmark[10].y
                y_punta_menique = hand_landmarks.landmark[20].y
                y_nudillo_menique = hand_landmarks.landmark[18].y

                indice_arriba = y_punta_indice < y_nudillo_indice
                medio_abajo = y_punta_medio > y_nudillo_medio
                menique_arriba = y_punta_menique < y_nudillo_menique


                tiempo_actual = time.time()
                if tiempo_actual - ultimo_cambio > cooldown_segundos:

                    if indice_arriba and medio_abajo and not menique_arriba:
                        print(">> SIGUIENTE")
                        pyautogui.press('right') 
                        ultimo_cambio = tiempo_actual
                        cv2.putText(image, 'SIGUIENTE >>', (50, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

                    elif menique_arriba and not indice_arriba:
                        print("<< ANTERIOR")
                        pyautogui.press('left') 
                        ultimo_cambio = tiempo_actual
                        cv2.putText(image, '<< ANTERIOR', (50, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        cv2.imshow('Control Gestual (ESC para salir)', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()