import cv2
import mediapipe as mp
import socket

'''
Gesture Recognizer
 >> Recognizes hand signals from camera
 >> Send data to voice_teleop ros node
'''

HOST = ""  # Localhost
PORT = 65434           
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

def detect_gesture(landmarks):
    # Gesture detection logic (simplified)
    thumb_up = landmarks[4].y < landmarks[2].y
    index_up = landmarks[8].y < landmarks[5].y
    middle_up = landmarks[12].y < landmarks[9].y
    ring_up = landmarks[16].y < landmarks[13].y
    pinky_up = landmarks[20].y < landmarks[17].y

    count = sum([thumb_up, index_up, middle_up, ring_up, pinky_up])

    if count == 5:
        return "w"
    elif count == 4:
        return "a"
    elif count == 3:
        return "s"
    elif count == 2:
        return "d"
    else :
        return "stop"


    

cap = cv2.VideoCapture(0)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(frame_rgb)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                gesture = detect_gesture(hand_landmarks.landmark)
                print(f"Gesture detected: {gesture}")
                s.sendall(gesture.encode())

                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Gesture Control", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()