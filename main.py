import cv2
import mediapipe as mp
import numpy as np
import math
import serial
import time

try:
    # HARD RULE: Keep baudrate at 115200! 
    # Standard 9600 is way too slow for real-time tracking.
    arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
    
    # Adjust the port above based on your OS:
    # - Linux   -> /dev/ttyUSBx or /dev/ttyACMx (check via terminal: ls /dev/ttyUSB* or ls /dev/ttyACM*)
        # i use nyarch btw :3
    # - Windows -> COMx (Check Device Manager)
    # - macOS   -> /dev/cu.usbmodemXXXX (Check via terminal: ls /dev/cu.*)

    time.sleep(2) 
    print("Arduino Connected.")
except:
    print("WARNING: Couldn't detect Arduino")
    arduino = None


mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
cap = cv2.VideoCapture(0)

# Standard 3D face model points. 
# DO NOT TOUCH (unless you really know the math behind it.)
model_points = np.array([
    (0.0, 0.0, 0.0), (0.0, -330.0, -65.0), (-225.0, 170.0, -135.0),
    (225.0, 170.0, -135.0), (-150.0, -150.0, -125.0), (150.0, -150.0, -125.0)
])

# Smoothing factor
alpha = 0.25 # <-- You can adjust this (I use 0.25)
# - Lower: smoother but more delay (laggy)
# - Higher: faster but more risk of jitter
prev_x = 0
prev_y = 0

print("Press 'q' to exit.")

while cap.isOpened():
    success, image = cap.read()
    if not success: break

    image = cv2.flip(image, 1)
    img_h, img_w, img_c = image.shape
    results = face_mesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            
            # -------------------------------------------------------------------------
            # Extract 2D landmarks from the face mesh
            # DO NOT TOUCH (unless you really know the math behind it.)
            face_2d = []
            for idx in [1, 152, 33, 263, 61, 291]:
                lm = face_landmarks.landmark[idx]
                x, y = int(lm.x * img_w), int(lm.y * img_h)
                face_2d.append([x, y])
            face_2d = np.array(face_2d, dtype=np.float64)

            # Camera matrix estimation
            focal_length = 1 * img_w
            cam_matrix = np.array([[focal_length, 0, img_h / 2], [0, focal_length, img_w / 2], [0, 0, 1]])
            dist_matrix = np.zeros((4, 1), dtype=np.float64)

            # Calculate rotation and translation vectors
            success, rot_vec, trans_vec = cv2.solvePnP(model_points, face_2d, cam_matrix, dist_matrix)
            rmat, jac = cv2.Rodrigues(rot_vec)
            sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
            
            if not sy < 1e-6:
                x = math.atan2(rmat[2,1] , rmat[2,2])
                y = math.atan2(-rmat[2,0], sy)
            else:
                x = math.atan2(-rmat[1,2], rmat[1,1])
                y = math.atan2(-rmat[2,0], sy)

            x_angle = np.degrees(x)
            y_angle = np.degrees(y)

            # Fix the 360-degree wrapping issue 
            if x_angle > 0: x_angle -= 180 
            else: x_angle += 180

            target_x = y_angle * -1
            target_y = x_angle * -1
            
            # Apply the smoothing filter we defined earlier
            smooth_x = (target_x * alpha) + (prev_x * (1 - alpha))
            smooth_y = (target_y * alpha) + (prev_y * (1 - alpha))
            prev_x = smooth_x
            prev_y = smooth_y

            print(f"X: {smooth_x:.2f} | Y: {smooth_y:.2f}")
            # -------------------------------------------------------------------------------

            if arduino is not None:
                # send as integer -> float are heavy.
                # Format: "X,Y\n" -> simple, lightweight, fast.
                data_str = f"{int(smooth_x)},{int(smooth_y)}\n"
                arduino.write(bytes(data_str, 'utf-8'))

            # Visualization stuff (Drawing lines and text on screen)
            nose_3d_projection, jacobian = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rot_vec, trans_vec, cam_matrix, dist_matrix)
            p1 = (int(face_2d[0][0]), int(face_2d[0][1]))
            p2 = (int(nose_3d_projection[0][0][0]), int(nose_3d_projection[0][0][1]))
            cv2.line(image, p1, p2, (255, 0, 0), 3)
            cv2.putText(image, f"X: {int(smooth_x)}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image, f"Y: {int(smooth_y)}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Head Pose Estimation', image)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()