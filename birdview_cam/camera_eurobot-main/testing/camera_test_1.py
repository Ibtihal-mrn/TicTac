"""Ouvre uniquement la caméra à l'index 1 pour vérifier si c'est Iriun.
Appuie sur 'q' pour quitter."""

import cv2

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Impossible d'ouvrir la caméra à l'index 1.")
else:
    print("Caméra 1 ouverte. Appuie sur 'q' pour quitter.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Impossible de lire un frame.")
            break
        cv2.imshow("Camera 1", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
