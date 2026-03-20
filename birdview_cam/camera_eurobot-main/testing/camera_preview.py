"""Ouvre chaque caméra détectée et affiche le flux vidéo.
Appuie sur une touche pour passer à la caméra suivante, ou 'q' pour quitter."""

import cv2

for i in range(2):
    cap = cv2.VideoCapture(i)
    if not cap.isOpened():
        continue

    print(f"--- Caméra index {i} --- (appuie sur une touche pour passer, 'q' pour quitter)")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow(f"Camera {i}", frame)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:  # any key pressed
            break

    cap.release()
    cv2.destroyAllWindows()

    if key == ord('q'):
        break

print("Terminé.")
