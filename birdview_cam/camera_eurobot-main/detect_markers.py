"""Point d'entree du pipeline de detection ArUco/QR."""

"""
Birdview camera pipeline. Its job is to:
    - open the camera,
    - detect ArUco and QR markers,
    - stabilize detections over time,
    - compute table geometry / aerial view,
    - send detected objects to the ESP32,
    - feed detections into cerebros for autonomy planning,
    - start the match when the team switch / “tirette” condition is met.
"""



import cv2
import sys, os
import time as _time
from __future__ import annotations

# 
from marker_detection import config
from marker_detection.detection import detect_all
from marker_detection.geometry import build_transforms
from marker_detection.markers import separate_markers
from marker_detection.runtime import (
    create_aruco_detector,
    create_capture,
    create_clahe,
    create_qr_detector,
    create_windows,
)
from marker_detection.tracking import Tracker
from marker_detection.visualization import (
    compute_aerial,
    draw_corner_markers,
    draw_grid,
    draw_object_markers,
    draw_qr_codes,
    draw_status,
    draw_table_outline,
)
from marker_detection.esp32_sender import ESP32Sender
from marker_detection.markers import send_detected_objects, _build_detected_list, get_marker_heading


# Ajouter le dossier racine TicTac au path pour importer cerebros
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from cerebros.brain import Brain, BrainPhase
from cerebros.models import Position, Team
from cerebros import config as cerebros_config


def main() -> None:
    """Boucle principale du pipeline."""

    # 1. Ouvre une connexion camera et esp32.
    try:
        cap = create_capture()
        sender = ESP32Sender()
        sender.connect()
    except RuntimeError as exc:
        print(f"[ERREUR] {exc}")
        return

    # 2. BLE connexion to the robot
    from cerebros.ble_sender import BLEBridge
    ble = BLEBridge(target_name="Eurobot")
    try:
        ble.connect()
    except Exception as e:
        print(f"[WARN] BLE non connecté: {e} — commandes loggées uniquement")

    # 3. Team Switch info from Robot (via heartbeat BLE) ─
    print("[INIT] Attente de l'équipe depuis le team switch ESP32...")
    _team_wait_start = _time.time()
    while ble.detected_team is None and (_time.time() - _team_wait_start) < 10.0:
        _time.sleep(0.2)

    # 4. Team, Robot ID & Marker ID
    if ble.detected_team == "YELLOW":
        team = Team.YELLOW
        robot_id = "YR1"
        our_robot_marker_id = cerebros_config.OUR_ROBOT_YELLOW_ID  # =6
    else:
        team = Team.BLUE
        robot_id = "BR1"
        our_robot_marker_id = cerebros_config.OUR_ROBOT_BLUE_ID    # =1

    if ble.detected_team is not None:
        print(f"[INIT] Équipe reçue du robot: {ble.detected_team}")
    else:
        print("[INIT] Timeout — équipe par défaut: BLUE")

    # 5. Creating the Robot Brain
    brain = Brain(
        team=team,
        robot_id=robot_id,
        initial_pos=Position(150, 1000),
        initial_heading=0.0,
    )
    brain.set_send_function(ble.send)

    # 6. PHASE INIT : mission hardcodee ────────────────────────────────
    targets = [
        #  Mission : milieu de la table, puis milieu tout a droite
        Position(1500, 1000),   # Centre de la table  
        Position(2850, 1000),   # Centre-droit
    ]
    labels = ["CENTRE_TABLE", "CENTRE_DROIT"]
    print(f"[INIT] Mission : {labels}")


    # 7. OpenCv Window and Tag Detection
    create_windows()                      # Ouverture des fenetres camera / aerienne.
    detector = create_aruco_detector()    # configures ArUco detection.
    qr_detector = create_qr_detector()    # configures QR detection.
    clahe = create_clahe()                # Pre-traitement pour stabiliser la detection.


    # 8. Tracking
    aruco_tracker = Tracker(buffer_size=3, min_hits=2)   # mutliple stable frames before accepting Aruco Tag
    qr_tracker = Tracker(buffer_size=3, min_hits=2)

    # 9. State Variables
    last_h_aerial = None         # last valid aerial homography.
    frame_count = 0              # frame-based throttling.
    init_plan_done = False       # ensures A* planning happens only once.
    init_vision_frames = 0       # counts stable frames before planning starts.


    # 10. Main Frame Loop
    while True:
        # 1. Read
        ret, frame = cap.read()
        if not ret:
            break

        # 2. Convert to grayScale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Detect ArUco and QR markers
        a_corners, a_ids, q_data, q_corners = detect_all(
            gray, detector, qr_detector, clahe)

        # 4. Temporal filtering (multi-image tag validation)
        a_ids, a_corners = aruco_tracker.update(a_ids, a_corners)
        q_data, q_corners = qr_tracker.update(q_data, q_corners)

        # 5. Separate table corners vs object markers
        corners_by_id, obj_aruco = separate_markers(a_ids, a_corners)

        # 6. Build transforms (image to usable matrix of coordinates)
        (
        h_img_to_grid,      # image coordinates → logical grid coordinates,
        h_grid_to_img,      # inverse transform,
        h_img_to_aerial,    # image → aerial top-down homography,
        table_pts,          # table outline points,
        aruco_pts           # detected ArUco corner points.
        )= build_transforms(corners_by_id)

        # 7. Keep the last valid aerial transform
        if h_img_to_aerial is not None:
            # Garder la derniere vue aerienne valide.
            last_h_aerial = h_img_to_aerial

        # 8. Compute aerial view
        aerial = compute_aerial(frame, last_h_aerial, frame_count)

        # 9. Draw overlays
        draw_grid(frame, h_grid_to_img)                     # logical grid,
        draw_table_outline(frame, table_pts, aruco_pts)     # table boundaries,
        draw_corner_markers(frame, corners_by_id)           # marker locations,
        draw_object_markers(frame, aerial, obj_aruco, h_img_to_grid, last_h_aerial, frame_count)    # object positions,
        draw_qr_codes(frame, aerial, q_data, q_corners, h_img_to_grid, last_h_aerial, frame_count)  # QR codes

        # 10. Send to ESP32 Robot
        if h_img_to_grid is not None and frame_count % 2 == 0:
            # Send every 2 frames
            send_detected_objects(corners_by_id, obj_aruco, h_img_to_grid, sender)

            # Build Detection List for Cerebros
            detections = _build_detected_list(
                corners_by_id, obj_aruco, h_img_to_grid)

            # Compute Robot Heading
            robot_heading = get_marker_heading(
                our_robot_marker_id, obj_aruco, h_img_to_grid)

            # 
            if detections:
                brain.feed_vision(detections, robot_heading=robot_heading)
                init_vision_frames += 1

                # ── INIT : lancer A* apres quelques frames stables ────
                if not init_plan_done and init_vision_frames >= 10:
                    print("[INIT] Vision stable — lancement planification A*")
                    ok = brain.init_plan(
                        target_positions=targets, target_labels=labels)
                    if ok:
                        init_plan_done = True
                        print("[INIT] Plan OK. Insere la tirette puis retire-la pour lancer le match.")
                    else:
                        print("[INIT] Echec planification — nouvel essai dans quelques frames")
                        init_vision_frames = 0  # retry

                # TIRETTE : demarrer le match quand elle est retiree ─
                if init_plan_done and brain.phase == BrainPhase.READY:
                    if ble.match_started:
                        print("[MATCH] Tirette retiree — MATCH START!")
                        brain.start_match()

                # RUN : tick de monitoring ───────────────────────────
                if brain.phase == BrainPhase.RUNNING:
                    brain.tick()


        # 11. Display status and windows
        draw_status(frame, corners_by_id, obj_aruco, q_data, h_img_to_grid)
        cv2.imshow(config.WINDOW_CAMERA, frame)
        if aerial is not None: cv2.imshow(config.WINDOW_AERIAL, aerial)
        frame_count += 1

        # 12. Exit Condition
        if cv2.waitKey(50) & 0xFF == ord("q"): break  # Quitter avec 'q'.


    # Clean Shutdown
    ble.disconnect()
    sender.disconnect()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__": main()
