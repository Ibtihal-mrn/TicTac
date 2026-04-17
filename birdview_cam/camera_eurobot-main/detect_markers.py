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
    draw_brain_overlay,
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


# Bypass de la tirette pour les tests PC/vision.
# Remettre a False pour revenir au demarrage sur front IN -> OUT.
BYPASS_TIRETTE = False


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
    home_pos = Position(150, 1000)  # Position de base pour retour (pas la pos reelle)
    brain = Brain(
        team=team,
        robot_id=robot_id,
        initial_pos=None,        # Pas de position par defaut — attente de la vision
        initial_heading=0.0,
    )
    brain.set_send_function(ble.send)
    brain.set_ble_bridge(ble)

    # ── PHASE INIT : mission en batches ───────────────────────────────
    # Batch 1 : aller faire la mission principale
    # Batch 2 : correction + suite depuis la position caméra
    # Batch 3 : retour au nid
    if team == Team.BLUE:
        batch1 = [
            # batch 1 : curseur de température et amener 12 caisses au nid
            Position(1500, 1100),
            Position(1200, 0),
            Position(1500, 0), #activer electr
            Position(2200, 0),#désactiver electr
            Position(2850, 0),
            Position(2850, 1800),
            Position(1500, 800), #position centrale pour recalcul
            
            
            ]
        batch2 = [
            # batch 2 : aller vider les nids adveres 
            Position(1600, 800),#nid du milieu
            Position(600, 800),
            Position(2000, 50),#vider haut
            Position(500, 50),
            Position(1500, 800), #position centrale pour recalcul
        ]
        batch3 = [
            # batch 3 : retour au nid
            Position(2600, 1100),
            Position(2600, 1900),  
        ]
    else:  # Team.YELLOW
        batch1 = [
            Position(1500, 1100),
            Position(1800, 0),
            Position(1500, 0),#activer electr
            Position(1000, 0),#désactiver electr
            Position(150, 0),
            Position(150, 1000),
            Position(1500, 800), #position centrale pour recalcul
            
        ]
        batch2 = [
            # batch 2 : aller vider les nids adverses 
            Position(1400, 800),#vider centre
            Position(2500, 800),
            Position(1100, 50),#vider haut
            Position(2600, 50),
            Position(1500, 800),#position centrale pour recalcul
        ]
        batch3 = [
            Position(400, 1100),
            Position(400, 1900),# retour au nid
        ]

    batches = [batch1, batch2, batch3]
    batch_labels = [
        ["B1_T" + str(i+1) for i in range(len(batch1))],
        ["B2_T" + str(i+1) for i in range(len(batch2))],
        ["B3_RETOUR"],
    ]
    brain.set_batches(batches, batch_labels)
    print(f"[INIT] Team={team.value} — {len(batches)} batches")


    # 7. OpenCv Window and Tag Detection
    create_windows()                      # Ouverture des fenetres camera / aerienne.
    detector = create_aruco_detector()    # configures ArUco detection.
    qr_detector = create_qr_detector()    # configures QR detection.
    clahe = create_clahe()                # Pre-traitement pour stabiliser la detection.


    # 8. Tracking
    aruco_tracker = Tracker(buffer_size=3, min_hits=2)   # mutliple stable frames before accepting Aruco Tag
    qr_tracker = Tracker(buffer_size=3, min_hits=2)

    last_h_aerial = None
    frame_count = 0
    init_plan_done = False       # A* calcule une seule fois
    init_vision_frames = 0       # Compteur de frames avec vision valide
    tirette_seen_inserted = False


    # 10. Main Frame Loop
    while True:
        # 1. Read
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        _ticked_this_frame = False

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

                # ── INIT : lancer le premier batch apres vision stable ──
                if (not init_plan_done and init_vision_frames >= 5
                        and brain.robot_position_known):
                    print("[INIT] Vision stable + robot detecte — prêt pour batches")
                    init_plan_done = True
                    if BYPASS_TIRETTE:
                        print("[INIT] Bypass tirette actif — demarrage immediat du match.")
                        brain.start_match()
                    else:
                        brain.phase = BrainPhase.READY
                        print("[INIT] Insere la tirette puis retire-la pour lancer le match.")

                # ── TIRETTE : demarrer uniquement sur front IN -> OUT ──
                if (not BYPASS_TIRETTE and init_plan_done
                        and brain.phase == BrainPhase.READY):
                    if ble.tirette_inserted is True:
                        if not tirette_seen_inserted:
                            print("[MATCH] Tirette détectée IN — attente du retrait")
                        tirette_seen_inserted = True
                    elif ble.tirette_inserted is False and tirette_seen_inserted:
                        print("[MATCH] Front tirette IN -> OUT détecté — démarrage!")
                        brain.start_match()
                        tirette_seen_inserted = False

                # ── RUN : tick du pipeline multi-batch ─────────────────
                if brain.phase in (BrainPhase.RUNNING_BATCH,
                                   BrainPhase.WAITING_RECALC,
                                   BrainPhase.WAITING_TIMER):
                    brain.tick()
                    _ticked_this_frame = True

        # ── Tick hors détection : garantir les timers même sans vision ──
        if not _ticked_this_frame and brain.phase in (
                BrainPhase.RUNNING_BATCH, BrainPhase.WAITING_TIMER):
            brain.tick()
        # draw_status(frame, corners_by_id, obj_aruco, q_data, h_img_to_grid)

        # ── Overlay Cerebros : targets + path A* + position robot ─────
        # draw_brain_overlay(
        #     frame, aerial,
        #     brain.planned_path,
        #     brain.planned_targets,
        #     brain.planned_target_labels,
        #     brain.robot.position,
        #     brain.robot.heading_deg,
        #     h_grid_to_img,
        # )

        cv2.imshow(config.WINDOW_CAMERA, frame)
        if aerial is not None: cv2.imshow(config.WINDOW_AERIAL, aerial)
        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break  # Quitter avec 'q'.


    # Clean Shutdown
    ble.disconnect()
    sender.disconnect()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__": main()
