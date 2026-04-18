"""Point d'entree du pipeline de detection ArUco/QR.

Birdview camera pipeline. Its job is to:
    - open the camera,
    - detect ArUco and QR markers,
    - stabilize detections over time,
    - compute table geometry / aerial view,
    - send detected objects to the ESP32,
    - feed detections into cerebros for autonomy planning,
    - start the match when the team switch / “tirette” condition is met.
"""
from __future__ import annotations


import cv2
import sys, os
import time as _time

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
from marker_detection.markers import _build_detected_list, get_marker_heading


# Ajouter le dossier racine TicTac au path pour importer cerebros
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from cerebros.brain import Brain, BrainPhase
from cerebros.models import Action, ActionType, Position, Team
from cerebros import config as cerebros_config


# Bypass de la tirette pour les tests PC/vision.
# Remettre a False pour revenir au demarrage sur front IN -> OUT.
BYPASS_TIRETTE = False


def main() -> None:
    """Boucle principale du pipeline."""

    # 1. Ouvre la camera.
    try:
        cap = create_capture()
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

    # 4. Team, Robot ID & Marker ID range
    if ble.detected_team == "YELLOW":
        team = Team.YELLOW
        robot_id = "YR1"
        our_robot_marker_ids = cerebros_config.OUR_ROBOT_YELLOW_IDS  # {6..10}
    else:
        team = Team.BLUE
        robot_id = "BR1"
        our_robot_marker_ids = cerebros_config.OUR_ROBOT_BLUE_IDS    # {1..5}

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
    # --- Patch : on démarre même sans voir le marker robot ---
    brain._robot_position_known = True
    brain.set_send_function(ble.send)
    brain.set_ble_bridge(ble)

    # ── PHASE INIT : mission en batches ───────────────────────────────
    # Batch 1 : aller faire la mission principale
    # Batch 2 : correction + suite depuis la position caméra
    # Batch 3 : retour au nid
    if team == Team.BLUE:
        # 3 commandes de sortie de zone (avant le batch)
        exit_actions = [
            Action(ActionType.FORWARD, 500),
            Action(ActionType.ROTATE, -90),
            Action(ActionType.FORWARD, 500),
        ]
        batch1 = [
            # batch 1 : amener 6 caisses dans le nid
            Position(2300, 700),
            Position(2600, 0),
            Position(2900, 50),
            Position(3000, 1600), 
            # implementer fonction de recul
            Position(2000, 800),
            
            
            ]
        batch2 = [  
            # batch 2 : attendre en zone trackable via caméra           
            Position(2400, 1000),       
        ]
        
        batch3 = [
            # batch 3 : retour au nid
            Position(2600, 800),
            Position(2600, 1900),  
        ]
    else:  # Team.YELLOW
        # 3 commandes de sortie de zone (avant le batch)
        exit_actions = [
            Action(ActionType.FORWARD, 500),
            Action(ActionType.ROTATE, 90),
            Action(ActionType.FORWARD, 500),
        ]
        batch1 = [
            Position(700, 700),
            Position(400, 50),
            Position(50, 0),
            Position(0, 1600),
            # implementer fonction de recul
            Position(700, 800),  
        ]
        batch2 = [
            # batch 2 : attendre en zone trackable via caméra
            Position(700, 1000),                
        ]
        batch3 = [
            #retour au nid
            Position(400, 1100),
            Position(300, 1900),
        ]

    batches = [batch1, batch2, batch3]
    batch_labels = [
        ["B1_T" + str(i+1) for i in range(len(batch1))],
        ["B2_T" + str(i+1) for i in range(len(batch2))],
        ["B3_RETOUR"],
    ]
    brain.set_exit_actions(exit_actions)
    brain.set_batches(batches, batch_labels)
    # Inject BACKWARD 100mm dans batch 1 après le 4ème target (index 3)
    brain.set_batch_inject_actions({
        (0, 3): [Action(ActionType.BACKWARD, 100)],
    })
    # Post-batch 3 : deploy/retract à l'infini (1s chaque) jusqu'à fin du match
    # 15 cycles × 4 cmds = 60 (queue ESP32 = 64 slots max)
    servo_loop: list[Action] = []
    for _ in range(15):
        servo_loop.append(Action(ActionType.DEPLOY))
        servo_loop.append(Action(ActionType.WAIT, 1000))
        servo_loop.append(Action(ActionType.RETRACT))
        servo_loop.append(Action(ActionType.WAIT, 1000))
    brain.set_post_batch_actions({2: servo_loop})   # batch index 2 = batch 3
    print(f"[INIT] Team={team.value} — {len(batches)} batches, "
          f"{len(exit_actions)} exit commands")

    # Heading attendu après la sortie de zone :
    #   BLUE  → face à -X = 180°
    #   YELLOW → face à +X = 0°
    _expected_heading_after_exit = 180.0 if team == Team.BLUE else 0.0
    print(f"[INIT] Heading attendu après sortie de zone: "
          f"{_expected_heading_after_exit:.0f}°")

    # Offset ArUco : calibré au premier reading après la sortie de zone
    # (robot visible, heading connu)
    _heading_offset: float | None = None  # None = pas encore calibré


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

        # 10. Feed vision to Cerebros
        if h_img_to_grid is not None and frame_count % 2 == 0:
            # Build Detection List for Cerebros
            detections = _build_detected_list(
                corners_by_id, obj_aruco, h_img_to_grid)

            # Compute Robot Heading (pick whichever marker in our range is visible)
            robot_heading = None
            for _mid, _ in obj_aruco:
                if _mid in our_robot_marker_ids:
                    raw_heading = get_marker_heading(
                        _mid, obj_aruco, h_img_to_grid)
                    if raw_heading is not None:
                        # Calibration : au premier reading après la sortie de zone
                        # (phase WAITING_RECALC = robot vient de sortir, heading connu)
                        if _heading_offset is None:
                            if brain.phase == BrainPhase.WAITING_RECALC:
                                _heading_offset = _expected_heading_after_exit - raw_heading
                                while _heading_offset > 180:
                                    _heading_offset -= 360
                                while _heading_offset < -180:
                                    _heading_offset += 360
                                print(f"[CALIB] Offset ArUco calibré: "
                                      f"expected={_expected_heading_after_exit:.0f}° "
                                      f"raw={raw_heading:.0f}° → "
                                      f"offset={_heading_offset:.0f}°")
                        if _heading_offset is not None:
                            robot_heading = raw_heading + _heading_offset
                            while robot_heading > 180:
                                robot_heading -= 360
                            while robot_heading < -180:
                                robot_heading += 360
                    break

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
                elif not init_plan_done and init_vision_frames % 30 == 0 and init_vision_frames > 0:
                    labels_seen = [d[0] for d in detections[:5]]
                    print(f"[INIT] frames={init_vision_frames}, robot_known={brain.robot_position_known}, "
                          f"labels={labels_seen}...")

                # ── RUN : tick du pipeline multi-batch ─────────────────
                if brain.phase in (BrainPhase.RUNNING_BATCH,
                                   BrainPhase.WAITING_RECALC,
                                   BrainPhase.WAITING_TIMER):
                    brain.tick()
                    _ticked_this_frame = True

        # ── TIRETTE : demarrer uniquement sur front IN -> OUT ──
        # (vérifié à chaque frame, indépendamment de la vision)
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
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__": main()
