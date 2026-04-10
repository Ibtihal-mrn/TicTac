"""Point d'entree du pipeline de detection ArUco/QR."""

from __future__ import annotations

import cv2

from marker_detection import config
from marker_detection.detection import detect_all
from marker_detection.geometry import build_transforms
from marker_detection.markers import print_detected_objects, separate_markers
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

import sys, os
# Ajouter le dossier racine TicTac au path pour importer cerebros
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from cerebros.brain import Brain
from cerebros.models import Position, Team
from cerebros import config as cerebros_config


def main() -> None:
    """Boucle principale du pipeline."""

    # Ouvre une connexion camera et esp32.
    try:
        cap = create_capture()
        sender = ESP32Sender()
        sender.connect()
    except RuntimeError as exc:
        print(f"[ERREUR] {exc}")
        return

    # ── Connexion BLE vers le robot ─────────────────────────
    from cerebros.ble_sender import BLEBridge
    ble = BLEBridge(target_name="Eurobot")
    try:
        ble.connect()
    except Exception as e:
        print(f"[WARN] BLE non connecté: {e} — commandes loggées uniquement")

    # ── Attendre l'équipe depuis le team switch ESP32 (via heartbeat BLE) ─
    import time as _time
    print("[INIT] Attente de l'équipe depuis le team switch ESP32...")
    _team_wait_start = _time.time()
    while ble.detected_team is None and (_time.time() - _team_wait_start) < 10.0:
        _time.sleep(0.2)

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

    brain = Brain(
        team=team,
        robot_id=robot_id,
        initial_pos=Position(150, 1000),
        initial_heading=0.0,
    )
    brain.set_send_function(ble.send)

    create_windows()  # Ouverture des fenetres camera / aerienne.

    detector = create_aruco_detector()
    qr_detector = create_qr_detector()
    clahe = create_clahe()  # Pre-traitement pour stabiliser la detection.

    # Lissage temporel pour eviter les detections intermittentes.
    # Par le parametre min_hits, on peut forcer a attendre plusieurs detections coherentes
    # avant de valider un marqueur.
    aruco_tracker = Tracker(buffer_size=3, min_hits=2)
    qr_tracker = Tracker(buffer_size=3, min_hits=2)

    last_h_aerial = None
    frame_count = 0

    while True:

        ret, frame = cap.read()
        if not ret:
            break

        # Detecteurs en niveaux de gris.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detection ArUco + QR (multi-resolution).
        a_corners, a_ids, q_data, q_corners = detect_all(
            gray, detector, qr_detector, clahe)

        # Filtrage temporel des detections.
        a_ids, a_corners = aruco_tracker.update(a_ids, a_corners)
        q_data, q_corners = qr_tracker.update(q_data, q_corners)

        # Separation des coins de table vs marqueurs objets.
        corners_by_id, obj_aruco = separate_markers(a_ids, a_corners)

        # Calcul des homographies et des points de table.
        h_img_to_grid, h_grid_to_img, h_img_to_aerial, table_pts, aruco_pts = build_transforms(
            corners_by_id)

        if h_img_to_aerial is not None:
            # Garder la derniere vue aerienne valide.
            last_h_aerial = h_img_to_aerial

        # Vue aerienne (une frame sur deux).
        aerial = compute_aerial(frame, last_h_aerial, frame_count)

        # Rendu overlays.
        draw_grid(frame, h_grid_to_img)
        draw_table_outline(frame, table_pts, aruco_pts)

        draw_corner_markers(frame, corners_by_id)
        draw_object_markers(
            frame, aerial, obj_aruco, h_img_to_grid, last_h_aerial, frame_count
        )

        draw_qr_codes(
            frame, aerial, q_data, q_corners, h_img_to_grid, last_h_aerial, frame_count
        )

        # envoies des donnees detectees dans l'ESP32 (ou la console si pas de connexion).
        if h_img_to_grid is not None and frame_count % 2 == 0:
            # print_detected_objects(corners_by_id, obj_aruco, h_img_to_grid)
            send_detected_objects(corners_by_id,
                                  obj_aruco, h_img_to_grid, sender)

            # ── Cerebros : nourrir le cerveau avec les detections ─────
            detections = _build_detected_list(
                corners_by_id, obj_aruco, h_img_to_grid)

            # Calculer le heading du robot depuis les corners ArUco
            robot_heading = get_marker_heading(
                our_robot_marker_id, obj_aruco, h_img_to_grid)

            if detections:
                brain.feed_vision(detections, robot_heading=robot_heading)
                brain.tick()

        draw_status(frame, corners_by_id, obj_aruco, q_data, h_img_to_grid)

        cv2.imshow(config.WINDOW_CAMERA, frame)

        if aerial is not None:
            cv2.imshow(config.WINDOW_AERIAL, aerial)

        frame_count += 1

        if cv2.waitKey(50) & 0xFF == ord("q"):
            break  # Quitter avec 'q'.

    # shut down proprement les ressources.
    ble.disconnect()
    sender.disconnect()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
