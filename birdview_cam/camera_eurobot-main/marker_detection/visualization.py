"""Fonctions de rendu pour la vue camera et la vue aerienne."""

from __future__ import annotations

import cv2
import numpy as np

from marker_detection import config
from marker_detection.geometry import to_cell


def draw_grid(frame: np.ndarray, h_grid_to_img: np.ndarray | None) -> None:
    """Dessine la grille de jeu projetee dans l'image camera."""
    if h_grid_to_img is None:
        return

    for x in range(0, config.GRID_COLS + 1, 5):
        line_grid = np.float32([[[x, 0]], [[x, config.GRID_ROWS]]])
        line_img = cv2.perspectiveTransform(line_grid, h_grid_to_img).astype(np.int32)

        if config.ARUCO_OFFSET_COLS <= x <= (config.ARUCO_OFFSET_COLS + config.ARUCO_INNER_COLS):
            color = (0, 255, 0)
        else:
            color = (0, 150, 0)

        cv2.line(frame, tuple(line_img[0, 0]), tuple(line_img[1, 0]), color, 1)

    for y in range(0, config.GRID_ROWS + 1, 5):
        line_grid = np.float32([[[0, y]], [[config.GRID_COLS, y]]])
        line_img = cv2.perspectiveTransform(line_grid, h_grid_to_img).astype(np.int32)

        if config.ARUCO_OFFSET_ROWS <= y <= (config.ARUCO_OFFSET_ROWS + config.ARUCO_INNER_ROWS):
            color = (0, 255, 0)
        else:
            color = (0, 150, 0)

        cv2.line(frame, tuple(line_img[0, 0]), tuple(line_img[1, 0]), color, 1)


def draw_table_outline(frame: np.ndarray, table_pts: np.ndarray | None, aruco_pts: np.ndarray | None) -> None:
    """Dessine les contours de table et les points ArUco retenus."""
    if table_pts is not None:
        cv2.polylines(frame, [table_pts.astype(np.int32).reshape(-1, 1, 2)], True, (0, 0, 255), 2)

    if aruco_pts is None:
        return

    visible_mask = np.isfinite(aruco_pts).all(axis=1)
    visible_pts = aruco_pts[visible_mask]
    if len(visible_pts) >= 2:
        cv2.polylines(
            frame,
            [visible_pts.astype(np.int32).reshape(-1, 1, 2)],
            len(visible_pts) >= 3,
            (255, 255, 0),
            2,
        )

    labels = ["TL(23)", "TR(22)", "BR(20)", "BL(21)"]
    colors = [(255, 0, 255), (255, 128, 0), (0, 255, 255), (128, 255, 0)]
    for pt, label, color, is_visible in zip(aruco_pts, labels, colors, visible_mask):
        if not is_visible:
            continue
        p = tuple(pt.astype(int))
        cv2.circle(frame, p, 8, color, -1)
        cv2.putText(frame, label, (p[0] + 10, p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def draw_aerial_grid(aerial: np.ndarray) -> None:
    """Dessine la grille dans la vue aerienne."""
    cell_w = config.AERIAL_W / config.GRID_COLS
    cell_h = config.AERIAL_H / config.GRID_ROWS

    for col in range(0, config.GRID_COLS + 1, 5):
        x = int(col * cell_w)
        color = (0, 255, 0) if config.ARUCO_OFFSET_COLS <= col <= (config.ARUCO_OFFSET_COLS + config.ARUCO_INNER_COLS) else (0, 150, 0)
        cv2.line(aerial, (x, 0), (x, config.AERIAL_H), color, 1)
        if col % 10 == 0:
            cv2.putText(aerial, f"{col}", (x + 2, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    for row in range(0, config.GRID_ROWS + 1, 5):
        y = int(row * cell_h)
        color = (0, 255, 0) if config.ARUCO_OFFSET_ROWS <= row <= (config.ARUCO_OFFSET_ROWS + config.ARUCO_INNER_ROWS) else (0, 150, 0)
        cv2.line(aerial, (0, y), (config.AERIAL_W, y), color, 1)
        if row % 5 == 0:
            cv2.putText(aerial, f"{row}", (2, y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)


def compute_aerial(frame: np.ndarray, h_aerial: np.ndarray | None, frame_count: int) -> np.ndarray | None:
    """Calcule la vue aerienne."""
    if h_aerial is None or frame_count % config.AERIAL_UPDATE_INTERVAL != 0:
        return None

    aerial = cv2.warpPerspective(frame, h_aerial, (config.AERIAL_W, config.AERIAL_H))
    draw_aerial_grid(aerial)

    return aerial


def draw_corner_markers(frame: np.ndarray, corners_by_id: dict[int, np.ndarray]) -> None:
    """Dessine les ArUco servant de coins de table."""
    for marker_id, corner in corners_by_id.items():
        draw_detection(frame, corner[0], f"C{marker_id}", (0, 255, 255))


def draw_object_markers(
    frame: np.ndarray,
    aerial: np.ndarray | None,
    obj_aruco: list[tuple[int, np.ndarray]],
    h_img_to_grid: np.ndarray | None,
    h_aerial: np.ndarray | None,
    frame_count: int,
) -> None:
    """Dessine les ArUco objets."""
    for marker_id, corner in obj_aruco:
        center = corner[0].mean(axis=0)
        pos = to_cell(center[0], center[1], h_img_to_grid)

        label = f"A{marker_id}[{pos[0]:.1f},{pos[1]:.1f}]" if pos else f"A{marker_id}"

        draw_detection(frame, corner[0], label, (0, 255, 0))

        if aerial is not None and frame_count % 2 == 0:
            draw_aerial_detection(
                aerial,
                corner[0],
                f"A{marker_id}",
                (0, 255, 0),
                h_aerial,
            )


def draw_qr_codes(
    frame: np.ndarray,
    aerial: np.ndarray | None,
    q_data: list[str],
    q_corners: list[np.ndarray],
    h_img_to_grid: np.ndarray | None,
    h_aerial: np.ndarray | None,
    frame_count: int,
) -> None:
    """Dessine les QR codes detectes."""
    for data, corners in zip(q_data, q_corners):
        center = corners.mean(axis=0)
        pos = to_cell(center[0], center[1], h_img_to_grid)

        short = data[:10] + "..." if len(data) > 10 else data
        label = f"QR:{short}[{pos[0]:.1f},{pos[1]:.1f}]" if pos else f"QR:{short}"

        draw_detection(frame, corners, label, (0, 165, 255))

        if aerial is not None and frame_count % 2 == 0:
            draw_aerial_detection(
                aerial,
                corners,
                f"QR:{short}",
                (0, 165, 255),
                h_aerial,
            )


def draw_status(
    frame: np.ndarray,
    corners_by_id: dict[int, np.ndarray],
    obj_aruco: list[tuple[int, np.ndarray]],
    q_data: list[str],
    h_img_to_grid: np.ndarray | None,
) -> None:
    """Dessine les informations de statut."""
    n_corners = len(corners_by_id)
    missing = config.CORNER_IDS - set(corners_by_id)
    grid_ok = h_img_to_grid is not None

    cv2.putText(
        frame,
        f"C:{n_corners}/4 Obj:{len(obj_aruco) + len(q_data)} Grid:{'OK' if grid_ok else '...'}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0) if grid_ok else (0, 165, 255),
        2,
    )

    if missing:
        cv2.putText(
            frame,
            f"Missing: {sorted(missing)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )


def draw_detection(frame: np.ndarray, pts_raw: np.ndarray, label: str, color: tuple[int, int, int]) -> None:
    """Dessine un polygone detecte et son libelle sur une image."""
    pts = pts_raw.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(frame, [pts], True, color, 2)
    center = pts_raw.mean(axis=0).astype(int)
    cv2.putText(frame, label, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)


def draw_aerial_detection(
    aerial: np.ndarray,
    pts_camera: np.ndarray,
    label: str,
    color: tuple[int, int, int],
    h_img_to_aerial: np.ndarray | None,
) -> None:
    """Projette et dessine une detection camera sur la vue aerienne."""
    if h_img_to_aerial is None:
        return

    pts_aerial = cv2.perspectiveTransform(pts_camera.reshape(-1, 1, 2).astype(np.float32), h_img_to_aerial)
    pts_int = pts_aerial.astype(np.int32)

    cv2.polylines(aerial, [pts_int], True, color, 2)
    center = pts_aerial.mean(axis=0)[0].astype(int)
    cv2.putText(aerial, label, (center[0] + 5, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1)


# ── Cerebros overlay : targets + A* path ─────────────────────────────────────

def _mm_to_pixel(x_mm: float, y_mm: float, h_mm_to_img: np.ndarray) -> tuple[int, int] | None:
    """Convertit une position mm de la table en pixel image via homographie."""
    # Symétrie gauche-droite : l'axe X de cerebros est inversé par rapport à la caméra
    x_mm = config.TABLE_W_MM - x_mm
    pt = np.float32([[[x_mm, y_mm]]])
    projected = cv2.perspectiveTransform(pt, h_mm_to_img)
    px, py = projected[0, 0]
    return (int(px), int(py))


def _mm_to_aerial(x_mm: float, y_mm: float) -> tuple[int, int]:
    """Convertit une position mm de la table en pixel dans la vue aérienne."""
    # Symétrie gauche-droite : l'axe X de cerebros est inversé par rapport à la caméra
    px = int((config.TABLE_W_MM - x_mm) / config.TABLE_W_MM * config.AERIAL_W)
    py = int(y_mm / config.TABLE_H_MM * config.AERIAL_H)
    return (px, py)


def draw_brain_overlay(
    frame: np.ndarray,
    aerial: np.ndarray | None,
    planned_path: list,
    planned_targets: list,
    planned_target_labels: list[str],
    robot_pos,
    robot_heading_deg: float,
    h_grid_to_img: np.ndarray | None,
) -> None:
    """Dessine les targets, le path A* et la position robot sur la caméra et la vue aérienne.

    Args:
        frame: image caméra (modifiée in-place)
        aerial: vue aérienne (modifiée in-place), ou None
        planned_path: liste de Position (mm) — waypoints A*
        planned_targets: liste de Position (mm) — objectifs
        planned_target_labels: labels des objectifs
        robot_pos: Position actuelle du robot (mm)
        robot_heading_deg: heading du robot en degrés (0° = axe X+, sens trigo)
        h_grid_to_img: homographie grille→image (pour la vue caméra)
    """
    if not planned_path and not planned_targets:
        return

    # Construire h_mm_to_img depuis h_grid_to_img
    h_mm_to_img = None
    if h_grid_to_img is not None:
        h_mm_to_grid = np.array(
            [[config.GRID_COLS / config.TABLE_W_MM, 0.0, 0.0],
             [0.0, config.GRID_ROWS / config.TABLE_H_MM, 0.0],
             [0.0, 0.0, 1.0]], dtype=np.float64)
        h_mm_to_img = h_grid_to_img @ h_mm_to_grid

    # ── Dessiner le path A* (ligne cyan) ──────────────────────────────
    if len(planned_path) >= 2:
        # Vue caméra
        if h_mm_to_img is not None:
            cam_pts = []
            for wp in planned_path:
                p = _mm_to_pixel(wp.x, wp.y, h_mm_to_img)
                if p is not None:
                    cam_pts.append(p)
            if len(cam_pts) >= 2:
                for i in range(len(cam_pts) - 1):
                    cv2.line(frame, cam_pts[i], cam_pts[i + 1], (255, 255, 0), 2)
                # Petits cercles aux waypoints
                for p in cam_pts:
                    cv2.circle(frame, p, 4, (255, 255, 0), -1)

        # Vue aérienne
        if aerial is not None:
            aer_pts = [_mm_to_aerial(wp.x, wp.y) for wp in planned_path]
            for i in range(len(aer_pts) - 1):
                cv2.line(aerial, aer_pts[i], aer_pts[i + 1], (255, 255, 0), 2)
            for p in aer_pts:
                cv2.circle(aerial, p, 3, (255, 255, 0), -1)

    # ── Dessiner les targets (losanges magenta) ──────────────────────
    for i, target in enumerate(planned_targets):
        label = planned_target_labels[i] if i < len(planned_target_labels) else f"T{i+1}"

        # Vue caméra
        if h_mm_to_img is not None:
            p = _mm_to_pixel(target.x, target.y, h_mm_to_img)
            if p is not None:
                _draw_target_marker(frame, p, label, (255, 0, 255), size=14)

        # Vue aérienne
        if aerial is not None:
            p = _mm_to_aerial(target.x, target.y)
            _draw_target_marker(aerial, p, label, (255, 0, 255), size=10)

    # ── Dessiner la position robot + flèche heading ──────────────────
    if robot_pos is not None:
        import math
        heading_rad = math.radians(robot_heading_deg)
        # Longueur de la flèche en mm (sur la table)
        arrow_len_mm = 150
        tip_x_mm = robot_pos.x + arrow_len_mm * math.cos(heading_rad)
        tip_y_mm = robot_pos.y + arrow_len_mm * math.sin(heading_rad)

        if h_mm_to_img is not None:
            p = _mm_to_pixel(robot_pos.x, robot_pos.y, h_mm_to_img)
            tip = _mm_to_pixel(tip_x_mm, tip_y_mm, h_mm_to_img)
            if p is not None:
                # Centre du robot (point rouge plein)
                cv2.circle(frame, p, 5, (0, 0, 255), -1)
                # Cercle extérieur vert
                cv2.circle(frame, p, 14, (0, 255, 0), 2)
                # Flèche de heading
                if tip is not None:
                    cv2.arrowedLine(frame, p, tip, (0, 255, 0), 3, tipLength=0.3)
                cv2.putText(frame, f"ROBOT {robot_heading_deg:.0f}deg",
                            (p[0] + 18, p[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if aerial is not None:
            p = _mm_to_aerial(robot_pos.x, robot_pos.y)
            tip = _mm_to_aerial(tip_x_mm, tip_y_mm)
            # Centre du robot (point rouge plein)
            cv2.circle(aerial, p, 4, (0, 0, 255), -1)
            # Cercle extérieur vert
            cv2.circle(aerial, p, 10, (0, 255, 0), 2)
            # Flèche de heading
            cv2.arrowedLine(aerial, p, tip, (0, 255, 0), 2, tipLength=0.3)
            cv2.putText(aerial, f"ROBOT {robot_heading_deg:.0f}deg",
                        (p[0] + 12, p[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)


def _draw_target_marker(
    img: np.ndarray,
    center: tuple[int, int],
    label: str,
    color: tuple[int, int, int],
    size: int = 12,
) -> None:
    """Dessine un losange (diamond) avec label pour marquer un objectif."""
    cx, cy = center
    pts = np.array([
        [cx, cy - size],
        [cx + size, cy],
        [cx, cy + size],
        [cx - size, cy],
    ], dtype=np.int32)
    cv2.polylines(img, [pts], True, color, 2)
    cv2.circle(img, center, 3, color, -1)
    cv2.putText(img, label, (cx + size + 4, cy + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
