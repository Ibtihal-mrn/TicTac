"""Geometrie de la table: selection de coins et matrices de perspective."""

from __future__ import annotations

import cv2
import numpy as np

from marker_detection import config

_LEARNED_MARKER_SIZE_MM: float | None = None


def _table_mm_corners() -> np.ndarray:
    return np.float32(
        [
            [0.0, 0.0],
            [config.TABLE_W_MM, 0.0],
            [config.TABLE_W_MM, config.TABLE_H_MM],
            [0.0, config.TABLE_H_MM],
        ]
    )


def _inner_table_mm_corners() -> np.ndarray:
    inset = float(config.ARUCO_INSET_MM)
    width = float(config.TABLE_W_MM)
    height = float(config.TABLE_H_MM)

    return np.float32(
        [
            [inset, inset],
            [width - inset, inset],
            [width - inset, height - inset],
            [inset, height - inset],
        ]
    )


def _order_quad_points(marker_pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(marker_pts, dtype=np.float32)
    sums = pts[:, 0] + pts[:, 1]
    diffs = pts[:, 0] - pts[:, 1]

    return np.array(
        [
            pts[int(np.argmin(sums))],
            pts[int(np.argmax(diffs))],
            pts[int(np.argmax(sums))],
            pts[int(np.argmin(diffs))],
        ],
        dtype=np.float32,
    )


def _resolve_marker_size_mm() -> float | None:
    configured_size = getattr(config, "ARUCO_MARKER_SIZE_MM", None)
    if configured_size is not None and configured_size > 0:
        return float(configured_size)

    return _LEARNED_MARKER_SIZE_MM


def _marker_world_corners(marker_id: int, marker_size_mm: float) -> np.ndarray | None:
    width = float(config.TABLE_W_MM)
    height = float(config.TABLE_H_MM)
    inset = float(config.ARUCO_INSET_MM)
    size = float(marker_size_mm)

    if size <= 0:
        return None

    if marker_id == 23:  # TL
        x0, y0 = inset, inset
        x1, y1 = inset + size, inset + size
    elif marker_id == 22:  # TR
        x0, y0 = width - inset - size, inset
        x1, y1 = width - inset, inset + size
    elif marker_id == 20:  # BR
        x0, y0 = width - inset - size, height - inset - size
        x1, y1 = width - inset, height - inset
    elif marker_id == 21:  # BL
        x0, y0 = inset, height - inset - size
        x1, y1 = inset + size, height - inset
    else:
        return None

    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32)


def _select_anchor_points(corners_by_id: dict[int, np.ndarray]) -> np.ndarray:
    positions = ["TL", "TR", "BR", "BL"]
    aruco_pts = np.full((len(config.CORNER_ORDER), 2), np.nan, dtype=np.float32)

    for index, (mid, pos) in enumerate(zip(config.CORNER_ORDER, positions)):
        if mid not in corners_by_id:
            continue

        marker_pts = corners_by_id[mid][0]
        aruco_pts[index] = select_table_corner_point(marker_pts, pos)

    return aruco_pts


def _build_marker_correspondences(
    corners_by_id: dict[int, np.ndarray], marker_size_mm: float
) -> tuple[np.ndarray | None, np.ndarray | None]:
    image_points: list[np.ndarray] = []
    world_points: list[np.ndarray] = []
    visible_markers = 0

    for marker_id in config.CORNER_ORDER:
        if marker_id not in corners_by_id:
            continue

        world_corners = _marker_world_corners(marker_id, marker_size_mm)
        if world_corners is None:
            continue

        visible_markers += 1
        image_points.extend(_order_quad_points(corners_by_id[marker_id][0]))
        world_points.extend(world_corners)

    if visible_markers < 2:
        return None, None

    return np.array(image_points, dtype=np.float32), np.array(world_points, dtype=np.float32)


def _build_anchor_correspondences(corners_by_id: dict[int, np.ndarray]) -> tuple[np.ndarray | None, np.ndarray | None]:
    aruco_pts = _select_anchor_points(corners_by_id)
    visible_mask = np.isfinite(aruco_pts).all(axis=1)

    if int(np.count_nonzero(visible_mask)) < 2:
        return None, None

    return aruco_pts[visible_mask].astype(np.float32), _inner_table_mm_corners()[visible_mask].astype(np.float32)


def _similarity_transform_from_two_points(src: np.ndarray, dst: np.ndarray) -> np.ndarray | None:
    src_vec = src[1] - src[0]
    dst_vec = dst[1] - dst[0]

    src_norm = float(np.linalg.norm(src_vec))
    dst_norm = float(np.linalg.norm(dst_vec))
    if src_norm <= 1e-6 or dst_norm <= 1e-6:
        return None

    src_unit = src_vec / src_norm
    dst_unit = dst_vec / dst_norm

    cos_angle = float(np.dot(src_unit, dst_unit))
    sin_angle = float(src_unit[0] * dst_unit[1] - src_unit[1] * dst_unit[0])
    scale = dst_norm / src_norm

    linear = scale * np.array(
        [[cos_angle, -sin_angle], [sin_angle, cos_angle]],
        dtype=np.float64,
    )
    translation = dst[0].astype(np.float64) - linear @ src[0].astype(np.float64)

    return np.array(
        [
            [linear[0, 0], linear[0, 1], translation[0]],
            [linear[1, 0], linear[1, 1], translation[1]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _approximate_mm_transform_from_anchors(corners_by_id: dict[int, np.ndarray]) -> np.ndarray | None:
    image_points, world_points = _build_anchor_correspondences(corners_by_id)
    if image_points is None or world_points is None:
        return None

    if len(image_points) == 2:
        return _similarity_transform_from_two_points(image_points, world_points)

    affine = cv2.getAffineTransform(image_points[:3], world_points[:3])
    return np.array(
        [
            [affine[0, 0], affine[0, 1], affine[0, 2]],
            [affine[1, 0], affine[1, 1], affine[1, 2]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _estimate_marker_size_mm(corners_by_id: dict[int, np.ndarray], h_img_to_mm: np.ndarray) -> float | None:
    estimates: list[float] = []

    for marker_id in config.CORNER_ORDER:
        if marker_id not in corners_by_id:
            continue

        image_corners = _order_quad_points(corners_by_id[marker_id][0]).reshape(-1, 1, 2)
        world_corners = cv2.perspectiveTransform(image_corners, h_img_to_mm).reshape(-1, 2)
        side_lengths = [
            float(np.linalg.norm(world_corners[(index + 1) % 4] - world_corners[index]))
            for index in range(4)
        ]
        mean_side = float(np.mean(side_lengths))
        if np.isfinite(mean_side) and mean_side > 0:
            estimates.append(mean_side)

    if not estimates:
        return None

    return float(np.mean(estimates))


def _update_learned_marker_size(corners_by_id: dict[int, np.ndarray], h_img_to_mm: np.ndarray) -> None:
    global _LEARNED_MARKER_SIZE_MM

    estimated_size = _estimate_marker_size_mm(corners_by_id, h_img_to_mm)
    if estimated_size is not None:
        _LEARNED_MARKER_SIZE_MM = estimated_size


def _compose_transforms(
    h_img_to_mm: np.ndarray,
    table_pts: np.ndarray,
    aruco_pts: np.ndarray | None,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    h_mm_to_grid = np.array(
        [
            [config.GRID_COLS / config.TABLE_W_MM, 0.0, 0.0],
            [0.0, config.GRID_ROWS / config.TABLE_H_MM, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    h_mm_to_aerial = np.array(
        [
            [config.AERIAL_W / config.TABLE_W_MM, 0.0, 0.0],
            [0.0, config.AERIAL_H / config.TABLE_H_MM, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    h_img_to_grid = h_mm_to_grid @ h_img_to_mm
    h_img_to_aerial = h_mm_to_aerial @ h_img_to_mm

    try:
        h_grid_to_img = np.linalg.inv(h_img_to_grid)
    except np.linalg.LinAlgError:
        return None, None, None, None, None

    return h_img_to_grid, h_grid_to_img, h_img_to_aerial, table_pts, aruco_pts

# top-left, top-right, bottom-right, bottom-left
def select_table_corner_point(marker_pts: np.ndarray, position: str) -> np.ndarray:
    """Choisit le coin du marqueur qui correspond au coin reel de table."""
    if position == "TL":
        idx = int(np.argmin(marker_pts[:, 0] + marker_pts[:, 1]))
    elif position == "TR":
        idx = int(np.argmin(marker_pts[:, 1] - marker_pts[:, 0]))
    elif position == "BR":
        idx = int(np.argmax(marker_pts[:, 0] + marker_pts[:, 1]))
    else:  # BL
        idx = int(np.argmax(marker_pts[:, 1] - marker_pts[:, 0]))

    return marker_pts[idx]


def select_table_points(corners_by_id: dict[int, np.ndarray]) -> np.ndarray | None:
    """Selectionne les 4 points ArUco utilises comme base d'extrapolation."""
    if not all(mid in corners_by_id for mid in config.CORNER_ORDER):
        return None

    positions = ["TL", "TR", "BR", "BL"]
    aruco_pts: list[np.ndarray] = []
    for mid, pos in zip(config.CORNER_ORDER, positions):
        marker_pts = corners_by_id[mid][0]
        aruco_pts.append(select_table_corner_point(marker_pts, pos))

    points = np.array(aruco_pts, dtype=np.float32)
    if abs(cv2.contourArea(points)) < 1000:
        return None

    return points


def extrapolate_table_corners(aruco_pts: np.ndarray) -> np.ndarray:
    """Extrapole les 4 coins reels de table depuis la zone ArUco interieure."""
    top_vec = aruco_pts[1] - aruco_pts[0]
    right_vec = aruco_pts[2] - aruco_pts[1]
    bottom_vec = aruco_pts[2] - aruco_pts[3]
    left_vec = aruco_pts[3] - aruco_pts[0]

    width_ratio = config.TABLE_W_MM / (config.TABLE_W_MM - 2 * config.ARUCO_INSET_MM)
    height_ratio = config.TABLE_H_MM / (config.TABLE_H_MM - 2 * config.ARUCO_INSET_MM)

    width_ext = (width_ratio - 1) / 2
    height_ext = (height_ratio - 1) / 2

    table_tl = aruco_pts[0] - width_ext * top_vec - height_ext * left_vec
    table_tr = aruco_pts[1] + width_ext * top_vec - height_ext * right_vec
    table_br = aruco_pts[2] + width_ext * bottom_vec + height_ext * right_vec
    table_bl = aruco_pts[3] - width_ext * bottom_vec + height_ext * left_vec

    return np.array([table_tl, table_tr, table_br, table_bl], dtype=np.float32)


def build_transforms(
    corners_by_id: dict[int, np.ndarray],
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """Construit les matrices de perspective image<->grille et image->vue aerienne."""
    aruco_pts = select_table_points(corners_by_id)
    if aruco_pts is None:
        h_img_to_mm = _approximate_mm_transform_from_anchors(corners_by_id)
        if h_img_to_mm is not None:
            try:
                h_mm_to_img = np.linalg.inv(h_img_to_mm)
            except np.linalg.LinAlgError:
                return None, None, None, None, None

            table_pts = cv2.perspectiveTransform(_table_mm_corners().reshape(-1, 1, 2), h_mm_to_img).reshape(-1, 2)
            if abs(cv2.contourArea(table_pts.astype(np.float32))) >= 1000:
                return _compose_transforms(
                    h_img_to_mm,
                    table_pts.astype(np.float32),
                    _select_anchor_points(corners_by_id),
                )

        marker_size_mm = _resolve_marker_size_mm()
        if marker_size_mm is None:
            return None, None, None, None, None

        image_points, world_points = _build_marker_correspondences(corners_by_id, marker_size_mm)
        if image_points is None or world_points is None:
            return None, None, None, None, None

        h_img_to_mm, _ = cv2.findHomography(image_points, world_points, method=0)
        if h_img_to_mm is None:
            return None, None, None, None, None

        try:
            h_mm_to_img = np.linalg.inv(h_img_to_mm)
        except np.linalg.LinAlgError:
            return None, None, None, None, None

        table_pts = cv2.perspectiveTransform(_table_mm_corners().reshape(-1, 1, 2), h_mm_to_img).reshape(-1, 2)
        if abs(cv2.contourArea(table_pts.astype(np.float32))) < 1000:
            return None, None, None, None, None

        return _compose_transforms(h_img_to_mm, table_pts.astype(np.float32), _select_anchor_points(corners_by_id))

    table_pts = extrapolate_table_corners(aruco_pts)
    h_img_to_mm = cv2.getPerspectiveTransform(table_pts, _table_mm_corners())
    _update_learned_marker_size(corners_by_id, h_img_to_mm)

    return _compose_transforms(h_img_to_mm, table_pts, _select_anchor_points(corners_by_id))


def to_cell(px: float, py: float, h_img_to_grid: np.ndarray | None) -> tuple[float, float] | None:
    """Projette un point image dans le repere grille (colonne, ligne)."""
    if h_img_to_grid is None:
        return None

    out = cv2.perspectiveTransform(np.float32([[[px, py]]]), h_img_to_grid)
    gx, gy = float(out[0, 0, 0]), float(out[0, 0, 1])
    gx = config.GRID_COLS - gx  # Inverser l'axe horizontal pour top-right = (0,0)

    if gx < -0.5 or gx > config.GRID_COLS + 0.5 or gy < -0.5 or gy > config.GRID_ROWS + 0.5:
        return None

    return gx, gy
