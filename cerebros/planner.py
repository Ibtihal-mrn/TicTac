"""
cerebros/planner.py — Planificateur de trajectoire.

MVP : ligne droite vers le goal avec évitement simple des obstacles.
Stratégie d'évitement :
  1. Calculer un chemin direct (waypoints en ligne droite)
  2. Pour chaque segment, vérifier s'il croise un obstacle
  3. Si oui, insérer un waypoint de contournement (décalage latéral)
"""

from __future__ import annotations

import math
from typing import List

from cerebros.config import (
    DEBUG,
    OBSTACLE_SAFETY_MARGIN_MM,
    TABLE_H_MM,
    TABLE_W_MM,
)
from cerebros.models import Action, ActionType, ObjectInfo, Position


class Planner:
    """Planificateur de trajectoire simple."""

    def __init__(self):
        if DEBUG:
            print("[Planner] Initialisé — mode ligne droite + évitement basique")

    # ── Planification de trajectoire ──────────────────────────────────────

    def plan_path(self, start: Position, goal: Position,
                  obstacles: List[ObjectInfo]) -> List[Position]:
        """Planifie un chemin de start vers goal en évitant les obstacles.

        Retourne une liste de waypoints (positions en mm).
        MVP : ligne droite, avec détour si obstacle sur le chemin.
        """
        if DEBUG:
            print(f"[Planner] Planification: {start} → {goal}")
            print(f"[Planner] {len(obstacles)} obstacles à éviter")

        # Chemin direct
        path = [Position(start.x, start.y), Position(goal.x, goal.y)]

        # Tentative d'évitement (max 5 itérations pour éviter boucle infinie)
        for iteration in range(5):
            collision_found = False
            new_path: List[Position] = [path[0]]

            for i in range(len(path) - 1):
                seg_start = path[i]
                seg_end = path[i + 1]

                blocking = self._find_blocking_obstacle(
                    seg_start, seg_end, obstacles
                )

                if blocking is not None:
                    collision_found = True
                    # Insérer un waypoint de contournement
                    detour = self._compute_detour(
                        seg_start, seg_end, blocking.position
                    )
                    if DEBUG:
                        print(f"[Planner] Obstacle {blocking.label} détecté "
                              f"entre {seg_start} et {seg_end}")
                        print(f"[Planner] Détour via {detour}")
                    new_path.append(detour)

                new_path.append(seg_end)

            path = new_path

            if not collision_found:
                break

        if DEBUG:
            print(f"[Planner] Chemin final: {len(path)} waypoints")
            for i, wp in enumerate(path):
                print(f"  [{i}] {wp}")

        return path

    # ── Détection d'obstacle sur un segment ───────────────────────────────

    def _find_blocking_obstacle(self, seg_start: Position,
                                seg_end: Position,
                                obstacles: List[ObjectInfo]
                                ) -> ObjectInfo | None:
        """Trouve le premier obstacle qui bloque le segment."""
        for obs in obstacles:
            dist = self._point_to_segment_distance(
                obs.position, seg_start, seg_end
            )
            if dist < OBSTACLE_SAFETY_MARGIN_MM:
                return obs
        return None

    @staticmethod
    def _point_to_segment_distance(point: Position,
                                   seg_a: Position,
                                   seg_b: Position) -> float:
        """Distance d'un point à un segment [A, B]."""
        dx = seg_b.x - seg_a.x
        dy = seg_b.y - seg_a.y
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 1e-6:
            return point.distance_to(seg_a)

        # Projection du point sur le segment (t ∈ [0, 1])
        t = ((point.x - seg_a.x) * dx + (point.y - seg_a.y) * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))

        proj = Position(seg_a.x + t * dx, seg_a.y + t * dy)
        return point.distance_to(proj)

    def _compute_detour(self, seg_start: Position, seg_end: Position,
                        obstacle_pos: Position) -> Position:
        """Calcule un waypoint de contournement autour d'un obstacle.

        Stratégie : décalage perpendiculaire au segment, du côté le plus
        éloigné du bord de la table.
        """
        # Vecteur du segment
        dx = seg_end.x - seg_start.x
        dy = seg_end.y - seg_start.y
        seg_len = math.hypot(dx, dy)

        if seg_len < 1e-6:
            return Position(obstacle_pos.x + OBSTACLE_SAFETY_MARGIN_MM,
                            obstacle_pos.y)

        # Vecteur perpendiculaire normalisé (2 options : gauche ou droite)
        nx = -dy / seg_len
        ny = dx / seg_len

        offset = OBSTACLE_SAFETY_MARGIN_MM * 1.5

        # Option A : décalage positif
        detour_a = Position(obstacle_pos.x + nx * offset,
                            obstacle_pos.y + ny * offset)
        # Option B : décalage négatif
        detour_b = Position(obstacle_pos.x - nx * offset,
                            obstacle_pos.y - ny * offset)

        # Choisir le détour qui reste le plus dans la table
        score_a = self._in_table_score(detour_a)
        score_b = self._in_table_score(detour_b)

        chosen = detour_a if score_a >= score_b else detour_b

        # Clamp dans la table
        chosen.x = max(50, min(TABLE_W_MM - 50, chosen.x))
        chosen.y = max(50, min(TABLE_H_MM - 50, chosen.y))

        return chosen

    @staticmethod
    def _in_table_score(pos: Position) -> float:
        """Score : plus le point est loin des bords, plus le score est haut."""
        return min(pos.x, TABLE_W_MM - pos.x, pos.y, TABLE_H_MM - pos.y)

    # ── Conversion path → actions ─────────────────────────────────────────

    def path_to_actions(self, path: List[Position],
                        current_heading_deg: float,
                        deploy_at_end: bool = False) -> List[Action]:
        """Convertit une liste de waypoints en séquence d'actions robot.

        Pour chaque segment :
          1. Calculer l'angle vers le prochain waypoint
          2. Tourner (LEFT/RIGHT) pour s'aligner
          3. Avancer (FORWARD) de la distance du segment

        Args:
            path: liste de waypoints (le premier = position actuelle)
            current_heading_deg: heading actuel du robot en degrés
            deploy_at_end: ajouter DEPLOY à la fin si True

        Returns:
            liste d'Actions à envoyer au robot
        """
        actions: List[Action] = []
        heading = current_heading_deg

        if DEBUG:
            print(f"[Planner] Conversion path→actions "
                  f"({len(path)} waypoints, heading={heading:.1f}°)")

        for i in range(len(path) - 1):
            wp_from = path[i]
            wp_to = path[i + 1]

            # Angle désiré
            desired_angle = wp_from.angle_to(wp_to)
            # Différence d'angle (normaliser dans [-180, 180])
            delta = self._normalize_angle(desired_angle - heading)

            distance = wp_from.distance_to(wp_to)

            if DEBUG:
                print(f"  Segment [{i}→{i+1}]: {wp_from} → {wp_to}")
                print(f"    angle_désiré={desired_angle:.1f}°, "
                      f"heading={heading:.1f}°, delta={delta:.1f}°, "
                      f"dist={distance:.0f}mm")

            # ── Rotation si nécessaire ────────────────────────────────
            if abs(delta) > 5:  # tolérance 5°
                if delta > 0:
                    actions.append(Action(ActionType.LEFT, abs(delta)))
                    if DEBUG:
                        print(f"    → LEFT {abs(delta):.0f}°")
                else:
                    actions.append(Action(ActionType.RIGHT, abs(delta)))
                    if DEBUG:
                        print(f"    → RIGHT {abs(delta):.0f}°")

                heading = desired_angle

            # ── Avancer ───────────────────────────────────────────────
            if distance > 10:  # pas la peine d'avancer pour < 10mm
                actions.append(Action(ActionType.FORWARD, distance))
                if DEBUG:
                    print(f"    → FORWARD {distance:.0f}mm")

        # ── Deploy à la fin si demandé ────────────────────────────────
        if deploy_at_end:
            actions.append(Action(ActionType.DEPLOY))
            if DEBUG:
                print(f"    → DEPLOY")

        if DEBUG:
            print(f"[Planner] Total: {len(actions)} actions générées")

        return actions

    @staticmethod
    def _normalize_angle(angle_deg: float) -> float:
        """Normalise un angle dans [-180, 180]."""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg
