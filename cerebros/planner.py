"""
cerebros/planner.py — Planificateur de trajectoire A*.

Utilise A* sur la grille de la table pour trouver le chemin optimal
en évitant les obstacles. Convertit ensuite le chemin en actions robot.
"""

from __future__ import annotations

from typing import List

from cerebros.astar import (
    AStarGrid,
    astar,
    mm_to_grid,
    path_cells_to_mm,
    simplify_path,
)
from cerebros.config import DEBUG
from cerebros.models import Action, ActionType, ObjectInfo, Position


class Planner:
    """Planificateur de trajectoire basé sur A*."""

    def __init__(self):
        self._grid = AStarGrid()
        if DEBUG:
            print("[Planner] Initialisé — mode A* grid pathfinding")

    # ── Planification de trajectoire ──────────────────────────────────────

    def plan_path(self, start: Position, goal: Position,
                  obstacles: List[ObjectInfo]) -> List[Position]:
        """Planifie un chemin A* de start vers goal en évitant les obstacles.

        Retourne une liste de waypoints (positions en mm).
        """
        if DEBUG:
            print(f"[Planner] A* planification: {start} → {goal}")
            print(f"[Planner] {len(obstacles)} obstacles à éviter")

        # Préparer la grille
        self._grid.reset()
        self._grid.mark_obstacles(obstacles)

        # Convertir start/goal en cellules grille
        start_cell = mm_to_grid(start.x, start.y)
        goal_cell = mm_to_grid(goal.x, goal.y)

        if DEBUG:
            blocked = self._grid.get_blocked_cells()
            print(f"[Planner] Grille: {len(blocked)} cellules bloquées")
            print(f"[Planner] Start cell: {start_cell}, Goal cell: {goal_cell}")

        # Exécuter A*
        cell_path = astar(self._grid, start_cell, goal_cell)

        if cell_path is None:
            if DEBUG:
                print("[Planner] A* n'a pas trouvé de chemin! "
                      "Fallback: ligne droite")
            return [Position(start.x, start.y), Position(goal.x, goal.y)]

        # Simplifier (supprimer les points colinéaires)
        simplified = simplify_path(cell_path)

        if DEBUG:
            print(f"[Planner] A* brut: {len(cell_path)} cellules → "
                  f"simplifié: {len(simplified)} waypoints")

        # Convertir en mm
        path_mm = path_cells_to_mm(simplified)

        # Remplacer le premier et dernier point par les positions exactes
        path_mm[0] = Position(start.x, start.y)
        path_mm[-1] = Position(goal.x, goal.y)

        if DEBUG:
            print(f"[Planner] Chemin final: {len(path_mm)} waypoints")
            for i, wp in enumerate(path_mm):
                print(f"  [{i}] {wp}")

        return path_mm

    def plan_full_route(self, start: Position, targets: List[Position],
                        obstacles: List[ObjectInfo]) -> List[Position]:
        """Planifie un chemin complet passant par tous les targets dans l'ordre.

        Concatène les chemins A* entre chaque paire de waypoints consécutifs.
        Utilisé pendant la phase init pour pré-calculer toute la route.
        """
        if not targets:
            return [start]

        full_path = []
        current = start

        for i, target in enumerate(targets):
            if DEBUG:
                print(f"[Planner] Segment {i + 1}/{len(targets)}: "
                      f"{current} → {target}")

            segment = self.plan_path(current, target, obstacles)

            # Éviter les doublons au point de jonction
            if full_path and segment:
                segment = segment[1:]

            full_path.extend(segment)
            current = target

        if DEBUG:
            print(f"[Planner] Route complète: {len(full_path)} waypoints "
                  f"pour {len(targets)} cibles")

        return full_path

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
