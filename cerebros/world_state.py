"""
cerebros/world_state.py — État du monde.

Centralise les informations sur tous les objets détectés,
les robots et les obstacles, mis à jour par la vision.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

from cerebros import config
from cerebros.config import DEBUG
from cerebros.models import (
    ObjectInfo, ObjectType, Position, Team,
)
from cerebros.robot_state import RobotState


def _classify_marker(marker_id: int, team: Team) -> ObjectType:
    """Détermine le type d'un marqueur ArUco en fonction de l'ID et de l'équipe."""
    if marker_id in config.CORNER_IDS:
        return ObjectType.CALIBRATION

    # IDs à éviter explicitement (ex: A41)
    if marker_id in config.AVOID_IDS:
        return ObjectType.ROBOT_AVOID

    # Robots bleus (1-5)
    if marker_id in config.BLUE_ROBOT_IDS:
        if team == Team.BLUE:
            return ObjectType.ROBOT_ALLY
        return ObjectType.ROBOT_ENEMY

    # Robots jaunes (6-10)
    if marker_id in config.YELLOW_ROBOT_IDS:
        if team == Team.YELLOW:
            return ObjectType.ROBOT_ALLY
        return ObjectType.ROBOT_ENEMY

    # Objets bleus → GOAL (à prendre)
    if marker_id in config.BLUE_OBJECT_IDS:
        return ObjectType.GOAL
    # Objets jaunes → GOAL (à prendre)
    if marker_id in config.YELLOW_OBJECT_IDS:
        return ObjectType.GOAL

    # Objets noirs → à éviter
    if marker_id in config.BLACK_OBJECT_IDS:
        return ObjectType.ROBOT_AVOID

    # Zones/Areas → OBSTACLE par défaut
    if marker_id in config.AREA_IDS:
        return ObjectType.OBSTACLE

    return ObjectType.UNKNOWN


def _label_from_id(marker_id: int) -> str:
    """Génère un label lisible depuis un marker ID (compatible markers.py)."""
    if marker_id in config.CORNER_IDS:
        return f"TABLE{marker_id}"
    if 1 <= marker_id <= 5:
        return f"BR{marker_id}"
    if 6 <= marker_id <= 10:
        return f"YR{marker_id - 5}"
    if marker_id in config.BLUE_OBJECT_IDS:
        return f"BLUE{marker_id}"
    if marker_id in config.YELLOW_OBJECT_IDS:
        return f"YELLOW{marker_id}"
    if marker_id in config.BLACK_OBJECT_IDS:
        return f"BLACK{marker_id}"
    if 11 <= marker_id <= 50:
        return f"AREA{marker_id}"
    return f"ARUCO{marker_id}"


def grid_to_mm(grid_x: int, grid_y: int) -> Position:
    """Convertit coordonnées grille → mm (centre de la cellule)."""
    x_mm = (grid_x + 0.5) * config.CELL_W_MM
    y_mm = (grid_y + 0.5) * config.CELL_H_MM
    return Position(x_mm, y_mm)


class WorldState:
    """État complet du monde : objets, obstacles, robots."""

    def __init__(self, team: Team):
        self.team = team
        self.objects: Dict[int, ObjectInfo] = {}       # marker_id → ObjectInfo
        self.our_robot: Optional[RobotState] = None
        self.ally_robots: Dict[str, RobotState] = {}
        self.enemy_robots: Dict[str, RobotState] = {}
        self._stale_timeout_s = 5.0                    # objets non vus → stale

        if DEBUG:
            print(f"[WorldState] Initialisé pour l'équipe {team.value}")

    # ── Configuration du robot principal ──────────────────────────────────

    def set_our_robot(self, robot: RobotState) -> None:
        self.our_robot = robot
        if DEBUG:
            print(f"[WorldState] Robot principal défini : {robot.robot_id}")

    # ── Mise à jour depuis la vision ──────────────────────────────────────

    def update_from_vision(self, detections: List[tuple]) -> None:
        """Met à jour le monde depuis les détections de la caméra.

        Args:
            detections: liste de tuples (label, grid_x, grid_y)
                        comme envoyés par marker_detection/markers.py
        """
        now = time.time()
        seen_ids: set = set()

        for label, grid_x, grid_y in detections:
            marker_id = self._extract_marker_id(label)
            if marker_id is None:
                if DEBUG:
                    print(f"[WorldState] WARN: impossible d'extraire l'ID de '{label}'")
                continue

            seen_ids.add(marker_id)
            pos_mm = grid_to_mm(grid_x, grid_y)
            obj_type = _classify_marker(marker_id, self.team)

            # ── Mise à jour des robots ────────────────────────────────
            if obj_type == ObjectType.ROBOT_ALLY:
                self._update_ally_robot(marker_id, label, pos_mm)
                continue
            if obj_type == ObjectType.ROBOT_ENEMY:
                self._update_enemy_robot(marker_id, label, pos_mm)
                continue

            # ── Mise à jour / création d'un objet ────────────────────
            # Markers multi-instance (36, 51) : clé = (marker_id, grid_x, grid_y)
            # Markers uniques : clé = marker_id
            if marker_id in config.MULTI_INSTANCE_IDS:
                obj_key = (marker_id, grid_x, grid_y)
            else:
                obj_key = marker_id

            if obj_key in self.objects:
                obj = self.objects[obj_key]
                obj.position = pos_mm
                obj.grid_x = grid_x
                obj.grid_y = grid_y
                obj.last_seen = now
            else:
                obj = ObjectInfo(
                    marker_id=marker_id,
                    label=label,
                    obj_type=obj_type,
                    position=pos_mm,
                    last_seen=now,
                    grid_x=grid_x,
                    grid_y=grid_y,
                )
                self.objects[obj_key] = obj
                if DEBUG:
                    print(f"[WorldState] Nouvel objet détecté : {obj}")

        if DEBUG:
            print(f"[WorldState] Update: {len(detections)} détections, "
                  f"{len(self.objects)} objets en mémoire")

    def _update_ally_robot(self, marker_id: int, label: str,
                           pos_mm: Position) -> None:
        rid = label
        if rid not in self.ally_robots:
            self.ally_robots[rid] = RobotState(
                rid, self.team, initial_pos=pos_mm)
        else:
            self.ally_robots[rid].update_from_vision(pos_mm.x, pos_mm.y)

        # Si c'est notre robot principal, mettre à jour aussi
        if self.our_robot and self.our_robot.robot_id == rid:
            self.our_robot.update_from_vision(pos_mm.x, pos_mm.y)

    def _update_enemy_robot(self, marker_id: int, label: str,
                            pos_mm: Position) -> None:
        enemy_team = Team.YELLOW if self.team == Team.BLUE else Team.BLUE
        rid = label
        if rid not in self.enemy_robots:
            self.enemy_robots[rid] = RobotState(
                rid, enemy_team, initial_pos=pos_mm)
        else:
            self.enemy_robots[rid].update_from_vision(pos_mm.x, pos_mm.y)

    # ── Requêtes ──────────────────────────────────────────────────────────

    def get_goals(self) -> List[ObjectInfo]:
        """Retourne tous les objectifs (GOAL) visibles et non stale."""
        now = time.time()
        return [
            obj for obj in self.objects.values()
            if obj.obj_type == ObjectType.GOAL
            and (now - obj.last_seen) < self._stale_timeout_s
        ]

    def get_obstacles(self) -> List[ObjectInfo]:
        """Retourne tous les obstacles (OBSTACLE + ROBOT_AVOID + ennemis)."""
        now = time.time()
        obstacles: List[ObjectInfo] = []

        # Objets statiques
        for obj in self.objects.values():
            if obj.obj_type in (ObjectType.OBSTACLE, ObjectType.ROBOT_AVOID):
                if (now - obj.last_seen) < self._stale_timeout_s:
                    obstacles.append(obj)

        # Robots ennemis comme obstacles dynamiques
        for enemy in self.enemy_robots.values():
            obstacles.append(ObjectInfo(
                marker_id=-1,
                label=f"enemy_{enemy.robot_id}",
                obj_type=ObjectType.OBSTACLE,
                position=enemy.position,
                last_seen=enemy.last_update,
            ))

        return obstacles

    def get_all_objects(self) -> List[ObjectInfo]:
        return list(self.objects.values())

    def remove_object(self, marker_id: int) -> None:
        """Retire un objet du monde (ex: objet ramassé).

        Pour les markers multi-instance, retire toutes les instances.
        """
        keys_to_remove = [
            k for k in self.objects
            if (k == marker_id) or (isinstance(k, tuple) and k[0] == marker_id)
        ]
        for k in keys_to_remove:
            obj = self.objects.pop(k)
            if DEBUG:
                print(f"[WorldState] Objet retiré : {obj}")

    # ── Helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _extract_marker_id(label: str) -> Optional[int]:
        """Extrait l'ID numérique d'un label comme 'BR1', 'BLUE55', 'AREA15'."""
        import re
        match = re.search(r'(\d+)$', label)
        if match:
            raw_id = int(match.group(1))
            # YR labels use marker_id - 5, so reverse it
            if label.startswith("YR"):
                return raw_id + 5
            return raw_id
        return None

    def dump(self) -> None:
        """Affiche l'état complet du monde (debug)."""
        print("=" * 60)
        print(f"[WorldState] Équipe: {self.team.value}")
        print(f"  Robot principal: {self.our_robot}")
        print(f"  Alliés:  {list(self.ally_robots.values())}")
        print(f"  Ennemis: {list(self.enemy_robots.values())}")
        print(f"  Objets ({len(self.objects)}):")
        for obj in self.objects.values():
            print(f"    {obj}")
        print("=" * 60)
