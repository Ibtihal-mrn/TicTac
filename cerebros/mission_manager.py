"""
cerebros/mission_manager.py — Gestionnaire de missions.

Responsable de :
  - Générer des missions à partir des goals détectés
  - Choisir la prochaine mission (priorité + distance)
  - Tenir une mémoire des missions déjà accomplies
  - Éviter de refaire les mêmes missions
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Set

from cerebros.config import DEBUG, GOAL_REACHED_THRESHOLD_MM
from cerebros.models import (
    Mission, MissionStatus, ObjectInfo, ObjectType, Position,
)


class MissionManager:
    """Gère la file de missions et la mémoire."""

    def __init__(self):
        self._missions: Dict[int, Mission] = {}       # mission_id → Mission
        self._completed_ids: Set[int] = set()          # marker_ids déjà traités
        self._failed_ids: Set[int] = set()             # marker_ids échoués
        self._next_mission_id = 1
        self._active_mission: Optional[Mission] = None

        if DEBUG:
            print("[MissionManager] Initialisé — aucune mission en cours")

    # ── Génération de missions ────────────────────────────────────────────

    def generate_missions_from_goals(self, goals: List[ObjectInfo],
                                     robot_pos: Position) -> None:
        """Crée des missions pour chaque goal non encore traité."""
        for goal in goals:
            mid = goal.marker_id

            # Déjà complétée ou échouée ? → skip
            if mid in self._completed_ids:
                continue
            if mid in self._failed_ids:
                continue

            # Mission déjà existante pour ce marker ?
            if any(m.target.marker_id == mid and m.status == MissionStatus.PENDING
                   for m in self._missions.values()):
                continue

            # Créer la mission
            priority = self._compute_priority(goal, robot_pos)
            mission = Mission(
                mission_id=self._next_mission_id,
                target=goal,
                priority=priority,
                requires_deploy=(goal.obj_type == ObjectType.GOAL),
            )
            self._missions[mission.mission_id] = mission
            self._next_mission_id += 1

            if DEBUG:
                print(f"[MissionManager] Nouvelle mission créée : {mission}")

    def _compute_priority(self, goal: ObjectInfo,
                          robot_pos: Position) -> int:
        """Priorité = inverse de la distance (plus proche → plus prioritaire).

        Retourne un score entier. Les goals très proches ont un score élevé.
        """
        dist = robot_pos.distance_to(goal.position)
        # Score basé sur 3000 (diagonale table ~ 3606mm)
        return max(1, int(3600 - dist))

    # ── Sélection de la prochaine mission ─────────────────────────────────

    def get_next_mission(self, robot_pos: Position) -> Optional[Mission]:
        """Retourne la mission PENDING la plus prioritaire, ou None."""
        pending = [
            m for m in self._missions.values()
            if m.status == MissionStatus.PENDING
        ]

        if not pending:
            if DEBUG:
                print("[MissionManager] Aucune mission PENDING")
            return None

        # Trier par priorité décroissante, puis distance croissante
        pending.sort(
            key=lambda m: (-m.priority, robot_pos.distance_to(m.target.position))
        )

        chosen = pending[0]
        chosen.status = MissionStatus.ACTIVE
        self._active_mission = chosen

        if DEBUG:
            dist = robot_pos.distance_to(chosen.target.position)
            print(f"[MissionManager] Mission sélectionnée : {chosen} "
                  f"(dist={dist:.0f}mm)")

        return chosen

    # ── Complétion / Échec ────────────────────────────────────────────────

    def complete_mission(self, mission: Mission) -> None:
        """Marque une mission comme complétée."""
        mission.status = MissionStatus.COMPLETED
        mission.completed_at = time.time()
        self._completed_ids.add(mission.target.marker_id)

        if self._active_mission and self._active_mission.mission_id == mission.mission_id:
            self._active_mission = None

        if DEBUG:
            print(f"[MissionManager] ✓ Mission complétée : {mission}")
            print(f"[MissionManager] Mémoire: {len(self._completed_ids)} goals faits, "
                  f"{len(self._failed_ids)} échoués")

    def fail_mission(self, mission: Mission) -> None:
        """Marque une mission comme échouée."""
        mission.status = MissionStatus.FAILED
        self._failed_ids.add(mission.target.marker_id)

        if self._active_mission and self._active_mission.mission_id == mission.mission_id:
            self._active_mission = None

        if DEBUG:
            print(f"[MissionManager] ✗ Mission échouée : {mission}")

    def skip_mission(self, mission: Mission) -> None:
        """Skip une mission (cible disparue, inaccessible, etc.)."""
        mission.status = MissionStatus.SKIPPED

        if self._active_mission and self._active_mission.mission_id == mission.mission_id:
            self._active_mission = None

        if DEBUG:
            print(f"[MissionManager] ⊘ Mission skip : {mission}")

    # ── Vérification goal atteint ─────────────────────────────────────────

    def is_goal_reached(self, robot_pos: Position, mission: Mission) -> bool:
        """Vérifie si le robot est assez proche du goal."""
        dist = robot_pos.distance_to(mission.target.position)
        reached = dist < GOAL_REACHED_THRESHOLD_MM

        if DEBUG and reached:
            print(f"[MissionManager] Goal atteint! dist={dist:.0f}mm < "
                  f"seuil={GOAL_REACHED_THRESHOLD_MM}mm")

        return reached

    # ── Accesseurs ────────────────────────────────────────────────────────

    @property
    def active_mission(self) -> Optional[Mission]:
        return self._active_mission

    @property
    def completed_count(self) -> int:
        return len(self._completed_ids)

    @property
    def pending_count(self) -> int:
        return sum(1 for m in self._missions.values()
                   if m.status == MissionStatus.PENDING)

    def is_marker_done(self, marker_id: int) -> bool:
        return marker_id in self._completed_ids

    def get_all_missions(self) -> List[Mission]:
        return list(self._missions.values())

    def dump(self) -> None:
        """Affiche l'état des missions (debug)."""
        print("-" * 50)
        print(f"[MissionManager] Total: {len(self._missions)} missions")
        print(f"  Complétées: {len(self._completed_ids)}")
        print(f"  Échouées:   {len(self._failed_ids)}")
        print(f"  Active:     {self._active_mission}")
        for m in self._missions.values():
            print(f"    {m}")
        print("-" * 50)
