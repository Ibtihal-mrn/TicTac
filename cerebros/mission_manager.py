"""
cerebros/mission_manager.py — Gestionnaire de missions par coordonnées.

Responsable de :
  - Stocker une liste ordonnée de coordonnées objectifs (la route)
  - Suivre la progression : aller vers A → A ok → aller vers B → B ok → ...
  - Fournir le prochain objectif à atteindre
  - Permettre de recalculer la route depuis la position actuelle
    (en ne gardant que les objectifs non encore atteints)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

from cerebros.config import DEBUG, GOAL_REACHED_THRESHOLD_MM
from cerebros.models import Position


@dataclass
class Waypoint:
    """Un objectif de coordonnées dans la route planifiée."""
    position: Position
    label: str = ""                # description optionnelle (ex: "BLUE36")
    completed: bool = False
    completed_at: Optional[float] = None

    def __repr__(self) -> str:
        status = "DONE" if self.completed else "TODO"
        return f"WP({self.label or self.position}, {status})"


class MissionManager:
    """Gère la route (suite de coordonnées objectifs) et la mémoire."""

    def __init__(self):
        self._route: List[Waypoint] = []
        self._current_index: int = 0       # index du prochain objectif à atteindre
        self._completed_positions: List[Position] = []  # mémoire des positions atteintes

        if DEBUG:
            print("[MissionManager] Initialisé — aucune route définie")

    # ── Définition de la route ────────────────────────────────────────────

    def set_route(self, targets: List[Position],
                  labels: Optional[List[str]] = None) -> None:
        """Définit la route complète (liste de positions objectifs dans l'ordre).

        Appelé pendant la phase init avec les coordonnées calculées par A*.
        """
        self._route = []
        self._current_index = 0

        for i, pos in enumerate(targets):
            label = labels[i] if labels and i < len(labels) else f"T{i + 1}"
            self._route.append(Waypoint(position=pos, label=label))

        if DEBUG:
            print(f"[MissionManager] Route définie: {len(self._route)} objectifs")
            for i, wp in enumerate(self._route):
                print(f"  [{i}] {wp}")

    # ── Progression ───────────────────────────────────────────────────────

    @property
    def current_target(self) -> Optional[Waypoint]:
        """Retourne le prochain objectif à atteindre, ou None si route finie."""
        if self._current_index < len(self._route):
            return self._route[self._current_index]
        return None

    @property
    def current_target_position(self) -> Optional[Position]:
        wp = self.current_target
        return wp.position if wp else None

    def check_and_advance(self, robot_pos: Position) -> bool:
        """Vérifie si le robot a atteint l'objectif courant.

        Si oui, marque comme complété et avance au suivant.
        Returns: True si un objectif a été complété.
        """
        wp = self.current_target
        if wp is None:
            return False

        dist = robot_pos.distance_to(wp.position)
        if dist < GOAL_REACHED_THRESHOLD_MM:
            wp.completed = True
            wp.completed_at = time.time()
            self._completed_positions.append(
                Position(wp.position.x, wp.position.y)
            )

            if DEBUG:
                print(f"[MissionManager] Objectif atteint: {wp} "
                      f"(dist={dist:.0f}mm)")

            self._current_index += 1

            if self._current_index < len(self._route):
                next_wp = self._route[self._current_index]
                if DEBUG:
                    print(f"[MissionManager] Prochain objectif: {next_wp}")
            else:
                if DEBUG:
                    print("[MissionManager] Route terminée!")

            return True

        return False

    # ── Recalcul de route ─────────────────────────────────────────────────

    def get_remaining_targets(self) -> List[Position]:
        """Retourne les positions des objectifs non encore atteints."""
        return [
            wp.position for wp in self._route[self._current_index:]
            if not wp.completed
        ]

    def rebuild_route_from(self, new_targets: List[Position],
                           labels: Optional[List[str]] = None) -> None:
        """Reconstruit la route avec de nouveaux objectifs,
        en conservant la mémoire des objectifs déjà atteints.

        Utilisé après un replan (monitoring a détecté un problème).
        """
        # Conserver la mémoire des waypoints complétés
        completed_part = [wp for wp in self._route if wp.completed]

        new_route = list(completed_part)
        for i, pos in enumerate(new_targets):
            label = labels[i] if labels and i < len(labels) else f"T{len(completed_part) + i + 1}"
            new_route.append(Waypoint(position=pos, label=label))

        self._route = new_route
        self._current_index = len(completed_part)

        if DEBUG:
            print(f"[MissionManager] Route recalculée: "
                  f"{len(completed_part)} faits, "
                  f"{len(new_targets)} restants")

    # ── Accesseurs ────────────────────────────────────────────────────────

    @property
    def is_route_complete(self) -> bool:
        return self._current_index >= len(self._route)

    @property
    def completed_count(self) -> int:
        return sum(1 for wp in self._route if wp.completed)

    @property
    def pending_count(self) -> int:
        return sum(1 for wp in self._route if not wp.completed)

    @property
    def total_count(self) -> int:
        return len(self._route)

    @property
    def progress(self) -> str:
        """Retourne une chaîne de progression : '2/5'."""
        return f"{self.completed_count}/{self.total_count}"

    @property
    def completed_positions(self) -> List[Position]:
        return list(self._completed_positions)

    def dump(self) -> None:
        """Affiche l'état des missions (debug)."""
        print("-" * 50)
        print(f"[MissionManager] Route: {self.progress} "
              f"(index courant: {self._current_index})")
        for i, wp in enumerate(self._route):
            marker = ">>>" if i == self._current_index else "   "
            print(f"  {marker} [{i}] {wp}")
        print("-" * 50)
