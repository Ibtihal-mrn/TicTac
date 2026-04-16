"""
cerebros/robot_state.py — État interne du robot.

Gère la position, le heading, le statut et la mémoire du robot.
"""

from __future__ import annotations

import time

from cerebros.config import DEBUG
from cerebros.models import Position, RobotStatus, Team


class RobotState:
    """État complet d'un robot (notre robot ou un allié)."""

    def __init__(self, robot_id: str, team: Team,
                initial_pos: Position | None = None,
                initial_heading: float = 0.0):
        self.robot_id = robot_id
        self.team = team
        self.position = initial_pos or Position(0.0, 0.0)
        self.heading_deg = initial_heading        # 0° = vers X+, sens trigo
        self.status = RobotStatus.IDLE
        self.last_update = time.time()
        self.velocity = 0.0                       # mm/s estimé

        if DEBUG:
            print(f"[RobotState] Créé : id={robot_id}, team={team.value}, "
                  f"pos={self.position}, heading={self.heading_deg}°")

    # ── Mise à jour depuis la vision ──────────────────────────────────────

    def update_from_vision(self, x_mm: float, y_mm: float,
                           heading_deg: float | None = None) -> None:
        """Met à jour la position du robot à partir des données vision."""
        old_pos = Position(self.position.x, self.position.y)
        self.position.x = x_mm
        self.position.y = y_mm

        if heading_deg is not None:
            self.heading_deg = heading_deg

        dt = time.time() - self.last_update
        if dt > 0:
            dist = old_pos.distance_to(self.position)
            self.velocity = dist / dt

        self.last_update = time.time()

        if DEBUG:
            print(f"[RobotState] {self.robot_id} vision update → "
                  f"pos={self.position}, heading={self.heading_deg:.1f}°, "
                  f"vel={self.velocity:.0f} mm/s")

    def set_status(self, status: RobotStatus) -> None:
        old = self.status
        self.status = status
        if DEBUG and old != status:
            print(f"[RobotState] {self.robot_id} status: "
                  f"{old.name} → {status.name}")

    def is_idle(self) -> bool:
        return self.status in (RobotStatus.IDLE, RobotStatus.STOPPED)

    def __repr__(self) -> str:
        return (f"RobotState({self.robot_id}, {self.team.value}, "
                f"pos={self.position}, heading={self.heading_deg:.1f}°, "
                f"status={self.status.name})")
