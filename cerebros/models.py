"""
cerebros/models.py — Structures de données du cerveau robot.

Dataclasses légères utilisées par tous les modules.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


# ── Enums ─────────────────────────────────────────────────────────────────────

# 1. Teams
class Team(Enum):
    BLUE   = "blue"
    YELLOW = "yellow"

# 2. Objects
class ObjectType(Enum):
    """Type d'objet détecté sur la table."""
    GOAL        = auto()   # objectif à ramasser/atteindre
    OBSTACLE    = auto()   # à éviter
    CALIBRATION = auto()   # coin de table
    ROBOT_ALLY  = auto()   # robot de notre équipe
    ROBOT_ENEMY = auto()   # robot adverse
    ROBOT_AVOID = auto()   # tag noir → à éviter
    UNKNOWN     = auto()

# 3. Robot State
class RobotStatus(Enum):    # TODO: sync with robot 
    IDLE       = auto()
    MOVING     = auto()
    TURNING    = auto()
    DEPLOYING  = auto()
    ERROR      = auto()
    STOPPED    = auto()


# 4. Robot Actions
class ActionType(Enum):    # TODO: sync with robot 
    """Commandes connues par le robot ESP32."""
    FORWARD     = "FORWARD"
    BACKWARD    = "BACKWARD"
    ROTATE      = "ROTATE"
    DEPLOY      = "DEPLOY"
    RETRACT     = "RETRACT"
    RELAIS_ON   = "RELAISON"
    RELAIS_OFF  = "RELAISOFF"
    STOP        = "STOP"
    STATUS      = "STATUS"
    RESET       = "RESET"
    CLEAR_QUEUE = "CLEAR_QUEUE"


# ── Position ──────────────────────────────────────────────────────────────────
@dataclass
class Position:
    """ Table Matrix : 
        - Position en mm sur la table (origine = coin haut-gauche).
        - Distance between 2 points on the Table
        - Angle    between 2 points on the Table
    """
    x: float = 0.0
    y: float = 0.0

    def distance_to(self, other: Position) -> float: return math.hypot(self.x - other.x, self.y - other.y)
    def angle_to(self, other: Position) -> float:
        """Angle en degrés de self vers other (0° = axe X+, sens trigo)."""
        return math.degrees(math.atan2(other.y - self.y, other.x - self.x))
    def __repr__(self) -> str: return f"Pos({self.x:.0f}, {self.y:.0f})"


# ── Objet détecté ────────────────────────────────────────────────────────────

@dataclass
class ObjectInfo:
    """Un objet détecté par la vision."""
    marker_id: int
    label: str                       # ex: "BR1", "BLUE55", "AREA15"
    obj_type: ObjectType
    position: Position
    last_seen: float = field(default_factory=time.time)
    grid_x: int = 0
    grid_y: int = 0

    def __repr__(self) -> str:
        return (f"Obj(id={self.marker_id}, label={self.label}, "
                f"type={self.obj_type.name}, pos={self.position})")


# ── Action ────────────────────────────────────────────────────────────────────

@dataclass
class Action:
    """Une commande atomique à envoyer au robot."""
    action_type: ActionType
    value: Optional[float] = None    # mm, angle or wait_ms
    speed: Optional[float] = None    # pmw

    def to_command(self) -> str:
        """Convertit en chaîne BLE (ex: 'FORWARD 200')."""
        if self.value is not None:
            return f"{self.action_type.value} {int(self.value)}"
        return self.action_type.value

    def __repr__(self) -> str:return f"Action({self.to_command()})"
