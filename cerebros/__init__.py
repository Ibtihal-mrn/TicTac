"""
cerebros — Module cerveau central du robot Eurobot.

Architecture :
  Vision → WorldState → MissionManager → Planner → ActionQueue → Executor

Modules :
  config.py         Constantes et configuration
  models.py         Dataclasses (Position, ObjectInfo, Action, Mission…)
  world_state.py    État du monde (objets, obstacles, robots)
  robot_state.py    État interne du robot
  mission_manager.py Gestion des missions + mémoire
  planner.py        Planification trajectoire + évitement
  actions.py        ActionQueue (file d'attente)
  executor.py       Exécution des actions (sans BLE)
  brain.py          Boucle principale temps réel
"""

from cerebros.models import (
    Action,
    ActionType,
    Mission,
    MissionStatus,
    ObjectInfo,
    ObjectType,
    Position,
    RobotStatus,
    Team,
)
from cerebros.brain import Brain

__all__ = [
    "Brain",
    "Action",
    "ActionType",
    "Mission",
    "MissionStatus",
    "ObjectInfo",
    "ObjectType",
    "Position",
    "RobotStatus",
    "Team",
]
