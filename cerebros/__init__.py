"""
cerebros — Module cerveau central du robot Eurobot.

Architecture :
  Vision → WorldState → Brain.init_plan() [A*] → ActionQueue → Executor
  Tirette → Brain.run() [monitoring + replan si nécessaire]

Modules :
  config.py          Constantes et configuration
  models.py          Dataclasses (Position, ObjectInfo, Action…)
  world_state.py     État du monde (objets, obstacles, robots)
  robot_state.py     État interne du robot
  mission_manager.py Suivi de route par coordonnées + mémoire
  astar.py           Pathfinding A* sur grille
  planner.py         Planification trajectoire A* + conversion en actions
  actions.py         ActionQueue (file d'attente)
  executor.py        Exécution des actions (sans BLE)
  brain.py           Orchestration : init_plan → start_match → monitoring
"""

from cerebros.models import (
    Action,
    ActionType,
    ObjectInfo,
    ObjectType,
    Position,
    RobotStatus,
    Team,
)
from cerebros.brain import Brain, BrainPhase

__all__ = [
    "Brain",
    "BrainPhase",
    "Action",
    "ActionType",
    "ObjectInfo",
    "ObjectType",
    "Position",
    "RobotStatus",
    "Team",
]
