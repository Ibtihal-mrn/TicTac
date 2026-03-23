"""
cerebros/brain.py — Boucle principale temps réel du cerveau robot.

Orchestre tous les modules :
  Vision → WorldState → MissionManager → Planner → ActionQueue → Executor

Usage:
    from cerebros.brain import Brain
    from cerebros.models import Team

    brain = Brain(team=Team.BLUE, robot_id="BR1")
    brain.set_send_function(my_ble_send)

    # Depuis la vision (appelé régulièrement) :
    brain.feed_vision([("BR1", 5, 10), ("BLUE55", 20, 8), ...])

    # Boucle principale :
    brain.run()   # ou brain.tick() dans votre propre boucle
"""

from __future__ import annotations

import time
from typing import Callable, List, Optional, Tuple

from cerebros import config
from cerebros.actions import ActionQueue
from cerebros.config import DEBUG
from cerebros.executor import Executor, SendActionFn
from cerebros.mission_manager import MissionManager
from cerebros.models import (
    Action, ActionType, Mission, MissionStatus,
    ObjectType, Position, RobotStatus, Team,
)
from cerebros.planner import Planner
from cerebros.robot_state import RobotState
from cerebros.world_state import WorldState


class Brain:
    """Cerveau central du robot — orchestre décision et action."""

    def __init__(self, team: Team, robot_id: str,
                 initial_pos: Position | None = None,
                 initial_heading: float = 0.0):
        print("=" * 60)
        print(f"[Brain] INITIALISATION — Équipe: {team.value}, "
              f"Robot: {robot_id}")
        print("=" * 60)

        # ── Modules ───────────────────────────────────────────────────
        self.world = WorldState(team)
        self.robot = RobotState(
            robot_id, team,
            initial_pos=initial_pos or Position(150, 1000),
            initial_heading=initial_heading,
        )
        self.world.set_our_robot(self.robot)

        self.mission_mgr = MissionManager()
        self.planner = Planner()
        self.action_queue = ActionQueue()
        self.executor = Executor(self.action_queue, self.robot)

        # ── État interne ──────────────────────────────────────────────
        self._running = False
        self._match_start: Optional[float] = None
        self._tick_count = 0
        self._current_mission: Optional[Mission] = None
        self._needs_replan = False

        print(f"[Brain] Prêt. Robot à {self.robot.position}, "
              f"heading={self.robot.heading_deg}°")

    # ── Configuration ─────────────────────────────────────────────────

    def set_send_function(self, fn: SendActionFn) -> None:
        """Branche la fonction d'envoi BLE."""
        self.executor.set_send_function(fn)
        print("[Brain] Fonction d'envoi BLE configurée")

    # ── Entrée vision ─────────────────────────────────────────────────

    def feed_vision(self, detections: List[Tuple[str, int, int]],
                     robot_heading: float | None = None) -> None:
        """Reçoit les détections de la caméra.

        Args:
            detections: liste de (label, grid_x, grid_y)
            robot_heading: heading du robot en degrés (depuis ArUco corners)
        """
        self.world.update_from_vision(detections)

        # Mettre à jour le heading du robot si la vision le fournit
        if robot_heading is not None and self.robot is not None:
            self.robot.heading_deg = robot_heading
            if config.DEBUG:
                print(f"[Brain] Heading vision: {robot_heading:.1f}°")

        # Vérifier si le monde a changé significativement → replanifier
        if self._current_mission and not self.executor.is_busy:
            self._needs_replan = True

    # ── Tick unique ───────────────────────────────────────────────────

    def tick(self) -> None:
        """Un cycle de décision. Appeler à ~10 Hz."""
        self._tick_count += 1

        # ── Vérifier temps de match ───────────────────────────────────
        if self._match_start is not None:
            elapsed_ms = (time.time() - self._match_start) * 1000
            if elapsed_ms >= config.MATCH_DURATION_MS:
                print("[Brain] ⏰ FIN DU MATCH — STOP")
                self.executor.abort()
                self._running = False
                return

        # ── L'executor a-t-il fini ? ──────────────────────────────────
        executor_busy = self.executor.tick()

        # ── Mission active : vérifier complétion ──────────────────────
        if self._current_mission:
            if self.mission_mgr.is_goal_reached(
                    self.robot.position, self._current_mission):
                # Goal atteint !
                self._on_mission_reached()
                return

            if not executor_busy and self.action_queue.is_empty():
                # Toutes les actions envoyées mais goal pas atteint
                # → la mission est probablement complétée (ou il faut replanifier)
                if DEBUG:
                    print("[Brain] Actions terminées, vérification du goal...")
                dist = self.robot.position.distance_to(
                    self._current_mission.target.position)
                if dist < config.GOAL_REACHED_THRESHOLD_MM * 2:
                    self._on_mission_reached()
                else:
                    # Replanifier
                    self._needs_replan = True

        # ── Besoin de replanifier ? ───────────────────────────────────
        if self._needs_replan and self._current_mission:
            self._needs_replan = False
            self._plan_current_mission()
            return

        # ── Pas de mission active → en chercher une ──────────────────
        if self._current_mission is None and not executor_busy:
            self._select_next_mission()

        # ── Debug périodique ──────────────────────────────────────────
        if DEBUG and self._tick_count % 50 == 0:
            self._debug_status()

    # ── Logique de mission ────────────────────────────────────────────

    def _select_next_mission(self) -> None:
        """Sélectionne et planifie la prochaine mission."""
        # Générer des missions à partir des goals visibles
        goals = self.world.get_goals()
        if goals:
            self.mission_mgr.generate_missions_from_goals(
                goals, self.robot.position
            )

        # Choisir la meilleure mission
        mission = self.mission_mgr.get_next_mission(self.robot.position)
        if mission is None:
            if DEBUG and self._tick_count % 30 == 0:
                print("[Brain] Aucune mission disponible — en attente")
            return

        self._current_mission = mission
        print(f"[Brain] === NOUVELLE MISSION === {mission}")
        self._plan_current_mission()

    def _plan_current_mission(self) -> None:
        """Planifie le chemin vers la mission active."""
        if self._current_mission is None:
            return

        target_pos = self._current_mission.target.position
        obstacles = self.world.get_obstacles()

        print(f"[Brain] Planification vers {target_pos} "
              f"({len(obstacles)} obstacles)")

        # Vider l'ancienne file
        self.action_queue.clear()

        # Planifier le chemin
        path = self.planner.plan_path(
            self.robot.position, target_pos, obstacles
        )

        # Convertir en actions
        actions = self.planner.path_to_actions(
            path,
            self.robot.heading_deg,
            deploy_at_end=self._current_mission.requires_deploy,
        )

        # Remplir la file
        self.action_queue.enqueue_many(actions)

        print(f"[Brain] {len(actions)} actions en file d'attente")

    def _on_mission_reached(self) -> None:
        """Appelé quand le robot atteint le goal de la mission."""
        if self._current_mission is None:
            return

        print(f"[Brain] 🎯 MISSION ACCOMPLIE : {self._current_mission}")
        self.mission_mgr.complete_mission(self._current_mission)

        # Retirer l'objet du monde (ramassé)
        self.world.remove_object(self._current_mission.target.marker_id)
        self._current_mission = None

        # Rétracter le bras après déploiement
        self.action_queue.enqueue(Action(ActionType.RETRACT))

        self.robot.set_status(RobotStatus.IDLE)

    # ── Boucle principale ─────────────────────────────────────────────

    def run(self) -> None:
        """Boucle principale temps réel. Bloquante.

        Appeller dans un thread ou comme programme principal.
        Ctrl+C pour arrêter.
        """
        self._running = True
        self._match_start = time.time()

        print("\n" + "=" * 60)
        print("[Brain] 🚀 DÉMARRAGE DE LA BOUCLE PRINCIPALE")
        print(f"[Brain] Fréquence: {config.BRAIN_LOOP_HZ} Hz "
              f"(dt={config.BRAIN_LOOP_DT_S*1000:.0f}ms)")
        print(f"[Brain] Durée match: {config.MATCH_DURATION_MS/1000:.0f}s")
        print("=" * 60 + "\n")

        try:
            while self._running:
                t_start = time.time()

                self.tick()

                # Respect du timing de la boucle
                elapsed = time.time() - t_start
                sleep_time = config.BRAIN_LOOP_DT_S - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[Brain] Interruption clavier — arrêt")
            self.stop()

    def stop(self) -> None:
        """Arrête proprement le cerveau."""
        print("[Brain] Arrêt en cours...")
        self._running = False
        self.executor.abort()
        print("[Brain] Arrêté.")

    # ── Commandes manuelles (pour debug / UI) ─────────────────────────

    def send_manual_command(self, cmd: str) -> None:
        """Envoie une commande manuelle au robot (bypass la file)."""
        print(f"[Brain] Commande manuelle: '{cmd}'")
        self.executor._send_fn(cmd)

    def force_stop(self) -> None:
        """Arrêt d'urgence."""
        print("[Brain] ⚠ ARRÊT D'URGENCE")
        self.executor.abort()
        self._current_mission = None

    # ── Debug ─────────────────────────────────────────────────────────

    def _debug_status(self) -> None:
        """Affiche un résumé de l'état (appelé périodiquement)."""
        elapsed = ""
        if self._match_start:
            e = time.time() - self._match_start
            elapsed = f" | match: {e:.1f}s"

        print(f"[Brain] tick#{self._tick_count} | "
              f"robot={self.robot.position} heading={self.robot.heading_deg:.0f}° "
              f"status={self.robot.status.name} | "
              f"missions: {self.mission_mgr.completed_count} done, "
              f"{self.mission_mgr.pending_count} pending | "
              f"queue: {self.action_queue.size} actions"
              f"{elapsed}")

    def dump(self) -> None:
        """Dump complet de l'état (debug)."""
        print("\n" + "#" * 60)
        print("# BRAIN DUMP")
        print("#" * 60)
        self.world.dump()
        self.mission_mgr.dump()
        self.action_queue.dump()
        print(f"Robot: {self.robot}")
        print(f"Mission active: {self._current_mission}")
        print("#" * 60 + "\n")
