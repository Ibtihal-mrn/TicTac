"""
cerebros/brain.py — Boucle principale du cerveau robot.

Deux phases :
  1. INIT  — Avant la tirette : la vision détecte la table, on calcule
             les chemins A* et on envoie la queue complète au robot.
  2. RUN   — Après la tirette : monitoring simple — on vérifie que le
             robot exécute correctement, et on replanifie si nécessaire.

Usage:
    from cerebros.brain import Brain
    from cerebros.models import Team

    brain = Brain(team=Team.BLUE, robot_id="BR1")
    brain.set_send_function(my_ble_send)

    # Depuis la vision (appelé régulièrement) :
    brain.feed_vision([("BR1", 5, 10), ("BLUE55", 20, 8), ...])

    # Phase 1 — Init (avant tirette) :
    brain.init_plan()          # calcule A* et prépare la queue

    # Phase 2 — Tirette tirée :
    brain.start_match()
    brain.run()                # boucle de monitoring
"""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import List, Optional, Tuple

import math

from cerebros import config
from cerebros.actions import ActionQueue
from cerebros.config import DEBUG
from cerebros.executor import Executor, SendActionFn
from cerebros.mission_manager import MissionManager
from cerebros.models import Action, ActionType, Position, Team
from cerebros.planner import Planner
from cerebros.robot_state import RobotState
from cerebros.world_state import WorldState


class BrainPhase(Enum):
    """Phase du cerveau."""
    INIT = auto()       # Avant tirette : détection + planification A*
    READY = auto()      # Plan calculé, en attente de la tirette
    RUNNING = auto()    # Match en cours : monitoring
    FINISHED = auto()   # Match terminé


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
        self.phase = BrainPhase.INIT
        self._match_start: Optional[float] = None
        self._tick_count = 0

        # ── Monitoring ────────────────────────────────────────────────
        self._monitoring_deviation_threshold_mm = config.REPLAN_DISTANCE_MM
        self._monitoring_stuck_ticks = 0
        self._monitoring_stuck_threshold = config.MONITORING_STUCK_THRESHOLD
        self._last_robot_pos = Position(self.robot.position.x,
                                        self.robot.position.y)
        self._last_replan_tick = 0
        self._replan_cooldown_ticks = 50   # ~5s à 10Hz entre deux replans

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

        if robot_heading is not None and self.robot is not None:
            self.robot.heading_deg = robot_heading
            if config.DEBUG:
                print(f"[Brain] Heading vision: {robot_heading:.1f}°")

    # ══════════════════════════════════════════════════════════════════
    # PHASE 1 — INIT : Planification A* avant la tirette
    # ══════════════════════════════════════════════════════════════════

    def init_plan(self, target_positions: Optional[List[Position]] = None,
                  target_labels: Optional[List[str]] = None) -> bool:
        """Calcule les chemins A* et prépare la queue d'actions complète.

        Args:
            target_positions: liste ordonnée de positions objectifs (mm).
                              Si None, utilise les goals détectés par la vision.
            target_labels: labels optionnels pour chaque objectif.

        Returns:
            True si le plan a été calculé avec succès.
        """
        print("\n" + "=" * 60)
        print("[Brain] === PHASE INIT — Planification A* ===")
        print("=" * 60)

        # Déterminer les cibles
        if target_positions is None:
            goals = self.world.get_goals()
            if not goals:
                print("[Brain] Aucun goal détecté — impossible de planifier")
                return False
            # Trier par distance depuis le robot (plus proche d'abord)
            goals.sort(key=lambda g: self.robot.position.distance_to(g.position))
            target_positions = [g.position for g in goals]
            target_labels = [g.label for g in goals]
            print(f"[Brain] {len(goals)} goals détectés par la vision")
        else:
            print(f"[Brain] {len(target_positions)} objectifs fournis manuellement")

        # Récupérer les obstacles
        obstacles = self.world.get_obstacles()
        print(f"[Brain] {len(obstacles)} obstacles détectés")

        # Afficher le plan
        print("[Brain] Objectifs dans l'ordre:")
        for i, pos in enumerate(target_positions):
            label = target_labels[i] if target_labels and i < len(target_labels) else ""
            print(f"  [{i + 1}] {pos} {label}")

        # Définir la route dans le mission manager
        self.mission_mgr.set_route(target_positions, target_labels)

        # ── FORWARD initial pour sortir de la zone de départ ─────────
        exit_mm = config.EXIT_ZONE_MM
        heading_rad = math.radians(self.robot.heading_deg)
        start_after_exit = Position(
            self.robot.position.x + exit_mm * math.cos(heading_rad),
            self.robot.position.y + exit_mm * math.sin(heading_rad),
        )
        print(f"[Brain] FORWARD {exit_mm}mm pour sortir de zone → "
              f"position estimée: {start_after_exit}")

        # Calculer le chemin A* depuis la position post-sortie
        full_path = self.planner.plan_full_route(
            start_after_exit, target_positions, obstacles
        )

        if len(full_path) < 2:
            print("[Brain] Chemin trop court — échec de planification")
            return False

        # Convertir en actions
        actions = self.planner.path_to_actions(
            full_path,
            self.robot.heading_deg,
            deploy_at_end=False,
        )

        # Prépendre le FORWARD de sortie de zone
        exit_action = Action(ActionType.FORWARD, exit_mm)
        actions.insert(0, exit_action)
        print(f"[Brain] Action de sortie ajoutée: {exit_action}")

        # Remplir la file d'actions
        self.action_queue.clear()
        self.action_queue.enqueue_many(actions)

        # self.phase = BrainPhase.READY
        print(f"\n[Brain] Plan prêt: {len(actions)} actions en queue")
        print(f"[Brain] Route: {self.mission_mgr.progress}")
        # print("[Brain] En attente de la tirette (start_match)...")
        print("=" * 60 + "\n")

        # Tirette commentée — lancement immédiat
        self.start_match()

        return True

    # ══════════════════════════════════════════════════════════════════
    # PHASE 2 — RUN : Match en cours + monitoring
    # ══════════════════════════════════════════════════════════════════

    def start_match(self) -> None:
        """Appelé quand la tirette est tirée — démarre le match."""
        if self.phase not in (BrainPhase.READY, BrainPhase.INIT):
            print(f"[Brain] WARN: start_match appelé en phase {self.phase.name}")

        self.phase = BrainPhase.RUNNING
        self._match_start = time.time()
        self._tick_count = 0

        print("\n" + "=" * 60)
        print("[Brain] MATCH DÉMARRÉ!")
        print(f"[Brain] Durée: {config.MATCH_DURATION_MS / 1000:.0f}s")
        print(f"[Brain] Actions en queue: {self.action_queue.size}")
        print("=" * 60 + "\n")

        # Envoyer la queue complète au robot
        self.executor.send_full_queue()

    def tick(self) -> None:
        """Un cycle de la boucle principale. Appeler à ~10 Hz."""
        self._tick_count += 1

        if self.phase != BrainPhase.RUNNING:
            return

        # ── Vérifier temps de match ───────────────────────────────────
        if self._match_start is not None:
            elapsed_ms = (time.time() - self._match_start) * 1000
            if elapsed_ms >= config.MATCH_DURATION_MS:
                print("[Brain] FIN DU MATCH — STOP")
                self.executor.abort()
                self.phase = BrainPhase.FINISHED
                return

        # ── Faire avancer l'executor ──────────────────────────────────
        self.executor.tick()

        # ── Vérifier si on a atteint un objectif ─────────────────────
        self.mission_mgr.check_and_advance(self.robot.position)

        # ── Route terminée ? ──────────────────────────────────────────
        if self.mission_mgr.is_route_complete and self.action_queue.is_empty():
            print(f"[Brain] Route terminée! "
                  f"{self.mission_mgr.progress} objectifs atteints")
            self.phase = BrainPhase.FINISHED
            return

        # ── Monitoring ────────────────────────────────────────────────
        self._monitor()

        # ── Debug périodique ──────────────────────────────────────────
        if DEBUG and self._tick_count % 50 == 0:
            self._debug_status()

    def _monitor(self) -> None:
        """Monitoring simple pendant l'exécution.

        Vérifie :
          1. Le robot progresse (pas bloqué / ultrasons)
          2. Le robot est sur la bonne trajectoire

        Si problème détecté → clear queue + replan depuis la position actuelle.
        """
        # ── Détection de blocage (ultrasons / immobilité) ─────────────
        dist_moved = self.robot.position.distance_to(self._last_robot_pos)

        if dist_moved < config.MONITORING_STUCK_MIN_MOVE_MM:
            self._monitoring_stuck_ticks += 1
        else:
            self._monitoring_stuck_ticks = 0
            self._last_robot_pos = Position(
                self.robot.position.x, self.robot.position.y
            )

        if self._monitoring_stuck_ticks >= self._monitoring_stuck_threshold:
            if not self.mission_mgr.is_route_complete:
                print(f"[Brain] MONITORING: Robot bloqué depuis "
                      f"{self._monitoring_stuck_ticks} ticks — replan!")
                self._replan_from_current()
                self._monitoring_stuck_ticks = 0
                return

        # ── Vérification de la trajectoire ────────────────────────────
        target = self.mission_mgr.current_target_position
        if target is not None:
            expected_dist = self.robot.position.distance_to(target)
            if expected_dist > self._monitoring_deviation_threshold_mm * 3:
                print(f"[Brain] MONITORING: Déviation trop grande "
                      f"(dist={expected_dist:.0f}mm) — replan!")
                self._replan_from_current()
                self._monitoring_stuck_ticks = 0
                return

    def _replan_from_current(self) -> None:
        """Replanifie depuis la position actuelle vers les objectifs restants.

        1. Clear la queue du robot (CLEAR_QUEUE via BLE)
        2. Recalcule A* pour les objectifs non atteints
        3. Envoie la nouvelle queue
        """
        # Cooldown : pas de replan trop fréquent
        if self._tick_count - self._last_replan_tick < self._replan_cooldown_ticks:
            if DEBUG:
                print(f"[Brain] REPLAN ignoré — cooldown "
                      f"({self._replan_cooldown_ticks - (self._tick_count - self._last_replan_tick)} ticks restants)")
            return
        self._last_replan_tick = self._tick_count

        print("\n" + "-" * 40)
        print("[Brain] REPLAN depuis position actuelle")
        print("-" * 40)

        # Envoyer STOP + clear queue
        self.executor.abort()

        # Objectifs restants
        remaining = self.mission_mgr.get_remaining_targets()
        if not remaining:
            print("[Brain] Aucun objectif restant — rien à replanifier")
            return

        obstacles = self.world.get_obstacles()

        print(f"[Brain] {len(remaining)} objectifs restants, "
              f"{len(obstacles)} obstacles")

        # Recalculer le chemin A*
        full_path = self.planner.plan_full_route(
            self.robot.position, remaining, obstacles
        )

        if len(full_path) < 2:
            print("[Brain] Échec du replan — chemin trop court")
            return

        # Convertir en actions
        actions = self.planner.path_to_actions(
            full_path,
            self.robot.heading_deg,
        )

        # Reconstruire la route dans le mission manager
        self.mission_mgr.rebuild_route_from(remaining)

        # Envoyer la nouvelle queue au robot
        self.action_queue.enqueue_many(actions)
        self.executor.send_full_queue()

        print(f"[Brain] Nouvelle queue envoyée: {len(actions)} actions")
        print("-" * 40 + "\n")

    # ── Boucle principale ─────────────────────────────────────────────

    def run(self) -> None:
        """Boucle principale temps réel (phase RUN). Bloquante."""
        if self.phase == BrainPhase.READY:
            self.start_match()

        print(f"[Brain] Boucle de monitoring à {config.BRAIN_LOOP_HZ} Hz")

        try:
            while self.phase == BrainPhase.RUNNING:
                t_start = time.time()
                self.tick()
                elapsed = time.time() - t_start
                sleep_time = config.BRAIN_LOOP_DT_S - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[Brain] Interruption clavier — arrêt")
            self.stop()

        print(f"[Brain] Fin du match. Progression: {self.mission_mgr.progress}")

    def stop(self) -> None:
        """Arrête proprement le cerveau."""
        print("[Brain] Arrêt en cours...")
        self.phase = BrainPhase.FINISHED
        self.executor.abort()
        print("[Brain] Arrêté.")

    # ── Commandes manuelles ───────────────────────────────────────────

    def send_manual_command(self, cmd: str) -> None:
        """Envoie une commande manuelle au robot (bypass la file)."""
        print(f"[Brain] Commande manuelle: '{cmd}'")
        self.executor._send_fn(cmd)

    def force_stop(self) -> None:
        """Arrêt d'urgence."""
        print("[Brain] ARRÊT D'URGENCE")
        self.executor.abort()
        self.phase = BrainPhase.FINISHED

    def force_replan(self) -> None:
        """Force un replan depuis la position actuelle (appelable depuis l'UI)."""
        self._replan_from_current()

    # ── Debug ─────────────────────────────────────────────────────────

    def _debug_status(self) -> None:
        elapsed = ""
        if self._match_start:
            e = time.time() - self._match_start
            elapsed = f" | match: {e:.1f}s"

        print(f"[Brain] tick#{self._tick_count} | "
              f"robot={self.robot.position} heading={self.robot.heading_deg:.0f}° "
              f"status={self.robot.status.name} | "
              f"route: {self.mission_mgr.progress} | "
              f"queue: {self.action_queue.size} actions"
              f"{elapsed}")

    def dump(self) -> None:
        """Dump complet de l'état (debug)."""
        print("\n" + "#" * 60)
        print("# BRAIN DUMP")
        print("#" * 60)
        print(f"Phase: {self.phase.name}")
        self.world.dump()
        self.mission_mgr.dump()
        self.action_queue.dump()
        print(f"Robot: {self.robot}")
        print("#" * 60 + "\n")
