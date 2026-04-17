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
    INIT = auto()               # Avant tirette : détection + planification A*
    READY = auto()              # Plan calculé, en attente de la tirette
    RUNNING_BATCH = auto()      # Batch envoyé, robot exécute sans monitoring
    WAITING_RECALC = auto()     # Batch terminé, recalcul du prochain batch
    WAITING_TIMER = auto()      # Attente d'un timer avant le prochain batch
    FINISHED = auto()           # Match terminé


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
            initial_pos=initial_pos,
            initial_heading=initial_heading,
        )
        self.world.set_our_robot(self.robot)
        self._robot_position_known = initial_pos is not None

        self.mission_mgr = MissionManager()
        self.planner = Planner()
        self.action_queue = ActionQueue()
        self.executor = Executor(self.action_queue, self.robot)

        # ── État interne ──────────────────────────────────────────────
        self.phase = BrainPhase.INIT
        self._match_start: Optional[float] = None
        self._tick_count = 0

        # ── Données de planification (pour visualisation) ─────────────
        self.planned_path: List[Position] = []       # waypoints A* (mm)
        self.planned_targets: List[Position] = []    # objectifs (mm)
        self.planned_target_labels: List[str] = []

        # ── Multi-batch pipeline ──────────────────────────────────────
        self._batches: List[List[Position]] = []     # list of target batches
        self._batch_labels: List[List[str]] = []     # labels per batch
        self._current_batch_idx: int = 0
        self._waiting_for_exit_queue_done: bool = False
        self._ble_bridge = None  # set via set_ble_bridge()
        self._last_batch_wait_s: float = 87.0  # attendre 87s avant le dernier batch
        # ── Monitoring ────────────────────────────────────────────────
        self._monitoring_deviation_threshold_mm = config.REPLAN_DISTANCE_MM
        self._monitoring_stuck_ticks = 0
        self._monitoring_stuck_threshold = config.MONITORING_STUCK_THRESHOLD
        self._last_robot_pos = Position(self.robot.position.x,
                                        self.robot.position.y)
        self._last_replan_tick = 0
        self._replan_cooldown_ticks = 5    # ~0.5s à 10Hz entre deux replans

        pos_info = str(self.robot.position) if self._robot_position_known else "inconnue (attente vision)"
        print(f"[Brain] Prêt. Robot à {pos_info}, "
              f"heading={self.robot.heading_deg}°")

    # ── Configuration ─────────────────────────────────────────────────

    @property
    def robot_position_known(self) -> bool:
        """True si la position du robot a ete vue par la vision."""
        return self._robot_position_known

    def set_send_function(self, fn: SendActionFn) -> None:
        """Branche la fonction d'envoi BLE."""
        self.executor.set_send_function(fn)
        print("[Brain] Fonction d'envoi BLE configurée")

    def set_ble_bridge(self, ble) -> None:
        """Branche le bridge BLE pour détecter QUEUE_DONE."""
        self._ble_bridge = ble

    def set_batches(self, batches: List[List[Position]],
                    batch_labels: Optional[List[List[str]]] = None) -> None:
        """Définit les batches de targets à exécuter séquentiellement.

        Chaque batch = liste de positions. Entre chaque batch, le brain
        attend QUEUE_DONE du robot, puis recalcule A* depuis la position
        caméra actuelle.
        """
        self._batches = batches
        self._batch_labels = batch_labels or [[] for _ in batches]
        self._current_batch_idx = 0
        print(f"[Brain] {len(batches)} batches définis:")
        for i, b in enumerate(batches):
            lbl = self._batch_labels[i] if i < len(self._batch_labels) else []
            print(f"  Batch {i + 1}: {len(b)} targets {lbl}")

    # ── Entrée vision ─────────────────────────────────────────────────

    def feed_vision(self, detections: List[Tuple[str, int, int]],
                     robot_heading: float | None = None) -> None:
        """Reçoit les détections de la caméra.

        Args:
            detections: liste de (label, grid_x, grid_y)
            robot_heading: heading du robot en degrés (depuis ArUco corners)
        """
        self.world.update_from_vision(detections)

        # Detecter la premiere apparition du robot par la vision
        if not self._robot_position_known:
            for label, gx, gy in detections:
                if label == self.robot.robot_id:
                    self._robot_position_known = True
                    print(f"[Brain] Robot detecte par la vision a "
                          f"{self.robot.position}")
                    break

        if robot_heading is not None and self.robot is not None:
            self.robot.heading_deg = robot_heading
            if config.DEBUG:
                print(f"[Brain] Heading vision: {robot_heading:.1f}°")

        # Compter les frames vision pendant le recalcul inter-batch
        # Ne compter que si le robot est réellement détecté dans cette frame
        if self.phase == BrainPhase.WAITING_RECALC:
            robot_seen = any(label == self.robot.robot_id for label, _, _ in detections)
            if robot_seen:
                self._recalc_vision_frames = getattr(self, '_recalc_vision_frames', 0) + 1
            else:
                self._recalc_vision_frames = 0  # reset si le robot n'est plus vu

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

        if not self._robot_position_known:
            print("[Brain] Position du robot inconnue — impossible de planifier")
            return False

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

        # Stocker pour la visualisation (inclure le segment de sortie de base)
        self.planned_path = [Position(self.robot.position.x, self.robot.position.y)] + list(full_path)
        self.planned_targets = list(target_positions)
        self.planned_target_labels = list(target_labels) if target_labels else []

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

        self.phase = BrainPhase.READY
        print(f"\n[Brain] Plan prêt: {len(actions)} actions en queue")
        print(f"[Brain] Route: {self.mission_mgr.progress}")
        print("[Brain] En attente de la tirette (start_match)...")
        print("=" * 60 + "\n")

        return True

    # ══════════════════════════════════════════════════════════════════
    # PHASE 2 — RUN : Match en cours + monitoring
    # ══════════════════════════════════════════════════════════════════

    def start_match(self) -> None:
        """Appelé quand la tirette est tirée — démarre le match.

        Si des batches sont définis, envoie le premier batch.
        Sinon, envoie la queue complète (ancien comportement).
        """
        if self.phase not in (BrainPhase.READY, BrainPhase.INIT):
            print(f"[Brain] WARN: start_match appelé en phase {self.phase.name}")

        self._match_start = time.time()
        self._tick_count = 0

        print("\n" + "=" * 60)
        print("[Brain] MATCH DÉMARRÉ!")
        print(f"[Brain] Durée: {config.MATCH_DURATION_MS / 1000:.0f}s")

        if self._batches:
            # Multi-batch mode : envoyer le FORWARD de sortie seul,
            # puis attendre QUEUE_DONE avant de calculer le batch 1.
            self._current_batch_idx = 0
            self._waiting_for_exit_queue_done = True
            self._send_exit_forward()
        else:
            # Legacy mode : queue unique déjà remplie par init_plan
            self._waiting_for_exit_queue_done = False
            print(f"[Brain] Actions en queue: {self.action_queue.size}")
            self.executor.send_full_queue()
            self.phase = BrainPhase.RUNNING_BATCH

        print("=" * 60 + "\n")

    def _send_exit_forward(self) -> None:
        """Envoie la séquence de sortie de zone hardcodée, avant le batch 1."""
        exit_mm = config.EXIT_ZONE_MM
        # BLUE → tourner à droite (-90°), YELLOW → tourner à gauche (+90°)
        rotate_angle = -90 if self.robot.team == Team.BLUE else 90
        print(f"[Brain] Envoi séquence sortie de zone: FORWARD {exit_mm}mm → "
              f"ROTATE {rotate_angle}° → FORWARD {exit_mm}mm — batch 1 calculé après QUEUE_DONE")

        self.action_queue.clear()
        self.action_queue.enqueue_many([
            Action(ActionType.FORWARD, exit_mm),
            Action(ActionType.ROTATE, rotate_angle),
            Action(ActionType.FORWARD, exit_mm),
        ])
        self.executor.send_full_queue()

        if self._ble_bridge:
            self._ble_bridge.clear_queue_done()

        self.phase = BrainPhase.RUNNING_BATCH

    def _plan_and_send_current_batch(self) -> bool:
        """Planifie A* pour le batch courant et envoie au robot.

        Returns:
            True si le batch a été envoyé avec succès.
        """
        idx = self._current_batch_idx
        if idx >= len(self._batches):
            print("[Brain] Tous les batches terminés!")
            self.phase = BrainPhase.FINISHED
            return False

        targets = self._batches[idx]
        labels = self._batch_labels[idx] if idx < len(self._batch_labels) else []

        print(f"\n[Brain] === BATCH {idx + 1}/{len(self._batches)} ===")
        print(f"[Brain] Position actuelle: {self.robot.position}, "
              f"heading={self.robot.heading_deg:.1f}°")
        print(f"[Brain] {len(targets)} targets: {labels}")

        obstacles = self.world.get_obstacles()

        # Point de départ : position caméra actuelle
        start_pos = Position(self.robot.position.x, self.robot.position.y)

        # A*
        full_path = self.planner.plan_full_route(start_pos, targets, obstacles)
        if len(full_path) < 2:
            print(f"[Brain] Batch {idx + 1}: échec A* — skip")
            self._current_batch_idx += 1
            # On repasse en WAITING_RECALC pour retenter au prochain tick
            self.phase = BrainPhase.WAITING_RECALC
            self._recalc_vision_frames = 0
            return False

        # Stocker pour visualisation
        self.planned_path = list(full_path)
        self.planned_targets = list(targets)
        self.planned_target_labels = list(labels)

        # Convertir en actions
        actions = self.planner.path_to_actions(
            full_path, self.robot.heading_deg, deploy_at_end=False)

        # Envoyer
        self.action_queue.clear()
        self.action_queue.enqueue_many(actions)
        self.executor.send_full_queue()

        # Reset le flag QUEUE_DONE pour attendre la fin de ce batch
        if self._ble_bridge:
            self._ble_bridge.clear_queue_done()

        self.phase = BrainPhase.RUNNING_BATCH
        print(f"[Brain] Batch {idx + 1} envoyé: {len(actions)} actions")
        return True


    def tick(self) -> None:
        """Un cycle de la boucle principale. Appeler à ~10 Hz."""
        self._tick_count += 1

        if self.phase == BrainPhase.FINISHED:
            return

        # ── Vérifier temps de match ───────────────────────────────────
        if self._match_start is not None:
            elapsed_ms = (time.time() - self._match_start) * 1000
            if elapsed_ms >= config.MATCH_DURATION_MS:
                print("[Brain] FIN DU MATCH — STOP")
                self.executor.abort()
                self.phase = BrainPhase.FINISHED
                return

        # ── RUNNING_BATCH : attendre QUEUE_DONE du robot ─────────────
        if self.phase == BrainPhase.RUNNING_BATCH:
            if self._ble_bridge and self._ble_bridge.queue_done:
                if self._waiting_for_exit_queue_done:
                    self._waiting_for_exit_queue_done = False
                    print("[Brain] QUEUE_DONE reçu — sortie de zone terminée")
                    self.phase = BrainPhase.WAITING_RECALC
                    self._recalc_vision_frames = 0
                    print("[Brain] Attente de vision stable pour batch 1...")
                    return

                print(f"[Brain] QUEUE_DONE reçu — batch "
                      f"{self._current_batch_idx + 1} terminé")
                self._current_batch_idx += 1

                if self._current_batch_idx >= len(self._batches):
                    print("[Brain] Tous les batches terminés!")
                    self.phase = BrainPhase.FINISHED
                elif self._current_batch_idx == len(self._batches) - 1:
                    # Dernier batch (retour au nid) : attendre 87s de match
                    self.phase = BrainPhase.WAITING_TIMER
                    print(f"[Brain] Attente timer {self._last_batch_wait_s}s "
                          f"avant le dernier batch (retour au nid)...")
                else:
                    # Passer en recalcul — attendre quelques frames vision
                    self.phase = BrainPhase.WAITING_RECALC
                    self._recalc_vision_frames = 0
                    print(f"[Brain] Attente de vision stable pour "
                          f"batch {self._current_batch_idx + 1}...")
            return

        # ── WAITING_RECALC : attendre vision stable puis replanifier ──
        if self.phase == BrainPhase.WAITING_RECALC:
            # On compte les frames dans feed_vision; ici on vérifie
            if self._recalc_vision_frames >= 3:
                self._plan_and_send_current_batch()
            return

        # ── WAITING_TIMER : attendre 87s de match pour lancer retour au nid ──
        if self.phase == BrainPhase.WAITING_TIMER:
            if self._match_start is not None:
                elapsed_s = time.time() - self._match_start
                if elapsed_s >= self._last_batch_wait_s:
                    print(f"[Brain] Timer {self._last_batch_wait_s}s atteint "
                          f"(écoulé={elapsed_s:.1f}s) — lancement retour au nid")
                    self.phase = BrainPhase.WAITING_RECALC
                    self._recalc_vision_frames = 0
                elif self._tick_count % 50 == 0:
                    remaining = self._last_batch_wait_s - elapsed_s
                    print(f"[Brain] Attente retour au nid: "
                          f"{remaining:.0f}s restantes")
            return

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

        # Stocker pour la visualisation
        self.planned_path = list(full_path)
        self.planned_targets = list(remaining)

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
            print("[Brain] En attente de start_match() avant la boucle RUN")
            return

        print(f"[Brain] Boucle de monitoring à {config.BRAIN_LOOP_HZ} Hz")

        try:
            while self.phase in (BrainPhase.RUNNING_BATCH,
                                BrainPhase.WAITING_RECALC,
                                BrainPhase.WAITING_TIMER):
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
