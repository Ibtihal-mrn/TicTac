"""
cerebros/executor.py — Exécuteur d'actions.

Consomme la file d'actions et appelle send_action() pour chaque commande.
NE CONTIENT PAS de BLE — utilise une fonction callback pour l'envoi.

L'executor gère le timing entre les commandes et attend la confirmation
(ou un timeout) avant de passer à la commande suivante.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from cerebros.actions import ActionQueue
from cerebros.config import DEBUG
from cerebros.models import Action, ActionType, RobotStatus
from cerebros.robot_state import RobotState


# Type pour la callback d'envoi — fournie par l'appelant (UI, BLE, mock…)
SendActionFn = Callable[[str], None]


class Executor:
    """Exécute les actions de la file en appelant send_action().

    Usage:
        def my_send(cmd: str):
            print(f"Envoi BLE: {cmd}")

        executor = Executor(action_queue, robot_state, send_fn=my_send)
        executor.tick()  # dans la boucle principale
    """

    def __init__(self, action_queue: ActionQueue,
                 robot_state: RobotState,
                 send_fn: Optional[SendActionFn] = None):
        self._queue = action_queue
        self._robot = robot_state
        self._send_fn = send_fn or self._default_send
        self._current_action: Optional[Action] = None
        self._action_sent_at: float = 0.0
        self._action_timeout_s: float = 5.0    # timeout par défaut
        self._min_delay_between_s: float = 0.3  # délai minimum entre 2 commandes
        self._last_send_time: float = 0.0

        if DEBUG:
            print("[Executor] Initialisé — prêt à exécuter des actions")

    def set_send_function(self, fn: SendActionFn) -> None:
        """Change la fonction d'envoi (pour brancher le BLE plus tard)."""
        self._send_fn = fn
        if DEBUG:
            print("[Executor] Fonction d'envoi mise à jour")

    # ── Tick principal ────────────────────────────────────────────────────

    def tick(self) -> bool:
        """Appelé à chaque itération de la boucle principale.

        Returns:
            True si une action a été envoyée ou est en cours,
            False si la file est vide et rien n'est en cours.
        """
        now = time.time()

        # ── Action en cours ? Vérifier timeout ────────────────────────
        if self._current_action is not None:
            elapsed = now - self._action_sent_at

            # Le robot est-il revenu IDLE ? → action terminée
            if self._robot.is_idle() and elapsed > self._min_delay_between_s:
                if DEBUG:
                    print(f"[Executor] Action terminée: {self._current_action} "
                          f"({elapsed:.2f}s)")
                self._current_action = None
                return True

            # Timeout ?
            if elapsed > self._action_timeout_s:
                if DEBUG:
                    print(f"[Executor] TIMEOUT sur {self._current_action} "
                          f"({elapsed:.1f}s)")
                self._current_action = None
                # On continue avec la prochaine action

            return True  # action en cours

        # ── Prochaine action ──────────────────────────────────────────
        if self._queue.is_empty():
            return False

        # Respect du délai minimum
        if (now - self._last_send_time) < self._min_delay_between_s:
            return True

        action = self._queue.dequeue()
        if action is None:
            return False

        # Envoyer la commande
        cmd = action.to_command()
        self._current_action = action
        self._action_sent_at = now
        self._last_send_time = now

        # Adapter le timeout selon le type d'action
        self._action_timeout_s = self._estimate_timeout(action)

        # Mettre à jour le statut du robot
        self._update_robot_status(action)

        if DEBUG:
            print(f"[Executor] >>> ENVOI: '{cmd}' "
                  f"(timeout={self._action_timeout_s:.1f}s)")

        self._send_fn(cmd)
        return True

    # ── Helpers ───────────────────────────────────────────────────────────

    def _estimate_timeout(self, action: Action) -> float:
        """Estime un timeout raisonnable pour une action."""
        if action.action_type in (ActionType.FORWARD, ActionType.BACKWARD):
            # ~200mm/s → timeout proportionnel à la distance
            dist = action.value or 200
            return max(2.0, dist / 150.0)
        if action.action_type in (ActionType.LEFT, ActionType.RIGHT):
            return 3.0
        if action.action_type in (ActionType.DEPLOY, ActionType.RETRACT):
            return 4.0
        return 2.0

    def _update_robot_status(self, action: Action) -> None:
        """Met à jour le statut du robot en fonction de l'action envoyée."""
        if action.action_type in (ActionType.FORWARD, ActionType.BACKWARD):
            self._robot.set_status(RobotStatus.MOVING)
        elif action.action_type in (ActionType.LEFT, ActionType.RIGHT):
            self._robot.set_status(RobotStatus.TURNING)
        elif action.action_type in (ActionType.DEPLOY, ActionType.RETRACT):
            self._robot.set_status(RobotStatus.DEPLOYING)
        elif action.action_type == ActionType.STOP:
            self._robot.set_status(RobotStatus.STOPPED)

    @staticmethod
    def _default_send(cmd: str) -> None:
        """Fonction d'envoi par défaut (print uniquement)."""
        print(f"[Executor] DEFAULT SEND (pas de BLE) → '{cmd}'")

    # ── Contrôle ──────────────────────────────────────────────────────────

    def abort(self) -> None:
        """Arrête l'action en cours et vide la file."""
        if DEBUG:
            print("[Executor] ABORT — arrêt de toutes les actions")
        self._current_action = None
        self._queue.clear()
        # Envoyer STOP immédiatement
        self._send_fn("STOP")
        self._robot.set_status(RobotStatus.STOPPED)

    @property
    def is_busy(self) -> bool:
        return self._current_action is not None or not self._queue.is_empty()

    def force_idle(self) -> None:
        """Force le robot à l'état IDLE (confirmation externe)."""
        self._robot.set_status(RobotStatus.IDLE)
        self._current_action = None
        if DEBUG:
            print("[Executor] Force IDLE — prêt pour la prochaine action")
