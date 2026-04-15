"""
cerebros/executor.py -- Executeur d'actions.

Envoie la queue complete d'actions au robot via BLE d'un coup.
Le robot stocke et execute les commandes localement (robuste
en cas de perte de connexion).

Pendant le match, l'executor ne fait que surveiller l'etat.
En cas de replan, il envoie CLEAR_QUEUE + nouvelle queue.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from cerebros.actions import ActionQueue
from cerebros.config import DEBUG
from cerebros.models import Action, ActionType, RobotStatus
from cerebros.robot_state import RobotState


# Type pour la callback d'envoi
SendActionFn = Callable[[str], None]

# Delai entre chaque commande BLE pour ne pas saturer le buffer
BLE_SEND_DELAY_S = 0.05   # 50ms entre chaque commande


class Executor:
    """Envoie la queue complete au robot et surveille l'execution.

    Le robot ESP32 stocke les commandes dans sa cmdQueue FreeRTOS (64 slots)
    et les execute sequentiellement. Le PC n'a plus besoin de piloter
    commande par commande.
    """

    def __init__(self, action_queue: ActionQueue,
                 robot_state: RobotState,
                 send_fn: Optional[SendActionFn] = None):
        self._queue = action_queue
        self._robot = robot_state
        self._send_fn = send_fn or self._default_send
        self._queue_sent = False

        if DEBUG:
            print("[Executor] Initialise")

    def set_send_function(self, fn: SendActionFn) -> None:
        self._send_fn = fn
        if DEBUG:
            print("[Executor] Fonction d'envoi mise a jour")

    # -- Envoi de la queue complete ------------------------------------

    def send_full_queue(self) -> int:
        """Envoie toute la queue d'actions au robot d'un coup.

        Chaque action est envoyee comme commande BLE individuelle,
        avec un petit delai pour ne pas saturer le buffer.
        Le robot les stocke dans sa cmdQueue FreeRTOS.

        Returns:
            Nombre d'actions envoyees.
        """
        count = 0
        total = self._queue.size

        while not self._queue.is_empty():
            action = self._queue.dequeue()
            if action is None:
                break

            cmd = action.to_command()
            if DEBUG:
                print(f"[Executor] >>> ENVOI [{count + 1}/{total}]: '{cmd}'")

            self._send_fn(cmd)
            count += 1

            # Petit delai pour ne pas saturer le BLE
            time.sleep(BLE_SEND_DELAY_S)

        self._queue_sent = True
        self._robot.set_status(RobotStatus.MOVING)

        print(f"[Executor] Queue complete envoyee: {count} actions au robot")
        return count

    # -- Tick (monitoring only) ----------------------------------------

    def tick(self) -> bool:
        """Tick de monitoring. Retourne True si le robot est occupe."""
        if not self._queue_sent:
            return False
        return self._robot.status not in (RobotStatus.IDLE, RobotStatus.STOPPED)

    # -- Controle ------------------------------------------------------

    def abort(self) -> None:
        """Envoie CLEAR_QUEUE au robot (vide sa queue + STOP)
        et vide la queue locale."""
        if DEBUG:
            print("[Executor] ABORT -- CLEAR_QUEUE")

        self._queue.clear()
        self._queue_sent = False
        self._send_fn("CLEAR_QUEUE")
        self._robot.set_status(RobotStatus.STOPPED)

    def send_command(self, cmd: str) -> None:
        """Envoie une commande unique au robot (bypass queue)."""
        if DEBUG:
            print(f"[Executor] Commande directe: '{cmd}'")
        self._send_fn(cmd)

    @property
    def is_busy(self) -> bool:
        return self._queue_sent and not self._queue.is_empty()

    @property
    def queue_sent(self) -> bool:
        return self._queue_sent

    def force_idle(self) -> None:
        """Force le robot a l'etat IDLE (confirmation externe)."""
        self._robot.set_status(RobotStatus.IDLE)
        if DEBUG:
            print("[Executor] Force IDLE")

    @staticmethod
    def _default_send(cmd: str) -> None:
        print(f"[Executor] DEFAULT SEND (pas de BLE) -> '{cmd}'")