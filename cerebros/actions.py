"""
cerebros/actions.py — File d'actions et gestion de la queue.

Gère la file d'attente des commandes à envoyer au robot.
Thread-safe grâce à collections.deque.
"""

from __future__ import annotations

from collections import deque
from typing import List, Optional

from cerebros.config import DEBUG
from cerebros.models import Action


class ActionQueue:
    """File FIFO d'actions à envoyer au robot."""

    def __init__(self):
        self._queue: deque[Action] = deque()
        self._history: List[Action] = []       # actions déjà envoyées

        if DEBUG:
            print("[ActionQueue] Initialisée — vide")

    def enqueue(self, action: Action) -> None:
        """Ajoute une action à la fin de la file."""
        self._queue.append(action)
        if DEBUG:
            print(f"[ActionQueue] + {action}  (file: {len(self._queue)})")

    def enqueue_many(self, actions: List[Action]) -> None:
        """Ajoute une liste d'actions à la file."""
        for a in actions:
            self._queue.append(a)
        if DEBUG:
            print(f"[ActionQueue] + {len(actions)} actions  "
                  f"(file: {len(self._queue)})")

    def dequeue(self) -> Optional[Action]:
        """Retire et retourne la prochaine action, ou None si vide."""
        if not self._queue:
            return None
        action = self._queue.popleft()
        self._history.append(action)
        if DEBUG:
            print(f"[ActionQueue] → Envoi: {action}  "
                  f"(restant: {len(self._queue)})")
        return action

    def peek(self) -> Optional[Action]:
        """Regarde la prochaine action sans la retirer."""
        if not self._queue:
            return None
        return self._queue[0]

    def clear(self) -> None:
        """Vide la file (ex: re-planification en cours)."""
        count = len(self._queue)
        self._queue.clear()
        if DEBUG:
            print(f"[ActionQueue] Vidée ({count} actions supprimées)")

    def is_empty(self) -> bool:
        return len(self._queue) == 0

    @property
    def size(self) -> int:
        return len(self._queue)

    @property
    def history(self) -> List[Action]:
        return list(self._history)

    def dump(self) -> None:
        """Affiche le contenu de la file (debug)."""
        print(f"[ActionQueue] Taille: {len(self._queue)}")
        for i, a in enumerate(self._queue):
            print(f"  [{i}] {a}")
