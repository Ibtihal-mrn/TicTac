"""Filtrage temporel simple des detections."""

from __future__ import annotations

from typing import Any

import numpy as np


class Tracker:
    """Lisse les detections sur quelques frames et supprime les intermittences."""

    def __init__(self, buffer_size: int = 3, min_hits: int = 2) -> None:
        self.buf: dict[Any, list[np.ndarray]] = {}
        self.hits: dict[Any, int] = {}
        self.buf_size = buffer_size
        self.min_hits = min_hits

    def update(self, keys: list[Any], values: list[np.ndarray]) -> tuple[list[Any], list[np.ndarray]]:
        """Met a jour l'historique et renvoie uniquement les detections stables.

        Supporte les IDs en double (ex: plusieurs markers 36 sur la table)
        en utilisant (id, instance_index) comme cle interne.
        """
        # Construire des cles uniques pour gerer les doublons :
        # si le marker 36 apparait 3 fois, on obtient (36,0), (36,1), (36,2)
        count: dict[Any, int] = {}
        unique_keys: list[tuple[Any, int]] = []
        for key in keys:
            idx = count.get(key, 0)
            count[key] = idx + 1
            unique_keys.append((key, idx))

        current = dict(zip(unique_keys, values))

        for ukey, val in current.items():
            if ukey not in self.buf:
                self.buf[ukey] = []
                self.hits[ukey] = 0
            self.buf[ukey].append(val)
            self.hits[ukey] += 1
            if len(self.buf[ukey]) > self.buf_size:
                self.buf[ukey].pop(0)

        for ukey in list(self.buf):
            if ukey not in current:
                self.hits[ukey] -= 1
                if self.hits[ukey] <= 0:
                    del self.buf[ukey]
                    del self.hits[ukey]

        # Sortie : restituer les cles originales (sans l'index d'instance)
        out_keys: list[Any] = []
        out_vals: list[np.ndarray] = []
        for ukey, history in self.buf.items():
            if len(history) >= self.min_hits:
                out_keys.append(ukey[0])  # cle originale
                out_vals.append(np.mean(history, axis=0))

        return out_keys, out_vals
