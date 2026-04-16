"""
cerebros/astar.py — Pathfinding A* sur la grille de la table.

Grille de GRID_COLS x GRID_ROWS cellules. Chaque cellule est libre ou bloquée.
L'algorithme A* retourne le chemin optimal (liste de cellules grille),
converti ensuite en positions mm pour le robot.
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Set, Tuple

from cerebros.config import (
    CELL_H_MM,
    CELL_W_MM,
    DEBUG,
    GRID_COLS,
    GRID_ROWS,
    OBSTACLE_SAFETY_MARGIN_MM,
)
from cerebros.models import ObjectInfo, Position


# Type alias pour une cellule grille (col, row)
Cell = Tuple[int, int]


class AStarGrid:
    """Grille 2D pour le pathfinding A*."""

    def __init__(self):
        # True = libre, False = bloqué
        self._grid: List[List[bool]] = [
            [True for _ in range(GRID_ROWS)]
            for _ in range(GRID_COLS)
        ]

    def reset(self) -> None:
        for c in range(GRID_COLS):
            for r in range(GRID_ROWS):
                self._grid[c][r] = True

    def block_cell(self, col: int, row: int) -> None:
        if 0 <= col < GRID_COLS and 0 <= row < GRID_ROWS:
            self._grid[col][row] = False

    def is_free(self, col: int, row: int) -> bool:
        if 0 <= col < GRID_COLS and 0 <= row < GRID_ROWS:
            return self._grid[col][row]
        return False

    def mark_obstacles(self, obstacles: List[ObjectInfo],
                       safety_margin_cells: int = 0) -> None:
        """Bloque les cellules occupées par des obstacles + marge de sécurité."""
        if safety_margin_cells == 0:
            # Calculer la marge en cellules depuis la marge en mm
            safety_margin_cells = max(1, int(
                OBSTACLE_SAFETY_MARGIN_MM / min(CELL_W_MM, CELL_H_MM)
            ))

        for obs in obstacles:
            cx = int(obs.position.x / CELL_W_MM)
            cy = int(obs.position.y / CELL_H_MM)

            for dc in range(-safety_margin_cells, safety_margin_cells + 1):
                for dr in range(-safety_margin_cells, safety_margin_cells + 1):
                    self.block_cell(cx + dc, cy + dr)

    def get_blocked_cells(self) -> List[Cell]:
        blocked = []
        for c in range(GRID_COLS):
            for r in range(GRID_ROWS):
                if not self._grid[c][r]:
                    blocked.append((c, r))
        return blocked


def mm_to_grid(x_mm: float, y_mm: float) -> Cell:
    """Convertit des coordonnées mm en cellule grille."""
    col = int(x_mm / CELL_W_MM)
    row = int(y_mm / CELL_H_MM)
    col = max(0, min(GRID_COLS - 1, col))
    row = max(0, min(GRID_ROWS - 1, row))
    return (col, row)


def grid_to_mm_center(col: int, row: int) -> Position:
    """Retourne le centre d'une cellule en mm."""
    return Position(
        (col + 0.5) * CELL_W_MM,
        (row + 0.5) * CELL_H_MM,
    )


def _heuristic(a: Cell, b: Cell) -> float:
    """Distance euclidienne (heuristique admissible pour 8-directions)."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


# 8 directions : haut, bas, gauche, droite + diagonales
_NEIGHBORS_8 = [
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
]


def astar(grid: AStarGrid, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    """A* sur grille 8-directions.

    Returns:
        Liste de cellules du start au goal (inclus), ou None si pas de chemin.
    """
    if not grid.is_free(start[0], start[1]):
        if DEBUG:
            print(f"[A*] Start {start} est bloqué!")
        # Essayer de trouver la cellule libre la plus proche du start
        start = _find_nearest_free(grid, start)
        if start is None:
            return None

    if not grid.is_free(goal[0], goal[1]):
        if DEBUG:
            print(f"[A*] Goal {goal} est bloqué!")
        goal = _find_nearest_free(grid, goal)
        if goal is None:
            return None

    if start == goal:
        return [start]

    open_set: list = []
    heapq.heappush(open_set, (0.0, start))

    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    f_score: dict[Cell, float] = {start: _heuristic(start, goal)}

    closed: Set[Cell] = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruire le chemin
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        if current in closed:
            continue
        closed.add(current)

        for dx, dy in _NEIGHBORS_8:
            neighbor = (current[0] + dx, current[1] + dy)

            if not grid.is_free(neighbor[0], neighbor[1]):
                continue
            if neighbor in closed:
                continue

            # Coût du mouvement : 1.0 cardinal, √2 diagonal
            move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0

            # Pour les diagonales, vérifier que les 2 cellules adjacentes
            # sont libres (pas de passage en diagonale à travers un coin)
            if dx != 0 and dy != 0:
                if (not grid.is_free(current[0] + dx, current[1])
                        or not grid.is_free(current[0], current[1] + dy)):
                    continue

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    if DEBUG:
        print(f"[A*] Pas de chemin trouvé de {start} à {goal}")
    return None


def _find_nearest_free(grid: AStarGrid, cell: Cell,
                       max_radius: int = 5) -> Optional[Cell]:
    """Trouve la cellule libre la plus proche de cell."""
    for r in range(1, max_radius + 1):
        for dc in range(-r, r + 1):
            for dr in range(-r, r + 1):
                if abs(dc) == r or abs(dr) == r:
                    nc = (cell[0] + dc, cell[1] + dr)
                    if grid.is_free(nc[0], nc[1]):
                        return nc
    return None


def simplify_path(path: List[Cell]) -> List[Cell]:
    """Réduit le nombre de waypoints en supprimant les points colinéaires.

    Garde uniquement les points où la direction change.
    """
    if len(path) <= 2:
        return path

    simplified = [path[0]]

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        nxt = path[i + 1]

        # Direction entrante vs sortante
        dir_in = (curr[0] - prev[0], curr[1] - prev[1])
        dir_out = (nxt[0] - curr[0], nxt[1] - curr[1])

        if dir_in != dir_out:
            simplified.append(curr)

    simplified.append(path[-1])
    return simplified


def _line_of_sight(grid: AStarGrid, a: Cell, b: Cell) -> bool:
    """Vérifie que toutes les cellules sur la ligne droite a→b sont libres."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return True
    for i in range(steps + 1):
        t = i / steps
        x = round(a[0] + dx * t)
        y = round(a[1] + dy * t)
        if not grid.is_free(x, y):
            return False
    return True


def smooth_path(grid: AStarGrid, path: List[Cell]) -> List[Cell]:
    """Lisse le chemin A* par vérification de ligne de vue.

    Supprime les waypoints intermédiaires inutiles en vérifiant si une
    ligne droite entre deux points non-adjacents est libre d'obstacles.
    Produit des diagonales franches au lieu de zig-zags grille.
    """
    if len(path) <= 2:
        return path

    result = [path[0]]
    current = 0

    while current < len(path) - 1:
        # Chercher le point le plus loin atteignable en ligne droite
        farthest = current + 1
        for j in range(len(path) - 1, current + 1, -1):
            if _line_of_sight(grid, path[current], path[j]):
                farthest = j
                break
        result.append(path[farthest])
        current = farthest

    return result


def path_cells_to_mm(cells: List[Cell]) -> List[Position]:
    """Convertit un chemin de cellules en positions mm (centres des cellules)."""
    return [grid_to_mm_center(c, r) for c, r in cells]
