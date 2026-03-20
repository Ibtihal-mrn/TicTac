"""
cerebros/config.py — Configuration centrale du cerveau robot.

Toutes les constantes physiques, seuils et paramètres de décision.
"""

from __future__ import annotations

# ── Dimensions de la table (mm) ──────────────────────────────────────────────
TABLE_W_MM = 3000
TABLE_H_MM = 2000

# ── Grille de la vision (cellules) ───────────────────────────────────────────
GRID_COLS = 30
GRID_ROWS = 20

# Taille d'une cellule en mm
CELL_W_MM = TABLE_W_MM / GRID_COLS   # 100 mm
CELL_H_MM = TABLE_H_MM / GRID_ROWS   # 100 mm

# ── Classification des marqueurs ArUco ───────────────────────────────────────
# D'après marker_detection/markers.py :
#   1‑5   → robots bleus  (BR1‑BR5)
#   6‑10  → robots jaunes (YR1‑YR5)
#   11‑50 → zones/objets sur la table  (sauf 20‑23)
#   20‑23 → coins de table (calibration)
#   51‑70 → objets bleus (BLUE*)
#   71‑90 → objets jaunes (YELLOW*)
CORNER_IDS = {20, 21, 22, 23}

BLUE_ROBOT_IDS  = set(range(1, 6))     # 1‑5
YELLOW_ROBOT_IDS = set(range(6, 11))   # 6‑10

BLUE_OBJECT_IDS   = set(range(51, 71))   # 51‑70
YELLOW_OBJECT_IDS = set(range(71, 91))   # 71‑90

# IDs 11‑50 (hors coins) sont des zones/obstacles/goals
AREA_IDS = set(range(11, 51)) - CORNER_IDS

# ── Équipe ───────────────────────────────────────────────────────────────────
TEAM_BLUE   = "blue"
TEAM_YELLOW = "yellow"

# ── Paramètres de planification ──────────────────────────────────────────────
OBSTACLE_SAFETY_MARGIN_MM = 200        # marge autour des obstacles
GOAL_REACHED_THRESHOLD_MM = 50         # distance pour considérer un goal atteint
REPLAN_DISTANCE_MM        = 150        # distance mini avant re-planification

# ── Paramètres de mouvement ──────────────────────────────────────────────────
DEFAULT_MOVE_STEP_MM      = 200        # pas de FORWARD/BACKWARD par défaut
DEFAULT_TURN_STEP_DEG     = 90         # pas de LEFT/RIGHT par défaut
HEADING_TOLERANCE_DEG     = 5          # tolérance angulaire avant déplacement

# ── Boucle principale ────────────────────────────────────────────────────────
BRAIN_LOOP_HZ   = 10                   # fréquence de la boucle (Hz)
BRAIN_LOOP_DT_S = 1.0 / BRAIN_LOOP_HZ

# ── Durée du match (ms) ─────────────────────────────────────────────────────
MATCH_DURATION_MS = 90_000             # 90 secondes (ou 100s selon règlement)

# ── Debug ────────────────────────────────────────────────────────────────────
DEBUG = True
