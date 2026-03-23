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
# Tags :
#   36 → Bleu (A36) — objectif à prendre
#   47 → Jaune (A47) — objectif à prendre
#   51 → Noir (A51) — à éviter
#   41 → Notre robot (A41)
CORNER_IDS = {20, 21, 22, 23}

OUR_ROBOT_ID = 41                        # Tag du robot A41

BLUE_ROBOT_IDS  = set(range(1, 6))     # 1‑5
YELLOW_ROBOT_IDS = set(range(6, 11))   # 6‑10

BLUE_OBJECT_IDS   = {36}                 # Bleu A36
YELLOW_OBJECT_IDS = {47}                 # Jaune A47
BLACK_OBJECT_IDS  = {51}                 # Noir A51 (à éviter)

# IDs 11‑50 (hors coins, robot et objectifs) sont des zones/obstacles
AREA_IDS = set(range(11, 51)) - CORNER_IDS - {OUR_ROBOT_ID} - BLUE_OBJECT_IDS - YELLOW_OBJECT_IDS - BLACK_OBJECT_IDS

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
