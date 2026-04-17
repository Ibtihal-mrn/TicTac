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
#   36 → Bleu (A36) — objectif à prendre   (plusieurs exemplaires sur la table)
#   47 → Jaune (A47) — objectif à prendre
#   51 → Noir (A51) — à éviter             (plusieurs exemplaires sur la table)
#   41 → Zone à éviter (anciennement robot)
#
# Notre robot :
#   Team BLUE   → marker 1 (BR1)
#   Team YELLOW → marker 6 (YR1)
CORNER_IDS = {20, 21, 22, 23}

# Notre robot est identifié par le marker de l'équipe (1 si bleu, 6 si jaune)
OUR_ROBOT_BLUE_ID   = 1   # Marker ArUco 1 → BR1
OUR_ROBOT_YELLOW_ID = 6   # Marker ArUco 6 → YR1

BLUE_ROBOT_IDS  = set(range(1, 6))     # 1‑5
YELLOW_ROBOT_IDS = set(range(6, 11))   # 6‑10

BLUE_OBJECT_IDS   = {36}                 # Bleu A36 (plusieurs sur la table)
YELLOW_OBJECT_IDS = {47}                 # Jaune A47
BLACK_OBJECT_IDS  = {51}                 # Noir A51 (plusieurs sur la table, à éviter)

# IDs à éviter (obstacles spécifiques)
AVOID_IDS = {41}                          # A41 — zone à éviter

# Markers pouvant apparaître en plusieurs exemplaires sur la table
MULTI_INSTANCE_IDS = {36, 47, 51}

# IDs 11‑50 (hors coins, objets spéciaux et avoid) sont des zones/obstacles
AREA_IDS = (set(range(11, 51)) - CORNER_IDS - BLUE_OBJECT_IDS
            - YELLOW_OBJECT_IDS - BLACK_OBJECT_IDS - AVOID_IDS)

# ── Équipe ───────────────────────────────────────────────────────────────────
# ── Paramètres de planification ──────────────────────────────────────────────
OBSTACLE_SAFETY_MARGIN_MM = 200        # marge autour des obstacles
GOAL_REACHED_THRESHOLD_MM = 50         # distance pour considérer un goal atteint
REPLAN_DISTANCE_MM        = 150        # distance mini avant re-planification

# ── Paramètres de monitoring ─────────────────────────────────────────────────
MONITORING_STUCK_THRESHOLD = 1         # ticks sans mouvement avant replan
MONITORING_STUCK_MIN_MOVE_MM = 5       # mouvement minimum pour ne pas être "stuck"

# ── Paramètres de mouvement ──────────────────────────────────────────────────
EXIT_ZONE_MM              = 500        # FORWARD initial pour sortir de la zone de départ
HEADING_TOLERANCE_DEG     = 5          # tolérance angulaire avant déplacement

# ── Boucle principale ────────────────────────────────────────────────────────
BRAIN_LOOP_HZ   = 10                   # fréquence de la boucle (Hz)
BRAIN_LOOP_DT_S = 1.0 / BRAIN_LOOP_HZ

# ── Durée du match (ms) ─────────────────────────────────────────────────────
MATCH_DURATION_MS = 100_000            # 100 secondes selon règlement

# ── Debug ────────────────────────────────────────────────────────────────────
DEBUG = True
