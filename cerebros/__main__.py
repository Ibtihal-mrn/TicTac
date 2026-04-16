#!/usr/bin/env python3
"""
cerebros/__main__.py — Point d'entrée pour tester le cerveau robot.

Démontre le flux en 2 phases :
    1. INIT : vision → A* → queue complète
    2. RUN  : tirette → monitoring + replan si bloqué

Usage:
    cd TicTac
    python -m cerebros
"""

from __future__ import annotations

import time

from cerebros.brain import Brain
from cerebros.models import Position, Team


def fake_send(cmd: str) -> None:
    """Simule l'envoi BLE — juste un print."""
    print(f"  [BLE MOCK] -> '{cmd}'")


def main() -> None:
    print("=" * 60)
    print("  CEREBROS — Test A* standalone")
    print("=" * 60)

    # ── Créer le cerveau ──────────────────────────────────────────────
    brain = Brain(
        team=Team.BLUE,
        robot_id="BR1",
        initial_pos=Position(150, 1000),
        initial_heading=0.0,
    )
    brain.set_send_function(fake_send)

    # ── Simuler des détections vision ─────────────────────────────────
    fake_detections = [
        ("BR1", 1, 10),          # Notre robot
        ("BLUE36", 20, 8),       # Goal bleu
        ("BLUE36", 25, 15),      # Goal bleu (autre instance)
        ("AREA15", 12, 9),       # Obstacle
        ("BLACK51", 15, 10),     # Obstacle noir
        ("YR1", 18, 10),         # Robot ennemi
    ]

    print("\n[Test] Injection des détections vision...")
    brain.feed_vision(fake_detections)

    # ══════════════════════════════════════════════════════════════════
    # PHASE 1 — INIT : Planification A*
    # ══════════════════════════════════════════════════════════════════
    print("\n[Test] === PHASE INIT ===")

    # Option A : laisser le brain utiliser les goals détectés
    # brain.init_plan()

    # Option B : fournir manuellement les coordonnées objectifs
    targets = [
        Position(2050, 850),     # Premier objectif
        Position(2550, 1550),    # Deuxième objectif
    ]
    labels = ["BLUE36_A", "BLUE36_B"]

    success = brain.init_plan(target_positions=targets, target_labels=labels)
    if not success:
        print("[Test] Échec de la planification — abandon")
        return

    brain.dump()

    # ══════════════════════════════════════════════════════════════════
    # PHASE 2 — RUN : Tirette tirée → queue envoyée au robot
    # ══════════════════════════════════════════════════════════════════
    print("\n[Test] === PHASE RUN (simulation) ===")
    print("[Test] start_match() envoie la queue complete au robot...")

    brain.start_match()

    # La queue a ete envoyee d'un coup au robot dans start_match().
    # On simule quelques ticks de monitoring.
    for i in range(10):
        brain.tick()
        time.sleep(0.05)

    brain.dump()
    print(f"\n[Test] Progression finale: {brain.mission_mgr.progress}")
    print("[Test] Test termine.")


if __name__ == "__main__":
    main()
