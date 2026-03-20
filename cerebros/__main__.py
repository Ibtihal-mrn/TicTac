#!/usr/bin/env python3
"""
cerebros/__main__.py — Point d'entrée pour tester le cerveau robot.

Lance le Brain avec des données simulées pour validation rapide.

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
    print(f"  📡 [BLE MOCK] → '{cmd}'")


def main() -> None:
    print("=" * 60)
    print("  CEREBROS — Test standalone")
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
    # Format : (label, grid_x, grid_y) — comme produit par markers.py
    fake_detections = [
        # Notre robot (bleu)
        ("BR1", 1, 10),
        # Quelques goals (objets bleus)
        ("BLUE55", 20, 8),
        ("BLUE60", 25, 15),
        ("BLUE52", 10, 5),
        # Un obstacle (zone)
        ("AREA15", 12, 9),
        # Un robot ennemi (jaune)
        ("YR1", 18, 10),
    ]

    print("\n[Test] Injection de détections simulées...")
    brain.feed_vision(fake_detections)
    brain.dump()

    # ── Lancer quelques ticks ─────────────────────────────────────────
    print("\n[Test] Lancement de 20 ticks...")
    for i in range(20):
        brain.tick()
        time.sleep(0.1)

        # Simuler que le robot devient IDLE après chaque commande
        if brain.robot.status != brain.robot.status.IDLE:
            brain.executor.force_idle()

    brain.dump()

    # ── Test boucle complète (courte) ─────────────────────────────────
    print("\n[Test] Voulez-vous lancer la boucle complète ? (Ctrl+C pour arrêter)")
    print("[Test] Appuyez sur Entrée pour continuer, ou Ctrl+C pour quitter")

    try:
        input()
        brain.run()
    except KeyboardInterrupt:
        print("\n[Test] Arrêt demandé")
        brain.stop()

    print("\n[Test] Test terminé.")
    brain.dump()


if __name__ == "__main__":
    main()
