"""
cerebros/ble_sender.py — Bridge BLE via bleak (Nordic UART Service).

Fournit une fonction synchrone send() utilisable comme callback par l'Executor,
avec une boucle async bleak en arriere-plan (meme approche que ui.py).
"""

from __future__ import annotations

import asyncio
import threading
from typing import Optional

from bleak import BleakClient, BleakScanner

# Nordic UART Service UUIDs (identiques a ui.py)
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> ESP32
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 -> PC

TARGET_NAME = "Eurobot"


class BLEBridge:
    """Connexion BLE vers un ESP32 — thread-safe, sync send()."""

    def __init__(self, target_name: str = TARGET_NAME):
        self._target_name = target_name
        self._client: Optional[BleakClient] = None
        self._rx_char = None
        self._connected = False
        self._detected_team: Optional[str] = None  # "BLUE" ou "YELLOW", lu depuis le heartbeat
        self._tirette_inserted: Optional[bool] = None  # True=IN, False=OUT
        self._match_started: bool = False  # passe a True sur [EVENT] MATCH_START
        self._queue_done: bool = False  # passe a True sur [EVENT] QUEUE_DONE
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _schedule(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    # -- Connexion -----------------------------------------------------

    def connect(self) -> bool:
        """Scan + connect (bloquant depuis le thread appelant)."""
        future = self._schedule(self._async_connect())
        return future.result(timeout=20.0)

    async def _async_connect(self) -> bool:
        print(f"[BLEBridge] Scanning for '{self._target_name}'...")
        devices = await BleakScanner.discover(timeout=5.0)
        candidates = [d for d in devices
                      if d.name and self._target_name in d.name]

        if not candidates:
            print("[BLEBridge] Device not found.")
            return False

        device = candidates[0]
        print(f"[BLEBridge] Found {device.name} [{device.address}]")

        self._client = BleakClient(device.address,
                                   disconnected_callback=self._on_disconnect)
        await self._client.connect(timeout=15.0)

        for svc in self._client.services:
            for c in svc.characteristics:
                if c.uuid == NUS_TX_UUID:
                    await self._client.start_notify(c, self._on_notify)
                if c.uuid == NUS_RX_UUID:
                    self._rx_char = c

        self._connected = True
        print(f"[BLEBridge] Connected to {device.name}")
        return True

    def _on_disconnect(self, _client):
        self._connected = False
        self._rx_char = None
        print("[BLEBridge] Disconnected.")

    def _on_notify(self, _sender, data: bytearray):
        text = data.decode("utf-8", errors="replace").strip()
        if text:
            print(f"[BLE <-] {text}")

            # Detect [EVENT] MATCH_START (tirette retiree)
            if "MATCH_START" in text:
                self._match_started = True
                print("[BLEBridge] >>> MATCH_START recu! <<<")

            # Detect [EVENT] QUEUE_DONE (robot a fini d'executer la queue)
            if "QUEUE_DONE" in text:
                self._queue_done = True
                print("[BLEBridge] >>> QUEUE_DONE recu! <<<")

            # Parse heartbeat: "[BLE] Heartbeat | ... | team=BLUE | tirette=IN"
            if "Heartbeat" in text:
                for part in text.split("|"):
                    part = part.strip()
                    if part.startswith("team=") and self._detected_team is None:
                        self._detected_team = part.split("=", 1)[1].strip()
                        print(f"[BLEBridge] Team detectee depuis ESP32: {self._detected_team}")
                    elif part.startswith("tirette="):
                        val = part.split("=", 1)[1].strip()
                        self._tirette_inserted = (val == "IN")

    @property
    def detected_team(self) -> Optional[str]:
        """Retourne l'equipe lue depuis le heartbeat ESP32, ou None si pas encore recu."""
        return self._detected_team

    @property
    def tirette_inserted(self) -> Optional[bool]:
        """True si la tirette est inseree, False si retiree, None si inconnu."""
        return self._tirette_inserted

    @property
    def match_started(self) -> bool:
        """True des que l'event MATCH_START a ete recu."""
        return self._match_started

    @property
    def queue_done(self) -> bool:
        """True des que l'event QUEUE_DONE a ete recu (robot a fini sa queue)."""
        return self._queue_done

    def clear_queue_done(self) -> None:
        """Reset le flag queue_done pour attendre le prochain batch."""
        self._queue_done = False

    # -- Envoi (sync, utilisable comme callback Executor) --------------

    def send(self, cmd: str) -> None:
        """Envoie une commande BLE (appel synchrone, thread-safe)."""
        print(f"[BLE ->] {cmd}")
        if not self._connected or not self._rx_char:
            print("[BLEBridge] Not connected — command not sent.")
            return
        future = self._schedule(self._async_write(cmd))
        try:
            future.result(timeout=2.0)
        except Exception as e:
            print(f"[BLEBridge] Send error: {e}")

    async def _async_write(self, cmd: str):
        if self._client and self._client.is_connected and self._rx_char:
            await self._client.write_gatt_char(
                self._rx_char, (cmd + "\n").encode("utf-8"), response=False)

    # -- Deconnexion ---------------------------------------------------

    def disconnect(self) -> None:
        if self._connected and self._client:
            future = self._schedule(self._client.disconnect())
            try:
                future.result(timeout=5.0)
            except Exception:
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)

    @property
    def connected(self) -> bool:
        return self._connected
