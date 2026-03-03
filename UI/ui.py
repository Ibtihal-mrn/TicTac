"""
ui.py  —  Eurobot BLE Log Viewer (Direct BLE via bleak)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Connexion BLE DIRECTE à l'ESP32 "Eurobot" via le Nordic UART Service (NUS).
Affiche en temps réel tous les logs/notifications BLE.

Flux :
  ESP32 (BLEBridge NUS)  ──BLE notify──►  ui.py (bleak + Tkinter)

Dépendances :
  pip install bleak

Usage :
  python ui.py                                  # scan automatique "Eurobot"
  python ui.py --device "Eurobot"               # nom BLE personnalisé
  python ui.py --address AA:BB:CC:DD:EE:FF      # adresse MAC directe
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import asyncio
import argparse
from datetime import datetime

from bleak import BleakClient, BleakScanner

# ── Configuration ─────────────────────────────────────────────────────────────
DEFAULT_DEVICE_NAME = "Eurobot"
RECONNECT_DELAY_S   = 3

# UUIDs Nordic UART Service — identiques à ceux dans BLEBridge.h (ESP32)
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 → PC (notify)


# ── Palette ───────────────────────────────────────────────────────────────────
BG           = "#1e1e2e"
TEXT_BG      = "#181825"
TEXT_FG      = "#cdd6f4"
ACCENT       = "#89b4fa"
GREEN        = "#a6e3a1"
RED          = "#f38ba8"
YELLOW       = "#f9e2af"
DIM          = "#6c7086"
BTN_BG       = "#313244"
TITLE_CLR    = "#cba6f7"


class EurobotBLEViewer(tk.Tk):
    """Fenêtre principale — connexion BLE directe via bleak."""

    def __init__(self, device_name: str, device_address: str | None):
        super().__init__()

        # ── Paramètres ────────────────────────────────────────────────────────
        self.device_name    = device_name
        self.device_address = device_address

        # ── État interne ──────────────────────────────────────────────────────
        self._client: BleakClient | None = None
        self._connected  = False
        self._ble_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._log_queue: queue.Queue[str | None] = queue.Queue()
        self._ble_loop: asyncio.AbstractEventLoop | None = None
        self._line_count = 0

        # ── Construction UI ───────────────────────────────────────────────────
        self._setup_window()
        self._build_header()
        self._build_controls()
        self._build_status_bar()
        self._build_log_area()

        # ── Polling & fermeture ───────────────────────────────────────────────
        self._poll_id = self.after(30, self._poll_queue_loop)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ─────────────────────────────────────────────────────────────────────────
    #  UI CONSTRUCTION
    # ─────────────────────────────────────────────────────────────────────────
    def _setup_window(self):
        self.title("Eurobot — BLE Log Viewer")
        self.geometry("920x640")
        self.minsize(700, 450)
        self.configure(bg=BG)

    def _build_header(self):
        f = tk.Frame(self, bg=BG, pady=8)
        f.pack(fill=tk.X, padx=16)
        tk.Label(f, text="Eurobot BLE Log Viewer",
                 font=("Segoe UI", 16, "bold"), bg=BG, fg=TITLE_CLR
                 ).pack(side=tk.LEFT)

    def _build_controls(self):
        f = tk.Frame(self, bg=BG, pady=4)
        f.pack(fill=tk.X, padx=16)

        # Device input
        tk.Label(f, text="Device:", bg=BG, fg=TEXT_FG,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self._device_var = tk.StringVar(
            value=self.device_address or self.device_name)
        tk.Entry(f, textvariable=self._device_var, width=24,
                 bg=BTN_BG, fg=TEXT_FG, insertbackground=TEXT_FG,
                 relief=tk.FLAT, font=("Consolas", 10)
                 ).pack(side=tk.LEFT, padx=(4, 16))

        # Bouton Scan
        self._scan_btn = tk.Button(
            f, text="🔍  Scan", command=self._start_scan,
            bg=BTN_BG, fg=TEXT_FG, font=("Segoe UI", 10),
            relief=tk.FLAT, padx=10, pady=4, cursor="hand2")
        self._scan_btn.pack(side=tk.LEFT, padx=(0, 8))

        # Bouton Connect / Disconnect
        self._connect_btn = tk.Button(
            f, text="⚡  Connect BLE", command=self._toggle_connection,
            bg=ACCENT, fg=BG, font=("Segoe UI", 10, "bold"),
            relief=tk.FLAT, padx=12, pady=4, cursor="hand2")
        self._connect_btn.pack(side=tk.LEFT)

        # Clear
        tk.Button(f, text="🗑  Clear", command=self._clear_logs,
                  bg=BTN_BG, fg=TEXT_FG, font=("Segoe UI", 10),
                  relief=tk.FLAT, padx=10, pady=4, cursor="hand2"
                  ).pack(side=tk.LEFT, padx=8)

        # Autoscroll
        self._autoscroll = tk.BooleanVar(value=True)
        tk.Checkbutton(f, text="Auto-scroll", variable=self._autoscroll,
                       bg=BG, fg=TEXT_FG, selectcolor=BTN_BG,
                       activebackground=BG, font=("Segoe UI", 10)
                       ).pack(side=tk.LEFT, padx=4)

        # Timestamps
        self._timestamps = tk.BooleanVar(value=True)
        tk.Checkbutton(f, text="Timestamps", variable=self._timestamps,
                       bg=BG, fg=TEXT_FG, selectcolor=BTN_BG,
                       activebackground=BG, font=("Segoe UI", 10)
                       ).pack(side=tk.LEFT, padx=4)

    def _build_status_bar(self):
        f = tk.Frame(self, bg=BG, pady=2)
        f.pack(fill=tk.X, padx=16)

        self._dot = tk.Label(f, text="●", font=("Segoe UI", 12),
                             bg=BG, fg=RED)
        self._dot.pack(side=tk.LEFT)
        self._status_lbl = tk.Label(f, text="Déconnecté",
                                    font=("Segoe UI", 10), bg=BG, fg=RED)
        self._status_lbl.pack(side=tk.LEFT, padx=4)
        self._lines_lbl = tk.Label(f, text="0 lignes",
                                   font=("Segoe UI", 9), bg=BG, fg=DIM)
        self._lines_lbl.pack(side=tk.RIGHT)

        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X,
                                                        padx=16, pady=4)

    def _build_log_area(self):
        f = tk.Frame(self, bg=BG)
        f.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0, 12))

        self._log = tk.Text(
            f, bg=TEXT_BG, fg=TEXT_FG, font=("Consolas", 11),
            relief=tk.FLAT, wrap=tk.WORD, state=tk.DISABLED,
            selectbackground=BTN_BG, insertbackground=TEXT_FG,
            padx=8, pady=6)
        self._log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        sb = tk.Scrollbar(f, command=self._log.yview,
                          bg=BTN_BG, troughcolor=BG, relief=tk.FLAT)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._log.config(yscrollcommand=sb.set)

        # Tags de coloration
        for tag, fg in [("ts", DIM), ("normal", TEXT_FG),
                        ("info", ACCENT), ("warn", YELLOW),
                        ("error", RED), ("ok", GREEN)]:
            self._log.tag_config(tag, foreground=fg)

    # ─────────────────────────────────────────────────────────────────────────
    #  BLE SCAN
    # ─────────────────────────────────────────────────────────────────────────
    def _start_scan(self):
        self._scan_btn.config(state=tk.DISABLED, text="⏳ Scanning…")
        self._push("[UI] Scan BLE en cours (5 s)…\n")

        def _run():
            loop = asyncio.new_event_loop()
            try:
                devs = loop.run_until_complete(
                    BleakScanner.discover(timeout=5.0))
                if not devs:
                    self._push("[UI] Aucun appareil BLE trouvé.\n")
                else:
                    self._push(f"[UI] {len(devs)} appareil(s) :\n")
                    for d in sorted(devs,
                                    key=lambda x: x.rssi or -999,
                                    reverse=True):
                        name = d.name or "?"
                        self._push(
                            f"  📡 {name}  [{d.address}]  RSSI={d.rssi}\n")
            except Exception as e:
                self._push(f"[UI] Erreur scan : {e}\n")
            finally:
                loop.close()
                self.after(0, lambda: self._scan_btn.config(
                    state=tk.NORMAL, text="🔍  Scan"))

        threading.Thread(target=_run, daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────────
    #  BLE CONNECT / DISCONNECT
    # ─────────────────────────────────────────────────────────────────────────
    def _toggle_connection(self):
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        target = self._device_var.get().strip()
        if not target:
            messagebox.showerror("Erreur", "Entrez un nom ou une adresse MAC")
            return
        self._stop_event.clear()
        self._connect_btn.config(state=tk.DISABLED, text="⏳ Connecting…")
        self._ble_thread = threading.Thread(
            target=self._ble_thread_entry, args=(target,), daemon=True)
        self._ble_thread.start()

    def _disconnect(self):
        self._stop_event.set()
        if self._ble_loop and self._ble_loop.is_running():
            self._ble_loop.call_soon_threadsafe(self._ble_loop.stop)
        self._set_status(False)

    # ─────────────────────────────────────────────────────────────────────────
    #  BLE THREAD  (boucle asyncio dédiée)
    # ─────────────────────────────────────────────────────────────────────────
    def _ble_thread_entry(self, target: str):
        """Thread dédié : sa propre boucle asyncio pour bleak."""
        self._ble_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._ble_loop)
        try:
            self._ble_loop.run_until_complete(
                self._ble_run(target))
        except Exception as e:
            self._push(f"[UI] Erreur fatale BLE : {e}\n")
        finally:
            self._ble_loop.close()
            self._ble_loop = None
            self._push(None)  # signal → déconnecté

    async def _ble_run(self, target: str):
        """Connexion + écoute notifications, avec reconnexion automatique."""
        while not self._stop_event.is_set():

            # ── Résolution de l'adresse ───────────────────────────────────
            device = await self._resolve(target)
            if device is None:
                self._push(
                    f"[UI] « {target} » introuvable — "
                    f"retry dans {RECONNECT_DELAY_S} s\n")
                self._push(None)
                await asyncio.sleep(RECONNECT_DELAY_S)
                continue

            # ── Connexion ─────────────────────────────────────────────────
            addr_str = device if isinstance(device, str) else device.address
            self._push(f"[UI] Connexion BLE à {addr_str}…\n")
            try:
                async with BleakClient(device, timeout=20.0) as client:
                    if not client.is_connected:
                        raise ConnectionError("échec connexion GATT")

                    self._client = client
                    self._push(f"[UI] ✅ Connecté à {addr_str}\n")
                    self.after(0, lambda: self._set_status(True))

                    # ── Trouver la caractéristique NUS TX ─────────────────
                    tx_char = self._find_tx_char(client)
                    if tx_char is None:
                        self._push(
                            "[UI] ⚠️ Caractéristique NUS TX introuvable. "
                            "Services disponibles :\n")
                        for svc in client.services:
                            self._push(f"  service {svc.uuid}\n")
                            for c in svc.characteristics:
                                self._push(
                                    f"    char {c.uuid}  "
                                    f"props={c.properties}\n")
                        await asyncio.sleep(RECONNECT_DELAY_S)
                        continue

                    self._push(
                        f"[UI] Abonné aux notifications : {tx_char.uuid}\n")

                    # ── Callback de notification ──────────────────────────
                    def _on_notify(_sender, data: bytearray):
                        text = data.decode("utf-8", errors="replace")
                        for line in text.splitlines(keepends=True):
                            if line.strip():
                                self._push(
                                    line if line.endswith("\n")
                                    else line + "\n")

                    await client.start_notify(tx_char.uuid, _on_notify)

                    # ── Maintien de la connexion ──────────────────────────
                    while (not self._stop_event.is_set()
                           and client.is_connected):
                        await asyncio.sleep(0.5)

                    try:
                        await client.stop_notify(tx_char.uuid)
                    except Exception:
                        pass

            except Exception as e:
                self._push(f"[UI] Connexion perdue : {e}\n")

            # ── Déconnexion / retry ───────────────────────────────────────
            self._client = None
            self._push(None)
            self.after(0, lambda: self._set_status(False))

            if not self._stop_event.is_set():
                self._push(
                    f"[UI] Reconnexion dans {RECONNECT_DELAY_S} s…\n")
                await asyncio.sleep(RECONNECT_DELAY_S)

    # ── Helpers BLE ───────────────────────────────────────────────────────────
    async def _resolve(self, target: str):
        """Nom → BLEDevice objet (ou adresse MAC brute si déjà une MAC)."""
        if len(target) == 17 and target.count(":") == 5:
            return target
        self._push(f"[UI] Recherche de « {target} »…\n")
        devs = await BleakScanner.discover(timeout=8.0, return_adv=False)
        for d in devs:
            if d.name and target.lower() in d.name.lower():
                self._push(f"[UI] Trouvé : {d.name} [{d.address}]\n")
                return d   # retourne le BLEDevice, pas juste l'adresse
        return None

    @staticmethod
    def _find_tx_char(client: BleakClient):
        """Cherche la caractéristique NUS TX dans les services GATT."""
        for svc in client.services:
            for c in svc.characteristics:
                if c.uuid == NUS_TX_CHAR_UUID:
                    return c
        # Fallback : première caractéristique avec « notify »
        for svc in client.services:
            for c in svc.characteristics:
                if "notify" in c.properties:
                    return c
        return None

    # ─────────────────────────────────────────────────────────────────────────
    #  LOG QUEUE  (thread-safe : BLE thread → main thread Tkinter)
    # ─────────────────────────────────────────────────────────────────────────
    def _push(self, msg: str | None):
        """Appelé depuis n'importe quel thread. None = signal de statut."""
        self._log_queue.put(msg)

    def _poll_queue_loop(self):
        """Vide la queue et injecte dans le widget Text (thread principal)."""
        try:
            while True:
                item = self._log_queue.get_nowait()
                if item is None:
                    self._refresh_status()
                else:
                    self._append(item)
        except queue.Empty:
            pass
        self._poll_id = self.after(30, self._poll_queue_loop)

    # ─────────────────────────────────────────────────────────────────────────
    #  AFFICHAGE LOGS
    # ─────────────────────────────────────────────────────────────────────────
    def _append(self, line: str):
        self._log.config(state=tk.NORMAL)
        if self._timestamps.get():
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self._log.insert(tk.END, f"[{ts}] ", "ts")
        self._log.insert(tk.END,
                         line if line.endswith("\n") else line + "\n",
                         self._tag_for(line))
        self._log.config(state=tk.DISABLED)
        self._line_count += 1
        self._lines_lbl.config(text=f"{self._line_count} lignes")
        if self._autoscroll.get():
            self._log.see(tk.END)

    @staticmethod
    def _tag_for(line: str) -> str:
        lo = line.lower()
        if any(k in lo for k in ("error", "fail", "erreur")):
            return "error"
        if any(k in lo for k in ("warn", "warning", "attention")):
            return "warn"
        if any(k in lo for k in ("[ui]", "connecté", "connected", "✅")):
            return "ok"
        if any(k in lo for k in ("[bt]", "[ble]", "notify", "scan")):
            return "info"
        return "normal"

    def _clear_logs(self):
        self._log.config(state=tk.NORMAL)
        self._log.delete("1.0", tk.END)
        self._log.config(state=tk.DISABLED)
        self._line_count = 0
        self._lines_lbl.config(text="0 lignes")

    # ─────────────────────────────────────────────────────────────────────────
    #  STATUT
    # ─────────────────────────────────────────────────────────────────────────
    def _refresh_status(self):
        ok = self._client is not None and self._client.is_connected
        self._set_status(ok)

    def _set_status(self, connected: bool):
        self._connected = connected
        if connected:
            self._dot.config(fg=GREEN)
            self._status_lbl.config(
                text=f"Connecté BLE — {self._device_var.get()}", fg=GREEN)
            self._connect_btn.config(
                text="✕  Disconnect", bg=RED, fg=BG, state=tk.NORMAL)
        else:
            self._dot.config(fg=RED)
            self._status_lbl.config(text="Déconnecté", fg=RED)
            self._connect_btn.config(
                text="⚡  Connect BLE", bg=ACCENT, fg=BG, state=tk.NORMAL)

    # ─────────────────────────────────────────────────────────────────────────
    #  FERMETURE PROPRE
    # ─────────────────────────────────────────────────────────────────────────
    def _on_close(self):
        self._disconnect()
        if self._poll_id:
            self.after_cancel(self._poll_id)
        self.destroy()


# ── Entrypoint ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Eurobot BLE Log Viewer (direct BLE via bleak)")
    parser.add_argument("--device", default=DEFAULT_DEVICE_NAME,
                        help="Nom BLE de l'appareil (défaut: Eurobot)")
    parser.add_argument("--address", default=None,
                        help="Adresse MAC BLE directe (ex: AA:BB:CC:DD:EE:FF)")
    args = parser.parse_args()

    app = EurobotBLEViewer(
        device_name=args.device,
        device_address=args.address)
    app.mainloop()


if __name__ == "__main__":
    main()
