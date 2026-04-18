"""
ui.py  —  Eurobot BLE Bridge — Dual Device
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Connects to TWO ESP32 BLE devices simultaneously via Nordic UART Service.
Each device has its own log panel and command input.

Dépendances :  pip install bleak
"""

import asyncio
import queue
import threading
import tkinter as tk
from tkinter import ttk
from datetime import datetime

from bleak import BleakClient, BleakScanner

# ── BLE NUS UUIDs ─────────────────────────────────────────────────────────────
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC → ESP32
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 → PC

TARGET_NAMES    = ["Eurobot", "TicTac"]   # one per panel
MAX_LOG_LINES   = 800
RECONNECT_S     = 3

# ── Palette (Catppuccin Mocha) ────────────────────────────────────────────────
BG        = "#1e1e2e"
TEXT_BG   = "#181825"
TEXT_FG   = "#cdd6f4"
ACCENT    = "#89b4fa"
GREEN     = "#a6e3a1"
RED       = "#f38ba8"
YELLOW    = "#f9e2af"
DIM       = "#6c7086"
BTN_BG    = "#313244"
TITLE_CLR = "#cba6f7"
BORDER    = "#45475a"


# ═════════════════════════════════════════════════════════════════════════════
#  DevicePanel — one per BLE device (log + cmd + quick buttons)
# ═════════════════════════════════════════════════════════════════════════════
class DevicePanel:
    """Self-contained panel for a single BLE device."""

    def __init__(self, parent: tk.Frame, label: str, app: "App",
                 target_name: str = "Eurobot"):
        self.app = app
        self.label = label
        self.target_name = target_name
        self.client: BleakClient | None = None
        self.address: str | None = None
        self.connected = False
        self._rx_char = None
        self._cmd_history: list[str] = []
        self._cmd_idx = -1
        self._line_count = 0
        self._log_queue: queue.Queue[str | None] = queue.Queue()

        # ── outer frame ──────────────────────────────────────────────────
        self.frame = tk.LabelFrame(
            parent, text=f"  {label} — disconnected  ",
            bg=BG, fg=DIM, font=("Segoe UI", 11, "bold"),
            labelanchor="n", padx=6, pady=4)
        self.frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ── status + connect row ─────────────────────────────────────────
        row = tk.Frame(self.frame, bg=BG)
        row.pack(fill=tk.X, pady=(0, 4))

        self._dot = tk.Label(row, text="●", font=("Segoe UI", 11),
                             bg=BG, fg=RED)
        self._dot.pack(side=tk.LEFT)
        self._status_lbl = tk.Label(row, text="Disconnected",
                                    font=("Segoe UI", 9), bg=BG, fg=RED)
        self._status_lbl.pack(side=tk.LEFT, padx=4)

        self._lines_lbl = tk.Label(row, text="0", font=("Segoe UI", 8),
                                   bg=BG, fg=DIM)
        self._lines_lbl.pack(side=tk.RIGHT, padx=4)

        self._connect_btn = tk.Button(
            row, text="⚡ Connect", bg=ACCENT, fg=BG,
            font=("Segoe UI", 9, "bold"), relief=tk.FLAT,
            padx=8, pady=2, cursor="hand2",
            command=lambda: app.schedule(self._toggle()))
        self._connect_btn.pack(side=tk.RIGHT, padx=2)

        tk.Button(row, text="🗑", bg=BTN_BG, fg=TEXT_FG,
                  font=("Segoe UI", 9), relief=tk.FLAT, padx=6,
                  cursor="hand2", command=self._clear_log
                  ).pack(side=tk.RIGHT, padx=2)

        # ── log area ─────────────────────────────────────────────────────
        log_frame = tk.Frame(self.frame, bg=BG)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self._log = tk.Text(
            log_frame, bg=TEXT_BG, fg=TEXT_FG, font=("Consolas", 10),
            relief=tk.FLAT, wrap=tk.WORD, state=tk.DISABLED,
            selectbackground=BTN_BG, insertbackground=TEXT_FG,
            padx=6, pady=4)
        self._log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        sb = tk.Scrollbar(log_frame, command=self._log.yview,
                          bg=BTN_BG, troughcolor=BG, relief=tk.FLAT)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._log.config(yscrollcommand=sb.set)

        for tag, fg in [("ts", DIM), ("info", TEXT_FG), ("fsm", ACCENT),
                        ("error", RED), ("warn", YELLOW), ("ok", GREEN),
                        ("cmd_echo", TITLE_CLR)]:
            self._log.tag_configure(tag, foreground=fg)

        # ── quick buttons ────────────────────────────────────────────────
        qf = tk.Frame(self.frame, bg=BG)
        qf.pack(fill=tk.X, pady=(4, 2))

        for txt, clr in [("FORWARD 255", GREEN), ("BACKWARD 255", YELLOW),
                         ("ROTATE 90", ACCENT), ("ROTATE -90", ACCENT),
                         ("DEPLOY", GREEN), ("RETRACT", YELLOW),
                         ("STOP", RED), ("STATUS", GREEN), ("RESET", YELLOW),
                         ("RELAISON", RED), 
                         ("RELAISOFF", RED),
                         ]:
            tk.Button(qf, text=txt, bg=BTN_BG, fg=clr,
                      font=("Consolas", 8), relief=tk.FLAT,
                      padx=6, pady=1, cursor="hand2",
                      command=lambda c=txt: self._send_quick(c)
                      ).pack(side=tk.LEFT, padx=2)

        tk.Button(qf, text="PING", bg=BTN_BG, fg=TITLE_CLR,
                  font=("Consolas", 8, "bold"), relief=tk.FLAT,
                  padx=6, pady=1, cursor="hand2",
                  command=self._send_ping
                  ).pack(side=tk.LEFT, padx=2)

        self._ping_time = None  # for RTT measurement

        # ── command input ────────────────────────────────────────────────
        cf = tk.Frame(self.frame, bg=BG)
        cf.pack(fill=tk.X, pady=(2, 0))

        tk.Label(cf, text="bleSerial >", bg=BG, fg=ACCENT,
                 font=("Consolas", 10, "bold")).pack(side=tk.LEFT)

        self._cmd_var = tk.StringVar()
        self._cmd_entry = tk.Entry(
            cf, textvariable=self._cmd_var, bg=TEXT_BG, fg=TEXT_FG,
            insertbackground=ACCENT, relief=tk.FLAT, font=("Consolas", 10),
            highlightbackground=BORDER, highlightthickness=1)
        self._cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 4))
        self._cmd_entry.bind("<Return>", lambda _: self._send_entry())
        self._cmd_entry.bind("<Up>", lambda _: self._hist_prev())
        self._cmd_entry.bind("<Down>", lambda _: self._hist_next())

        tk.Button(cf, text="▶", bg=GREEN, fg=BG,
                  font=("Segoe UI", 9, "bold"), relief=tk.FLAT,
                  padx=8, cursor="hand2",
                  command=self._send_entry).pack(side=tk.LEFT)

        # ── start polling ────────────────────────────────────────────────
        self._poll()

    # ── log helpers ──────────────────────────────────────────────────────
    def _push(self, msg: str | None):
        self._log_queue.put(msg)

    def _poll(self):
        try:
            while True:
                item = self._log_queue.get_nowait()
                if item is None:
                    self._refresh_ui_status()
                else:
                    self._append(item)
        except queue.Empty:
            pass
        self.frame.after(30, self._poll)

    def _append(self, line: str):
        self._log.config(state=tk.NORMAL)
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.insert(tk.END, f"[{ts}] ", "ts")
        self._log.insert(tk.END,
                         line if line.endswith("\n") else line + "\n",
                         self._tag_for(line))
        # trim old lines
        total = int(self._log.index("end-1c").split(".")[0])
        if total > MAX_LOG_LINES:
            self._log.delete("1.0", f"{total - MAX_LOG_LINES}.0")
        self._log.config(state=tk.DISABLED)
        self._log.see(tk.END)
        self._line_count += 1
        self._lines_lbl.config(text=str(self._line_count))

    def _append_cmd(self, line: str):
        self._log.config(state=tk.NORMAL)
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.insert(tk.END, f"[{ts}] ", "ts")
        self._log.insert(tk.END, line + "\n", "cmd_echo")
        self._log.config(state=tk.DISABLED)
        self._log.see(tk.END)
        self._line_count += 1
        self._lines_lbl.config(text=str(self._line_count))

    @staticmethod
    def _tag_for(line: str) -> str:
        lo = line.lower()
        if any(k in lo for k in ("error", "fail", "emergency")):
            return "error"
        if any(k in lo for k in ("warn", "warning")):
            return "warn"
        if any(k in lo for k in ("✅", "complete", "done", "ready")):
            return "ok"
        if any(k in lo for k in ("[fsm]", "dispatch", "exec_")):
            return "fsm"
        return "info"

    def _clear_log(self):
        self._log.config(state=tk.NORMAL)
        self._log.delete("1.0", tk.END)
        self._log.config(state=tk.DISABLED)
        self._line_count = 0
        self._lines_lbl.config(text="0")

    # ── BLE connection ───────────────────────────────────────────────────
    async def _toggle(self):
        if self.connected:
            await self._disconnect()
        else:
            await self._connect()

    async def _connect(self):
        self._push("[UI] Scanning…\n")
        self._set_status("Scanning…", YELLOW)

        devices = await BleakScanner.discover(timeout=5.0)
        candidates = [d for d in devices if d.name and self.target_name in d.name]

        # exclude addresses already used by the other panel
        used = {p.address for p in self.app.panels
                if p is not self and p.address and p.connected}
        candidates = [d for d in candidates if d.address not in used]

        if not candidates:
            self._push("[UI] No available device found.\n")
            self._set_status("Not found", RED)
            return

        device = candidates[0]
        self.address = device.address
        self._push(f"[UI] Found {device.name} [{device.address}]\n")
        self._set_status("Connecting…", YELLOW)

        try:
            self.client = BleakClient(
                device.address,
                disconnected_callback=self._on_disconnect_cb)
            await self.client.connect(timeout=15.0)

            # find NUS characteristics
            self._rx_char = None
            for svc in self.client.services:
                for c in svc.characteristics:
                    if c.uuid == NUS_TX_UUID:
                        await self.client.start_notify(c, self._on_notify)
                        self._push(f"[UI] Subscribed to TX: {c.uuid}\n")
                    if c.uuid == NUS_RX_UUID:
                        self._rx_char = c
                        self._push(f"[UI] Found RX: {c.uuid}\n")

            self.connected = True
            short = device.address[-5:]
            self._set_status(f"Connected [{short}]", GREEN)
            self._update_title(f"  {self.label} — {device.name} [{short}]  ")
            self._push("[UI] ✅ Connected — type MOVE, ROTATE, STOP, STATUS, RESET\n")

        except Exception as e:
            self._push(f"[UI] Connection failed: {e}\n")
            self._set_status("Failed", RED)
            self.client = None

    async def _disconnect(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
            except Exception:
                pass
        self._cleanup()

    def _on_disconnect_cb(self, _client):
        self.frame.after(0, self._cleanup)
        self._push("[UI] Device disconnected.\n")

    def _cleanup(self):
        self.connected = False
        self.address = None
        self.client = None
        self._rx_char = None
        self._set_status("Disconnected", RED)
        self._update_title(f"  {self.label} — disconnected  ")
        self._connect_btn.config(text="⚡ Connect", bg=ACCENT)

    def _on_notify(self, _sender, data: bytearray):
        text = data.decode("utf-8", errors="replace").strip()
        if text:
            # Measure RTT on PONG reception
            if "[PONG]" in text and self._ping_time is not None:
                rtt = (datetime.now() - self._ping_time).total_seconds() * 1000
                text += f"  ← RTT: {rtt:.1f} ms"
                self._ping_time = None
            self._push(text + "\n")

    # ── send commands ────────────────────────────────────────────────────
    async def _ble_write(self, cmd: str):
        if not self.client or not self.client.is_connected or not self._rx_char:
            self._push("[UI] ⚠️ Not connected — command not sent\n")
            return
        try:
            await self.client.write_gatt_char(
                self._rx_char, (cmd + "\n").encode("utf-8"), response=False)
        except Exception as e:
            self._push(f"[UI] Send error: {e}\n")

    def _send_ping(self):
        """Send PING and start RTT timer."""
        self._ping_time = datetime.now()
        self._send_quick("PING")

    def _send_quick(self, cmd: str):
        self._append_cmd(f">>> {cmd}")
        if self.connected:
            self.app.schedule(self._ble_write(cmd))
        else:
            self._push("[UI] ⚠️ Not connected\n")

    def _send_entry(self):
        cmd = self._cmd_var.get().strip()
        if not cmd:
            return
        self._cmd_history.append(cmd)
        self._cmd_idx = -1
        self._cmd_var.set("")
        self._send_quick(cmd)

    # ── history ──────────────────────────────────────────────────────────
    def _hist_prev(self):
        if not self._cmd_history:
            return
        if self._cmd_idx == -1:
            self._cmd_idx = len(self._cmd_history) - 1
        elif self._cmd_idx > 0:
            self._cmd_idx -= 1
        self._cmd_var.set(self._cmd_history[self._cmd_idx])

    def _hist_next(self):
        if self._cmd_idx == -1:
            return
        if self._cmd_idx < len(self._cmd_history) - 1:
            self._cmd_idx += 1
            self._cmd_var.set(self._cmd_history[self._cmd_idx])
        else:
            self._cmd_idx = -1
            self._cmd_var.set("")

    # ── UI state helpers ─────────────────────────────────────────────────
    def _set_status(self, text: str, color: str):
        self.frame.after(0, lambda: (
            self._dot.config(fg=color),
            self._status_lbl.config(text=text, fg=color),
            self._connect_btn.config(
                text="✕ Disconnect" if self.connected else "⚡ Connect",
                bg=RED if self.connected else ACCENT),
        ))

    def _refresh_ui_status(self):
        ok = self.client is not None and self.connected
        self._set_status(
            f"Connected [{self.address[-5:]}]" if ok and self.address else "Disconnected",
            GREEN if ok else RED)

    def _update_title(self, title: str):
        self.frame.after(0, lambda: self.frame.config(text=title))


# ═════════════════════════════════════════════════════════════════════════════
#  App — main window hosting two DevicePanels + shared async loop
# ═════════════════════════════════════════════════════════════════════════════
class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("BLE Bridge — Dual Device")
        self.root.geometry("1280x660")
        self.root.minsize(900, 480)
        self.root.configure(bg=BG)

        # ── toolbar ──────────────────────────────────────────────────────
        bar = tk.Frame(self.root, bg=BG, pady=6)
        bar.pack(fill=tk.X, padx=12)

        tk.Label(bar, text="Eurobot BLE Bridge",
                 font=("Segoe UI", 14, "bold"), bg=BG, fg=TITLE_CLR
                 ).pack(side=tk.LEFT)

        tk.Button(bar, text="⚡ Connect All", bg=ACCENT, fg=BG,
                  font=("Segoe UI", 9, "bold"), relief=tk.FLAT,
                  padx=10, pady=3, cursor="hand2",
                  command=lambda: self.schedule(self._connect_all())
                  ).pack(side=tk.LEFT, padx=(20, 4))

        tk.Button(bar, text="✕ Disconnect All", bg=RED, fg=BG,
                  font=("Segoe UI", 9, "bold"), relief=tk.FLAT,
                  padx=10, pady=3, cursor="hand2",
                  command=lambda: self.schedule(self._disconnect_all())
                  ).pack(side=tk.LEFT, padx=4)

        # broadcast command
        tk.Label(bar, text="Broadcast:", bg=BG, fg=DIM,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(16, 4))

        self._broadcast_var = tk.StringVar(value="STATUS")
        bc_entry = tk.Entry(bar, textvariable=self._broadcast_var, width=20,
                            bg=TEXT_BG, fg=TEXT_FG, insertbackground=ACCENT,
                            relief=tk.FLAT, font=("Consolas", 10),
                            highlightbackground=BORDER, highlightthickness=1)
        bc_entry.pack(side=tk.LEFT, padx=2)
        bc_entry.bind("<Return>", lambda _: self._broadcast())

        tk.Button(bar, text="▶ All", bg=GREEN, fg=BG,
                  font=("Segoe UI", 9, "bold"), relief=tk.FLAT,
                  padx=8, pady=3, cursor="hand2",
                  command=self._broadcast
                  ).pack(side=tk.LEFT, padx=4)

        # scan button
        self._scan_btn = tk.Button(
            bar, text="🔍 Scan", bg=BTN_BG, fg=TEXT_FG,
            font=("Segoe UI", 9), relief=tk.FLAT,
            padx=8, pady=3, cursor="hand2",
            command=lambda: self.schedule(self._scan()))
        self._scan_btn.pack(side=tk.RIGHT, padx=4)

        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X,
                                                              padx=12, pady=4)

        # ── panels container ─────────────────────────────────────────────
        container = tk.Frame(self.root, bg=BG)
        container.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self.panels: list[DevicePanel] = [
            DevicePanel(container, "Device 1", self, target_name=TARGET_NAMES[0]),
            DevicePanel(container, "Device 2", self, target_name=TARGET_NAMES[1]),
        ]

        # ── async event loop (background thread) ─────────────────────────
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def schedule(self, coro):
        asyncio.run_coroutine_threadsafe(coro, self._loop)

    # ── bulk actions ─────────────────────────────────────────────────────
    async def _connect_all(self):
        for p in self.panels:
            if not p.connected:
                await p._connect()

    async def _disconnect_all(self):
        for p in self.panels:
            if p.connected:
                await p._disconnect()

    async def _scan(self):
        for p in self.panels:
            p._push("[UI] Scanning BLE (5s)…\n")
        devices = await BleakScanner.discover(timeout=5.0)
        for p in self.panels:
            if not devices:
                p._push("[UI] No BLE devices found.\n")
            else:
                p._push(f"[UI] {len(devices)} device(s):\n")
                for d in sorted(devices, key=lambda x: x.rssi or -999,
                                reverse=True):
                    name = d.name or "?"
                    p._push(f"  📡 {name}  [{d.address}]  RSSI={d.rssi}\n")

    def _broadcast(self):
        cmd = self._broadcast_var.get().strip()
        if not cmd:
            return
        for p in self.panels:
            if p.connected:
                p._append_cmd(f">>> [BROADCAST] {cmd}")
                self.schedule(p._ble_write(cmd))

    # ── run / close ──────────────────────────────────────────────────────
    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        for p in self.panels:
            if p.connected and p.client:
                self.schedule(p._disconnect())
        self._loop.call_soon_threadsafe(self._loop.stop)
        self.root.after(300, self.root.destroy)


if __name__ == "__main__":
    App().run()
