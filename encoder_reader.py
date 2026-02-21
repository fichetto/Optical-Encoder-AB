#!/usr/bin/env python3
"""
PMW3901 Optical Encoder - Desktop Reader
=========================================
Legge i contatori X/Y dal sensore via Modbus RTU su USB CDC (ESP32-S3).

Dipendenze (solo pyserial):
    pip install pyserial

Uso:
    python encoder_reader.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct
import threading
import time
import sys

# ──────────────────────────────────────────────────────────────────────────────
# Modbus RTU - implementazione minimale (nessuna dipendenza extra)
# ──────────────────────────────────────────────────────────────────────────────
SLAVE_ID = 1


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _read_regs(ser: serial.Serial, start: int, count: int) -> list | None:
    """FC 04 – Read Input Registers."""
    req = struct.pack('>BBHH', SLAVE_ID, 0x04, start, count)
    req += struct.pack('<H', _crc16(req))
    ser.reset_input_buffer()
    ser.write(req)
    expected = 3 + count * 2 + 2
    raw = ser.read(expected)
    if len(raw) < expected:
        return None
    if _crc16(raw[:-2]) != struct.unpack_from('<H', raw, -2)[0]:
        return None
    if raw[0] != SLAVE_ID or raw[1] != 0x04:
        return None
    return [struct.unpack_from('>H', raw, 3 + i * 2)[0] for i in range(count)]


def _write_reg(ser: serial.Serial, reg: int, val: int) -> bool:
    """FC 06 – Write Single Register."""
    req = struct.pack('>BBHH', SLAVE_ID, 0x06, reg, val)
    req += struct.pack('<H', _crc16(req))
    ser.reset_input_buffer()
    ser.write(req)
    rsp = ser.read(8)
    if len(rsp) < 8:
        return False
    return _crc16(rsp[:-2]) == struct.unpack_from('<H', rsp, 6)[0]


def _to_int32(high: int, low: int) -> int:
    raw = ((high & 0xFFFF) << 16) | (low & 0xFFFF)
    return struct.unpack('>i', struct.pack('>I', raw))[0]


def list_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


# ──────────────────────────────────────────────────────────────────────────────
# Thread di lettura
# ──────────────────────────────────────────────────────────────────────────────
class ReaderThread(threading.Thread):
    def __init__(self, ser: serial.Serial, callback, interval_ms: int = 50):
        super().__init__(daemon=True)
        self.ser = ser
        self.callback = callback
        self.interval = interval_ms / 1000.0
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            t0 = time.perf_counter()
            regs = _read_regs(self.ser, 0x0000, 5)
            if regs is not None:
                x     = _to_int32(regs[0], regs[1])
                y     = _to_int32(regs[2], regs[3])
                squal = regs[4]
                self.callback(x, y, squal, ok=True)
            else:
                self.callback(0, 0, 0, ok=False)
            elapsed = time.perf_counter() - t0
            wait = self.interval - elapsed
            if wait > 0:
                time.sleep(wait)

    def stop(self):
        self._stop.set()


# ──────────────────────────────────────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────────────────────────────────────
class App(tk.Tk):
    POLL_INTERVALS = {"20 Hz": 50, "10 Hz": 100, "5 Hz": 200, "1 Hz": 1000}
    BAUD_RATES     = ["921600", "460800", "115200", "57600", "19200", "9600"]

    def __init__(self):
        super().__init__()
        self.title("PMW3901 Optical Encoder Reader")
        self.resizable(False, False)
        self.configure(bg="#1e1e2e")

        self._ser    : serial.Serial | None = None
        self._reader : ReaderThread  | None = None
        self._led    = False
        self._errors = 0
        self._samples= 0
        self._ref_x  = 0
        self._ref_y  = 0

        self._build_ui()
        self._refresh_ports()

    # ── Layout ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        PAD  = 10
        BG   = "#1e1e2e"
        FG   = "#cdd6f4"
        CARD = "#313244"
        ACC  = "#89b4fa"
        RED  = "#f38ba8"
        GRN  = "#a6e3a1"
        YLW  = "#f9e2af"

        self._fg  = FG
        self._grn = GRN
        self._red = RED
        self._ylw = YLW
        self._acc = ACC

        # ── Barra connessione ──────────────────────────────────────────────
        bar = tk.Frame(self, bg=BG, pady=PAD)
        bar.pack(fill="x", padx=PAD)

        tk.Label(bar, text="Porta:", bg=BG, fg=FG).pack(side="left")
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(bar, textvariable=self._port_var, width=10, state="readonly")
        self._port_cb.pack(side="left", padx=(4, 12))

        tk.Label(bar, text="Baud:", bg=BG, fg=FG).pack(side="left")
        self._baud_var = tk.StringVar(value="921600")
        self._baud_cb  = ttk.Combobox(bar, textvariable=self._baud_var, values=self.BAUD_RATES,
                                      width=8, state="readonly")
        self._baud_cb.pack(side="left", padx=(4, 12))

        tk.Label(bar, text="Poll:", bg=BG, fg=FG).pack(side="left")
        self._poll_var = tk.StringVar(value="20 Hz")
        self._poll_cb  = ttk.Combobox(bar, textvariable=self._poll_var,
                                      values=list(self.POLL_INTERVALS.keys()),
                                      width=7, state="readonly")
        self._poll_cb.pack(side="left", padx=(4, 12))

        self._btn_refresh = tk.Button(bar, text="⟳", bg=CARD, fg=FG, relief="flat",
                                      command=self._refresh_ports, cursor="hand2")
        self._btn_refresh.pack(side="left", padx=(0, 6))

        self._btn_conn = tk.Button(bar, text="Connetti", bg=ACC, fg="#1e1e2e",
                                   relief="flat", padx=10, font=("", 9, "bold"),
                                   command=self._toggle_connection, cursor="hand2")
        self._btn_conn.pack(side="left")

        self._lbl_status = tk.Label(bar, text="● Disconnesso", bg=BG, fg=RED, padx=12)
        self._lbl_status.pack(side="left")

        # ── Valori principali ──────────────────────────────────────────────
        vals = tk.Frame(self, bg=BG)
        vals.pack(fill="x", padx=PAD, pady=(0, PAD))

        self._x_var     = tk.StringVar(value="---")
        self._y_var     = tk.StringVar(value="---")
        self._squal_var = tk.StringVar(value="---")

        for col, label, var, color in [
            (0, "X  (pixel)", self._x_var, ACC),
            (1, "Y  (pixel)", self._y_var, ACC),
            (2, "SQUAL",      self._squal_var, YLW),
        ]:
            card = tk.Frame(vals, bg=CARD, padx=16, pady=10, relief="flat")
            card.grid(row=0, column=col, padx=6, sticky="ew")
            vals.columnconfigure(col, weight=1)
            tk.Label(card, text=label, bg=CARD, fg=FG, font=("", 8)).pack()
            tk.Label(card, textvariable=var, bg=CARD, fg=color,
                     font=("Consolas", 28, "bold")).pack()

        # ── Delta / Velocità ───────────────────────────────────────────────
        delt = tk.Frame(self, bg=BG)
        delt.pack(fill="x", padx=PAD, pady=(0, PAD))

        self._dx_var = tk.StringVar(value="ΔX: ---")
        self._dy_var = tk.StringVar(value="ΔY: ---")
        self._rate_var = tk.StringVar(value="Rate: ---")
        self._err_var  = tk.StringVar(value="Errori: 0")

        for col, var in enumerate([self._dx_var, self._dy_var, self._rate_var, self._err_var]):
            tk.Label(delt, textvariable=var, bg=BG, fg=FG, font=("Consolas", 10)).grid(
                row=0, column=col, padx=20)

        # ── Barra qualità SQUAL ────────────────────────────────────────────
        qf = tk.Frame(self, bg=BG)
        qf.pack(fill="x", padx=PAD, pady=(0, PAD))
        tk.Label(qf, text="SQUAL", bg=BG, fg=FG, width=6, anchor="w").pack(side="left")
        self._squal_bar = ttk.Progressbar(qf, maximum=255, length=320, mode="determinate")
        self._squal_bar.pack(side="left", padx=6)
        self._squal_lbl = tk.Label(qf, text="", bg=BG, fg=GRN, width=12, anchor="w")
        self._squal_lbl.pack(side="left")

        # ── Controlli ──────────────────────────────────────────────────────
        ctrl = tk.Frame(self, bg=BG, pady=6)
        ctrl.pack(pady=(0, PAD))

        btn_style = dict(relief="flat", padx=14, pady=6, cursor="hand2",
                         font=("", 9, "bold"))

        self._btn_reset = tk.Button(ctrl, text="Reset X/Y", bg=RED, fg="#1e1e2e",
                                    command=self._reset_counters, **btn_style)
        self._btn_reset.pack(side="left", padx=6)

        self._btn_ref = tk.Button(ctrl, text="Segna Riferimento", bg=YLW, fg="#1e1e2e",
                                  command=self._set_reference, **btn_style)
        self._btn_ref.pack(side="left", padx=6)

        self._btn_led = tk.Button(ctrl, text="LED OFF", bg=CARD, fg=FG,
                                  command=self._toggle_led, **btn_style)
        self._btn_led.pack(side="left", padx=6)

        # ── Grafico mini (canvas) ──────────────────────────────────────────
        self._canvas = tk.Canvas(self, bg=CARD, height=120, highlightthickness=0)
        self._canvas.pack(fill="x", padx=PAD, pady=(0, PAD))
        self._history_x : list[int] = []
        self._history_y : list[int] = []

        # ── Footer ─────────────────────────────────────────────────────────
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", padx=PAD, pady=(0, 6))
        tk.Label(foot, text="Slave ID: 1  |  FC 04  |  Reg 0x0000–0x0004",
                 bg=BG, fg="#6c7086", font=("", 8)).pack(side="left")
        tk.Label(foot, text="R=reset  L=LED  Q=esci",
                 bg=BG, fg="#6c7086", font=("", 8)).pack(side="right")

        # Bind tasti
        self.bind("<r>", lambda e: self._reset_counters())
        self.bind("<R>", lambda e: self._reset_counters())
        self.bind("<l>", lambda e: self._toggle_led())
        self.bind("<L>", lambda e: self._toggle_led())
        self.bind("<q>", lambda e: self._on_close())
        self.bind("<Q>", lambda e: self._on_close())
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Aggiornamento UI periodico (dal thread principale)
        self._pending : dict | None = None
        self._last_x  = 0
        self._last_y  = 0
        self._last_t  = time.perf_counter()
        self._after_update()

    # ── Connessione ───────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = list_ports()
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connection(self):
        if self._ser and self._ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_var.get()
        baud = int(self._baud_var.get())
        if not port:
            messagebox.showwarning("Attenzione", "Seleziona una porta COM.")
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as e:
            messagebox.showerror("Errore", f"Impossibile aprire {port}:\n{e}")
            return

        interval_ms = self.POLL_INTERVALS[self._poll_var.get()]
        self._reader = ReaderThread(self._ser, self._on_data, interval_ms)
        self._reader.start()

        self._btn_conn.config(text="Disconnetti", bg="#f38ba8")
        self._lbl_status.config(text="● Connesso", fg=self._grn)
        self._errors = 0
        self._samples = 0

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._ser:
            self._ser.close()
            self._ser = None
        self._btn_conn.config(text="Connetti", bg=self._acc)
        self._lbl_status.config(text="● Disconnesso", fg=self._red)
        self._x_var.set("---")
        self._y_var.set("---")
        self._squal_var.set("---")

    # ── Callback dati (dal thread reader) ─────────────────────────────────────
    def _on_data(self, x: int, y: int, squal: int, ok: bool):
        # Chiamato dal thread reader – NON modificare widget Tk qui
        self._pending = {"x": x, "y": y, "squal": squal, "ok": ok}
        if not ok:
            self._errors += 1
        else:
            self._samples += 1

    # ── Aggiornamento UI (loop Tk) ────────────────────────────────────────────
    def _after_update(self):
        if self._pending:
            d = self._pending
            self._pending = None

            if d["ok"]:
                x, y, squal = d["x"], d["y"], d["squal"]

                # Applica riferimento
                disp_x = x - self._ref_x
                disp_y = y - self._ref_y

                self._x_var.set(f"{disp_x:+,}")
                self._y_var.set(f"{disp_y:+,}")
                self._squal_var.set(str(squal))

                # Delta
                now = time.perf_counter()
                dt  = now - self._last_t
                if dt > 0:
                    vx = (x - self._last_x) / dt
                    vy = (y - self._last_y) / dt
                    self._dx_var.set(f"ΔX/s: {vx:+.1f}")
                    self._dy_var.set(f"ΔY/s: {vy:+.1f}")
                self._last_x, self._last_y, self._last_t = x, y, now

                # Rate stimato
                iv = self.POLL_INTERVALS.get(self._poll_var.get(), 50)
                self._rate_var.set(f"Poll: {1000//iv} Hz")

                # SQUAL bar + colore
                self._squal_bar["value"] = squal
                if squal >= 80:
                    color, label = self._grn, "Eccellente"
                elif squal >= 40:
                    color, label = self._ylw, "Buono"
                else:
                    color, label = self._red,  "Scarso"
                self._squal_lbl.config(text=label, fg=color)

                # Mini grafico
                self._history_x.append(disp_x)
                self._history_y.append(disp_y)
                if len(self._history_x) > 200:
                    self._history_x.pop(0)
                    self._history_y.pop(0)
                self._draw_graph()

            self._err_var.set(f"Errori: {self._errors}")

        self.after(30, self._after_update)  # ~33 Hz UI refresh

    def _draw_graph(self):
        c = self._canvas
        c.delete("all")
        W = c.winfo_width()
        H = c.winfo_height()
        if W < 10 or H < 10:
            return

        # Griglia centrale
        c.create_line(0, H//2, W, H//2, fill="#45475a", dash=(4, 4))

        def draw_series(data, color):
            if len(data) < 2:
                return
            mn, mx = min(data), max(data)
            span = max(mx - mn, 1)
            pts = []
            for i, v in enumerate(data):
                px = int(i * W / len(data))
                py = int(H - (v - mn) / span * (H - 4) - 2)
                pts.extend([px, py])
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=1.5, smooth=True)

        draw_series(self._history_x, self._acc)
        draw_series(self._history_y, self._grn)

        # Legenda
        c.create_text(6, 8,  text="X", fill=self._acc, anchor="nw", font=("", 8))
        c.create_text(20, 8, text="Y", fill=self._grn, anchor="nw", font=("", 8))

    # ── Comandi Modbus ────────────────────────────────────────────────────────
    def _reset_counters(self):
        if not self._ser or not self._ser.is_open:
            return
        ok = _write_reg(self._ser, 0x0010, 0x0001)
        self._ref_x = self._ref_y = 0
        if not ok:
            messagebox.showwarning("Reset", "Nessuna risposta dal dispositivo.")

    def _set_reference(self):
        """Memorizza il valore corrente come zero di riferimento."""
        if self._pending and self._pending.get("ok"):
            self._ref_x = self._pending["x"]
            self._ref_y = self._pending["y"]

    def _toggle_led(self):
        if not self._ser or not self._ser.is_open:
            return
        self._led = not self._led
        ok = _write_reg(self._ser, 0x0011, 0x0001 if self._led else 0x0000)
        label = "LED ON ●" if self._led else "LED OFF"
        color = self._ylw if self._led else "#313244"
        self._btn_led.config(text=label, bg=color,
                             fg="#1e1e2e" if self._led else self._fg)
        if not ok:
            messagebox.showwarning("LED", "Nessuna risposta dal dispositivo.")

    # ── Chiusura ───────────────────────────────────────────────────────────────
    def _on_close(self):
        self._disconnect()
        self.destroy()


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = App()
    app.mainloop()
