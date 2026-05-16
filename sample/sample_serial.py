"""
MDD x2 Serial Viewer
====================
Packet format (MCU -> PC):
  [0xAA][0x55][DEV_ID][12][deg0L][deg0H]...[deg3L][deg3H][lsw0][lsw1][lsw2][lsw3][XOR]
  DEV_ID: 0x01=MDD1, 0x02=MDD2
  Sent at 10ms only while CAN is active.

Usage:
  python viewer.py
  python viewer.py COM14
"""

import sys
import time
import struct
import threading
import serial
import serial.tools.list_ports

try:
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live
    from rich.panel import Panel
    from rich.text import Text
    RICH = True
except ImportError:
    RICH = False

BAUD        = 115200
PKT_H1      = 0xAA
PKT_H2      = 0x55
DEV_MDD1    = 0x01
DEV_MDD2    = 0x02
PKT_DATA_LEN = 12
TIMEOUT_S   = 0.5   # offline if no packet for this long


class MddState:
    def __init__(self, name):
        self.name    = name
        self.deg     = [0.0] * 4
        self.lsw     = [0]   * 4
        self.count   = 0
        self.last_rx = 0.0

    def update(self, data: bytes):
        for i in range(4):
            raw = struct.unpack_from("<h", data, i * 2)[0]
            self.deg[i] = raw / 10.0
        self.lsw     = list(data[8:12])
        self.count  += 1
        self.last_rx = time.time()

    def online(self):
        return self.last_rx > 0 and (time.time() - self.last_rx) < TIMEOUT_S


g_mdd1 = MddState("MDD1")
g_mdd2 = MddState("MDD2")


def parse_stream(port: serial.Serial):
    buf = bytearray()
    while True:
        chunk = port.read(256)
        if not chunk:
            continue
        buf.extend(chunk)

        while True:
            if len(buf) < 5:
                break
            if buf[0] != PKT_H1 or buf[1] != PKT_H2:
                buf.pop(0)
                continue

            data_len = buf[3]
            total    = 5 + data_len   # header(4) + data + checksum(1)
            if len(buf) < total:
                break

            dev_id  = buf[2]
            data    = bytes(buf[4 : 4 + data_len])
            cs_recv = buf[4 + data_len]

            cs = dev_id ^ data_len
            for b in data:
                cs ^= b
            cs &= 0xFF

            if cs != cs_recv:
                buf.pop(0)
                continue

            if data_len == PKT_DATA_LEN:
                if dev_id == DEV_MDD1:
                    g_mdd1.update(data)
                elif dev_id == DEV_MDD2:
                    g_mdd2.update(data)

            del buf[:total]


def make_panel(state: MddState) -> "Panel":
    if not state.online():
        return Panel(
            Text("OFFLINE", style="bold red", justify="center"),
            title=f"[bold]{state.name}[/bold]",
        )

    t = Table(show_header=True, header_style="bold cyan", box=None)
    t.add_column("ch",    justify="center", width=6)
    t.add_column("deg",   justify="right",  width=9)
    t.add_column("lsw",   justify="center", width=8)

    for i in range(4):
        lsw_str = Text("ON",  style="bold red") if state.lsw[i] \
                  else Text("off", style="dim")
        t.add_row(f"M{i+1}", f"{state.deg[i]:+.1f}", lsw_str)

    age = time.time() - state.last_rx
    return Panel(
        t,
        title=f"[bold green]{state.name}[/bold green]  ONLINE",
        subtitle=f"cnt:{state.count}  age:{age:.2f}s",
    )


def run_rich(port: serial.Serial):
    from rich.console import Console, Group
    console = Console()
    stop    = threading.Event()
    threading.Thread(target=parse_stream, args=(port,), daemon=True).start()

    try:
        import msvcrt
        def kb():
            while not stop.is_set():
                if msvcrt.kbhit():
                    if msvcrt.getwch() in ('q', 'Q'):
                        stop.set()
                time.sleep(0.02)
        threading.Thread(target=kb, daemon=True).start()
    except ImportError:
        pass

    hint = "Press [bold red]q[/bold red] to quit"
    with Live(console=console, refresh_per_second=20) as live:
        while not stop.is_set():
            live.update(Group(
                Panel(
                    __import__('rich.columns', fromlist=['Columns']).Columns(
                        [make_panel(g_mdd1), make_panel(g_mdd2)]
                    ),
                    title="[bold]MDD Monitor[/bold]",
                ),
                hint,
            ))
            time.sleep(0.05)


def run_simple(port: serial.Serial):
    threading.Thread(target=parse_stream, args=(port,), daemon=True).start()
    print("Receiving... Ctrl+C to quit\n")
    while True:
        time.sleep(0.5)
        for s in (g_mdd1, g_mdd2):
            status = "ONLINE" if s.online() else "OFFLINE"
            deg = "  ".join(f"M{i+1}:{s.deg[i]:+.1f}" for i in range(4))
            lsw = "  ".join(f"SW{i+1}:{'ON' if s.lsw[i] else 'off'}" for i in range(4))
            print(f"[{s.name}] {status}  {deg}  {lsw}")
        print()


def select_port() -> str:
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found")
        sys.exit(1)
    if len(ports) == 1:
        print(f"Auto-select: {ports[0].device}  ({ports[0].description})")
        return ports[0].device
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    idx = int(input("Select: ").strip())
    return ports[idx].device


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else select_port()
    try:
        port = serial.Serial(port_name, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open port: {e}")
        sys.exit(1)
    print(f"Connected: {port_name}  {BAUD}bps")
    try:
        if RICH:
            run_rich(port)
        else:
            print("pip install rich for better display\n")
            run_simple(port)
    except KeyboardInterrupt:
        print("\nQuit")
    finally:
        port.close()


if __name__ == "__main__":
    main()