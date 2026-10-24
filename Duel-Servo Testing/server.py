#!/usr/bin/env python3
"""Browser <-> serial bridge for the dual-stepper printer-board rig.

Uses only the Python stdlib + pyserial. Run:  python3 server.py
Then open http://localhost:8765
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial
from serial.tools import list_ports

HERE = Path(__file__).parent
PORT = 8765

ser_lock = threading.Lock()
ser = None          # active serial.Serial or None
ser_port_name = None
last_error = ""


def open_port(port_name):
    """Open the serial port and wait for the sketch's READY banner."""
    global ser, ser_port_name, last_error
    close_port()
    try:
        s = serial.Serial(port_name, 115200, timeout=0.1)
    except serial.SerialException as e:
        last_error = str(e)
        return False
    # Opening the port resets the Uno; give the sketch time to boot.
    deadline = time.time() + 4.0
    banner = ""
    while time.time() < deadline:
        line = s.readline().decode(errors="replace").strip()
        if "READY" in line:
            banner = line
            break
    if not banner:
        # Sketch may already be running (no reset on some setups) — ping it.
        s.reset_input_buffer()
        s.write(b"P\n")
        reply = s.readline().decode(errors="replace").strip()
        if "PONG" not in reply:
            s.close()
            last_error = "No response from board. Is the sketch uploaded?"
            return False
    ser = s
    ser_port_name = port_name
    last_error = ""
    return True


def close_port():
    global ser, ser_port_name
    if ser is not None:
        try:
            # Stop both motors and release the drivers before leaving.
            ser.write(b"X\n")
            time.sleep(0.05)
            ser.close()
        except serial.SerialException:
            pass
    ser = None
    ser_port_name = None


def send_command(cmd):
    """Write one command line and return the board's reply."""
    global last_error
    if ser is None:
        return {"ok": False, "reply": "not connected"}
    try:
        ser.reset_input_buffer()
        ser.write((cmd.strip() + "\n").encode())
        reply = ser.readline().decode(errors="replace").strip()
        return {"ok": reply.startswith("OK"), "reply": reply}
    except serial.SerialException as e:
        last_error = str(e)
        close_port()
        return {"ok": False, "reply": f"serial error: {e}"}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # keep the terminal quiet

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = (HERE / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/ports":
            ports = [
                {"device": p.device, "description": p.description}
                for p in list_ports.comports()
                if "Bluetooth" not in p.device
            ]
            self._json({"ports": ports})
        elif self.path == "/api/status":
            self._json({
                "connected": ser is not None,
                "port": ser_port_name,
                "error": last_error,
            })
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        if self.path == "/api/connect":
            data = self._read_body()
            with ser_lock:
                ok = open_port(data.get("port", ""))
            self._json({"ok": ok, "error": last_error})
        elif self.path == "/api/disconnect":
            with ser_lock:
                close_port()
            self._json({"ok": True})
        elif self.path == "/api/send":
            data = self._read_body()
            with ser_lock:
                result = send_command(data.get("cmd", ""))
            self._json(result)
        else:
            self._json({"error": "not found"}, 404)


def main():
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"Servo control interface: http://localhost:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        with ser_lock:
            close_port()


if __name__ == "__main__":
    main()
