#!/usr/bin/env python3
"""
Visualizzatore per lo stream OV7670 -> ESP32-C3 -> USB CDC.

Legge i pacchetti binari prodotti dallo sketch ov7670_stream.ino, ricostruisce
i frame 100x100 RGB565 e li mostra in una finestra sul PC.

Uso tipico:
    python ov7670_viewer.py                 # porta rilevata automaticamente
    python ov7670_viewer.py --port COM7 --scale 5
    python ov7670_viewer.py --list          # elenca le porte seriali

Tasti nella finestra:
    q / ESC  esci                w  salva l'immagine corrente
    m        mirror              f  flip verticale
    b        color bar di test   r  reinizializza la camera
    + -      luminosita'         [ ]  contrasto
    i        stato del firmware (stampato sul terminale)

Dipendenze: pyserial, numpy. Opzionali: opencv-python (piu' fluido), pillow.
"""

import argparse
import base64
import os
import struct
import sys
import threading
import time
from collections import deque

try:
    import numpy as np
except ImportError:
    sys.exit("Serve numpy:  pip install numpy")

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Serve pyserial:  pip install pyserial")


MAGIC = b"OVC1"
HDR_LEN = 20
PKT_FRAME = 1
PKT_TEXT = 2
FMT_RGB565 = 1
ESPRESSIF_VID = 0x303A


# --------------------------------------------------------------------- porta

def find_port(explicit=None):
    """Restituisce il nome della porta da usare."""
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    if not ports:
        return None
    # 1) VID Espressif (USB Serial/JTAG integrato)
    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device
    # 2) nomi tipici delle CDC native
    for p in ports:
        low = (p.device or "").lower()
        if "acm" in low or "usbmodem" in low:
            return p.device
    return ports[0].device


def print_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("Nessuna porta seriale trovata.")
        return
    for p in ports:
        vid = f"{p.vid:04X}" if p.vid is not None else "----"
        pid = f"{p.pid:04X}" if p.pid is not None else "----"
        print(f"  {p.device:20s}  VID:PID {vid}:{pid}  {p.description}")


def open_serial(port, baud, timeout=0.05):
    """Apre la porta senza toccare DTR/RTS: sulla CDC nativa dell'ESP32-C3
    una sequenza DTR/RTS puo' far ripartire il chip in bootloader."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = 1.0
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    ser.open()
    time.sleep(0.05)
    ser.reset_input_buffer()
    return ser


# ------------------------------------------------------------- parser stream

class PacketReader:
    """Estrae i pacchetti dal flusso seriale, risincronizzandosi sul magic."""

    MAX_BUF = 1 << 20

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.bad_header = 0
        self.bad_payload = 0

    def poll(self):
        waiting = 0
        try:
            waiting = self.ser.in_waiting
        except Exception:
            pass
        chunk = self.ser.read(waiting if waiting else 1)
        if chunk:
            self.buf += chunk
        if len(self.buf) > self.MAX_BUF:            # non dovrebbe servire
            del self.buf[:-(HDR_LEN + 3)]

        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                if len(self.buf) > 3:
                    del self.buf[:-3]               # il magic potrebbe essere a cavallo
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                return

            hdr = bytes(self.buf[:HDR_LEN])
            want = struct.unpack_from("<H", hdr, 18)[0]
            if (sum(hdr[:18]) & 0xFFFF) != want:
                self.bad_header += 1
                del self.buf[:1]                    # magic casuale nei dati
                continue

            plen = struct.unpack_from("<H", hdr, 16)[0]
            total = HDR_LEN + plen + 2
            if len(self.buf) < total:
                return

            payload = bytes(self.buf[HDR_LEN:HDR_LEN + plen])
            chk = struct.unpack_from("<H", self.buf, HDR_LEN + plen)[0]
            del self.buf[:total]

            if plen and (int(np.frombuffer(payload, np.uint8).sum()) & 0xFFFF) != chk:
                self.bad_payload += 1
                continue

            yield hdr, payload


def decode_rgb565(payload, w, h, swap=False):
    """RGB565 big-endian -> array (h, w, 3) uint8."""
    dtype = "<u2" if swap else ">u2"
    v = np.frombuffer(payload, dtype=dtype)
    if v.size != w * h:
        return None
    v = v.reshape(h, w)
    r = ((v >> 11) & 0x1F).astype(np.uint8)
    g = ((v >> 5) & 0x3F).astype(np.uint8)
    b = (v & 0x1F).astype(np.uint8)
    out = np.empty((h, w, 3), np.uint8)
    out[..., 0] = (r << 3) | (r >> 2)      # 5 -> 8 bit
    out[..., 1] = (g << 2) | (g >> 4)      # 6 -> 8 bit
    out[..., 2] = (b << 3) | (b >> 2)
    return out


# ------------------------------------------------------------ thread lettura

class Grabber(threading.Thread):
    """Legge la seriale e tiene pronto solo il frame piu' recente."""

    daemon = True

    def __init__(self, ser, swap=False):
        super().__init__(name="grabber")
        self.ser = ser
        self.swap = swap
        self.reader = PacketReader(ser)
        self._lock = threading.Lock()
        self._frame = None          # (seq, ndarray)
        self._texts = deque(maxlen=32)
        self._stop = threading.Event()
        self.n_frames = 0
        self.n_lost = 0             # frame persi (buchi nella sequenza)
        self.n_late = 0             # frame scartati perche' la GUI era lenta
        self.last_seq = None
        self.stamps = deque(maxlen=60)

    def stop(self):
        self._stop.set()

    def take_frame(self):
        with self._lock:
            f, self._frame = self._frame, None
            return f

    def take_texts(self):
        with self._lock:
            out = list(self._texts)
            self._texts.clear()
            return out

    def fps(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def run(self):
        while not self._stop.is_set():
            try:
                for hdr, payload in self.reader.poll():
                    self._handle(hdr, payload)
            except serial.SerialException as exc:
                with self._lock:
                    self._texts.append(f"[seriale] {exc}")
                self._stop.set()
            except Exception as exc:                 # pragma: no cover
                with self._lock:
                    self._texts.append(f"[errore] {exc!r}")

    def _handle(self, hdr, payload):
        ptype = hdr[5]
        if ptype == PKT_TEXT:
            with self._lock:
                self._texts.append("[esp32] " + payload.decode("utf-8", "replace"))
            return
        if ptype != PKT_FRAME or hdr[6] != FMT_RGB565:
            return

        w, h = struct.unpack_from("<HH", hdr, 8)
        seq = struct.unpack_from("<I", hdr, 12)[0]
        img = decode_rgb565(payload, w, h, self.swap)
        if img is None:
            return

        if self.last_seq is not None and seq > self.last_seq + 1:
            self.n_lost += seq - self.last_seq - 1
        self.last_seq = seq
        self.n_frames += 1
        self.stamps.append(time.monotonic())

        with self._lock:
            if self._frame is not None:
                self.n_late += 1        # la GUI non ha ritirato il precedente
            self._frame = (seq, img)


# ----------------------------------------------------------------- finestre

class CvDisplay:
    """Backend OpenCV: il piu' fluido."""

    name = "opencv"

    def __init__(self, title, w, h, scale, osd):
        import cv2
        self.cv2 = cv2
        self.title, self.w, self.h, self.scale, self.osd = title, w, h, scale, osd
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, w * scale, h * scale)
        self._alive = True

    def show(self, img, status):
        cv2 = self.cv2
        big = cv2.resize(img[:, :, ::-1], (self.w * self.scale, self.h * self.scale),
                         interpolation=cv2.INTER_NEAREST)
        if self.osd:
            cv2.putText(big, status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(big, status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(self.title, big)

    def poll_keys(self):
        keys = []
        while True:
            k = self.cv2.waitKey(1)
            if k == -1:
                break
            if k == 27:
                keys.append("q")
            elif 0 <= k < 256:
                keys.append(chr(k))
        try:
            if self.cv2.getWindowProperty(self.title, self.cv2.WND_PROP_VISIBLE) < 1:
                self._alive = False
        except self.cv2.error:
            self._alive = False
        return keys

    def alive(self):
        return self._alive

    def close(self):
        try:
            self.cv2.destroyWindow(self.title)
            self.cv2.waitKey(1)
        except Exception:
            pass


class TkDisplay:
    """Backend Tkinter: usa Pillow se presente, altrimenti PPM+base64.
    Non richiede nulla oltre alla libreria standard."""

    name = "tkinter"

    def __init__(self, title, w, h, scale, osd):
        import tkinter as tk
        self.tk = tk
        self.title, self.w, self.h, self.scale = title, w, h, scale
        self.root = tk.Tk()
        self.root.title(title)
        self.root.resizable(False, False)
        self.label = tk.Label(self.root, borderwidth=0)
        self.label.pack()
        self._keys = []
        self._alive = True
        self._photo = None
        self.root.bind("<Key>", self._on_key)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        try:
            from PIL import Image, ImageTk
            self._pil = (Image, ImageTk)
        except ImportError:
            self._pil = None

    def _on_key(self, event):
        if event.keysym == "Escape":
            self._keys.append("q")
        elif event.char:
            self._keys.append(event.char)

    def _on_close(self):
        self._alive = False

    def show(self, img, status):
        if self._pil is not None:
            Image, ImageTk = self._pil
            im = Image.fromarray(img)
            if self.scale != 1:
                im = im.resize((self.w * self.scale, self.h * self.scale),
                               Image.NEAREST)
            self._photo = ImageTk.PhotoImage(im)
        else:
            ppm = b"P6\n%d %d\n255\n" % (self.w, self.h) + img.tobytes()
            photo = self.tk.PhotoImage(
                data=base64.b64encode(ppm).decode("ascii"))
            self._photo = photo.zoom(self.scale) if self.scale > 1 else photo
        self.label.configure(image=self._photo)
        self.root.title(f"{self.title} - {status}")

    def poll_keys(self):
        try:
            self.root.update()
        except self.tk.TclError:
            self._alive = False
        keys, self._keys = self._keys, []
        return keys

    def alive(self):
        return self._alive

    def close(self):
        try:
            self.root.destroy()
        except Exception:
            pass


def make_display(backend, title, w, h, scale, osd):
    order = {"auto": ["cv2", "tk"], "cv2": ["cv2"], "tk": ["tk"]}[backend]
    errors = []
    for name in order:
        try:
            if name == "cv2":
                return CvDisplay(title, w, h, scale, osd)
            return TkDisplay(title, w, h, scale, osd)
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    sys.exit("Nessun backend grafico disponibile -> " + " | ".join(errors))


# -------------------------------------------------------------------- salva

def save_frame(img, directory):
    os.makedirs(directory, exist_ok=True)
    stem = time.strftime("ov7670_%Y%m%d_%H%M%S")
    try:
        import cv2
        path = os.path.join(directory, stem + ".png")
        cv2.imwrite(path, img[:, :, ::-1])
    except ImportError:
        try:
            from PIL import Image
            path = os.path.join(directory, stem + ".png")
            Image.fromarray(img).save(path)
        except ImportError:
            path = os.path.join(directory, stem + ".ppm")
            h, w = img.shape[:2]
            with open(path, "wb") as fh:
                fh.write(b"P6\n%d %d\n255\n" % (w, h))
                fh.write(img.tobytes())
    return path


# --------------------------------------------------------------------- main

# tasto locale -> comando da inviare al firmware
FORWARD = {"m": "m", "f": "v", "b": "b", "r": "r", "i": "i", "p": "s",
           "+": "+", "-": "-", "[": "[", "]": "]"}


def main():
    ap = argparse.ArgumentParser(
        description="Visualizza lo stream 100x100 RGB565 dell'OV7670 su ESP32-C3.")
    ap.add_argument("--port", help="porta seriale (default: rilevata da sola)")
    ap.add_argument("--baud", type=int, default=921600,
                    help="ignorato dalla CDC nativa, utile su ponti UART")
    ap.add_argument("--scale", type=int, default=4, help="zoom della finestra")
    ap.add_argument("--backend", choices=["auto", "cv2", "tk"], default="auto")
    ap.add_argument("--swap", action="store_true",
                    help="inverte l'ordine dei byte RGB565")
    ap.add_argument("--save-dir", default="snapshots")
    ap.add_argument("--no-osd", action="store_true",
                    help="non disegnare il testo di stato sull'immagine")
    ap.add_argument("--stats", type=float, default=5.0,
                    help="intervallo in s delle statistiche sul terminale (0=off)")
    ap.add_argument("--list", action="store_true", help="elenca le porte ed esci")
    args = ap.parse_args()

    if args.list:
        print_ports()
        return 0

    port = find_port(args.port)
    if not port:
        print("Nessuna porta seriale trovata. Usa --list per controllare.")
        return 1

    try:
        ser = open_serial(port, args.baud)
    except serial.SerialException as exc:
        print(f"Impossibile aprire {port}: {exc}")
        return 1
    print(f"Porta {port} aperta. In attesa dei frame... (q per uscire)")

    grab = Grabber(ser, args.swap)
    grab.start()

    disp = None
    last_img = None
    t_stats = time.monotonic()
    exit_code = 0

    try:
        while True:
            for line in grab.take_texts():
                print(line)

            got = grab.take_frame()
            if got is not None:
                _, last_img = got
                if disp is None:
                    h, w = last_img.shape[:2]
                    disp = make_display(args.backend, f"OV7670 {w}x{h}", w, h,
                                        max(1, args.scale), not args.no_osd)
                    print(f"Backend grafico: {disp.name}")

            if disp is None:
                if not grab.is_alive():
                    print("Lettura seriale interrotta.")
                    exit_code = 1
                    break
                time.sleep(0.02)
                continue

            if got is not None:
                status = (f"{grab.fps():4.1f} fps  ok:{grab.n_frames} "
                          f"persi:{grab.n_lost} tardivi:{grab.n_late} "
                          f"crc:{grab.reader.bad_payload}")
                disp.show(last_img, status)

            stop = False
            for key in disp.poll_keys():
                if key in ("q", "Q"):
                    stop = True
                elif key in ("w", "W") and last_img is not None:
                    print("Salvato:", save_frame(last_img, args.save_dir))
                elif key in FORWARD:
                    try:
                        ser.write(FORWARD[key].encode())
                    except Exception as exc:
                        print("Invio comando fallito:", exc)
            if stop or not disp.alive():
                break

            if not grab.is_alive():
                print("Lettura seriale interrotta.")
                exit_code = 1
                break

            if got is None:
                time.sleep(0.002)

            if args.stats > 0 and time.monotonic() - t_stats >= args.stats:
                t_stats = time.monotonic()
                print(f"[stat] {grab.fps():.1f} fps  frame:{grab.n_frames}  "
                      f"persi:{grab.n_lost}  tardivi:{grab.n_late}  "
                      f"hdr_err:{grab.reader.bad_header}  "
                      f"crc_err:{grab.reader.bad_payload}")
    except KeyboardInterrupt:
        pass
    finally:
        grab.stop()
        if disp is not None:
            disp.close()
        try:
            ser.close()
        except Exception:
            pass
        print(f"Totale: {grab.n_frames} frame, {grab.n_lost} persi, "
              f"{grab.reader.bad_payload} checksum errati.")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
